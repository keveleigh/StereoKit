#include "audio.h"
#include "input.h"
#include "../asset_types/sound.h"
#include "../sk_memory.h"
#include "../sk_math.h"
#include "../sk_math_dx.h"
#include "../libraries/atomic_util.h"

#include <string.h>

using namespace DirectX;

namespace sk {

///////////////////////////////////////////

// Play commands only: stops travel through the voice's atomic stop_request
// param instead, which can't overflow or fail.
struct au_cmd_t {
	int16_t  slot;
	uint16_t id;
};

// Pushed by the audio thread when a voice finishes, its slot is stolen, or
// it's stopped. Drained on main where releasing and freeing is safe. A -1
// slot is steal cleanup, it frees resources without touching voice state.
struct au_ret_t {
	sound_t     sound;
	ma_decoder* decoder;
	ma_pcm_rb*  rb;
	float*      ring_data;
	int16_t     slot;
};

#define AU_RET_RING_SIZE 512

au_voice_t au_voices[AU_VOICE_COUNT] = {};
bool32_t   au_offline = false;

// SPSC rings. Head/tail are free-running counters, masked on use. All
// arithmetic is done in uint32_t to keep wraparound defined.
static au_cmd_t au_cmd_ring[AU_CMD_RING_SIZE] = {};
static int32_t  au_cmd_head = 0; // Main writes
static int32_t  au_cmd_tail = 0; // Audio writes
static au_ret_t au_ret_ring[AU_RET_RING_SIZE] = {};
static int32_t  au_ret_head = 0; // Audio writes
static int32_t  au_ret_tail = 0; // Main writes

// Listener pose, published field-by-field through the atomics. A read may
// mix components from two consecutive frames, which is at most one frame
// of head motion for one mix block - individual fields never tear.
static pose_t au_listener_pose = {{0,0,0}, {0,0,0,1}};

static uint64_t au_mix_temp_size = 0;
static float*   au_mix_temp      = nullptr;
static bool     au_mix_truncate_warned = false;

// Pitch resampling reads sources at up to AU_PITCH_MAX rate.
static uint64_t au_resample_temp_size = 0;
static float*   au_resample_temp      = nullptr;

// The first order ambisonic bus, interleaved ambiX ACN order - one W, Y,
// Z, X float4 per frame, so encode and decode are single SIMD ops per
// sample. Spatial voices encode into it, one decode per block renders it
// binaural. When nothing has touched the bus and the decode filters have
// rung down, the decode is skipped entirely - a head-locked-only or idle
// mix costs no spatialization work at all.
static float*   au_foa         = nullptr;
static bool     au_foa_touched = false; // Any encode this block, audio thread
static int32_t  au_foa_tail    = 0;     // Decode ring-down frames left

///////////////////////////////////////////
// Binaural decode: 8 virtual speakers at the cube corners, sampled with
// cardioid pickups. The symmetric layout makes the coherent sum exactly
// 1.0 in every direction, so calibration passes straight through. Each
// speaker feeds both ears through fixed head-relative spherical-head
// filters; measured HRIRs can replace these later without touching
// anything upstream.
//
// The 16 ear paths (8 speakers x 2 ears) run structure-of-arrays in four
// SIMD lane groups - left ears of speakers 0-3 and 4-7, then the same for
// right - each path padded to a uniform pipeline of head-shadow one-pole
// plus three biquad stages (identity where unused), so the filter bank
// steps 4 paths per op. Only the fractional ITD delays stay scalar, they
// need per-lane buffer indexing.

#define AU_SPEAKER_COUNT 8
#define AU_PATH_COUNT    (AU_SPEAKER_COUNT * 2)
#define AU_PATH_GROUPS   (AU_PATH_COUNT / 4)
#define AU_DECODE_STAGES 3
#define AU_EAR_DELAY_MAX 48

// Interaural decorrelator sizes: Schroeder allpass delays per [ear][section],
// distinct primes so the ears' phase responses never line up, sums matched so
// neither ear leads on average and the image stays centered.
#define AU_DECO_SECTIONS 2
#define AU_DECO_MAX      256
#define AU_DECO_GAIN     0.5f
static const int32_t au_deco_len[2][AU_DECO_SECTIONS] = {{89, 199}, {107, 181}};

struct au_biquad_t {
	float b0, b1, b2, a1, a2;
};

struct au_decode_t {
	vec3     speaker_dir[AU_SPEAKER_COUNT]; // Head-relative
	// Biquad coefficients and filter states, [stage][lane group]
	XMVECTOR b0[AU_DECODE_STAGES][AU_PATH_GROUPS], b1[AU_DECODE_STAGES][AU_PATH_GROUPS];
	XMVECTOR b2[AU_DECODE_STAGES][AU_PATH_GROUPS], a1[AU_DECODE_STAGES][AU_PATH_GROUPS];
	XMVECTOR a2[AU_DECODE_STAGES][AU_PATH_GROUPS];
	XMVECTOR x1[AU_DECODE_STAGES][AU_PATH_GROUPS], x2[AU_DECODE_STAGES][AU_PATH_GROUPS];
	XMVECTOR y1[AU_DECODE_STAGES][AU_PATH_GROUPS], y2[AU_DECODE_STAGES][AU_PATH_GROUPS];
	// Head shadow one-pole at ~1.5kHz, mixed in by lateralness, 0 = bypass
	XMVECTOR shadow_wet  [AU_PATH_GROUPS];
	XMVECTOR shadow_state[AU_PATH_GROUPS];
	// Woodworth ITD as a fractional delay per lane, 0 = bypass
	float    delay    [AU_PATH_COUNT];
	float    delay_buf[AU_PATH_COUNT][AU_EAR_DELAY_MAX];
	int32_t  delay_at;
	// Diffuse-field interaural coherence: a real diffuse field is coherent
	// between the ears only below ~c/(2*ear spacing), but the decode alone
	// leaves the ears correlated at every frequency - which images as a
	// narrow band in the head instead of enveloping width. Above the
	// crossover each ear runs its own allpass chain, below it passes clean.
	float    deco_k;         // Crossover one-pole coefficient, ~980Hz
	float    deco_lp [2];    // Crossover state per ear
	float    deco_buf[2][AU_DECO_SECTIONS][AU_DECO_MAX];
	int32_t  deco_at [2][AU_DECO_SECTIONS];
};
static au_decode_t au_decode = {};

// RBJ cookbook peaking EQ, gain_db < 0 makes a dip.
static au_biquad_t au_biquad_peaking(float hz, float q, float gain_db) {
	float A     = powf(10.0f, gain_db / 40.0f);
	float w0    = 6.2831853f * hz / AU_SAMPLE_RATE;
	float alpha = sinf(w0) / (2.0f * q);
	float a0    = 1 + alpha / A;
	au_biquad_t r = {};
	r.b0 = (1 + alpha * A) / a0;
	r.b1 = (-2 * cosf(w0)) / a0;
	r.b2 = (1 - alpha * A) / a0;
	r.a1 = (-2 * cosf(w0)) / a0;
	r.a2 = (1 - alpha / A) / a0;
	return r;
}

// RBJ cookbook high shelf, S = 1.
static au_biquad_t au_biquad_highshelf(float hz, float gain_db) {
	float A     = powf(10.0f, gain_db / 40.0f);
	float w0    = 6.2831853f * hz / AU_SAMPLE_RATE;
	float cw    = cosf(w0);
	float alpha = sinf(w0) / 2.0f * 1.4142135f;
	float sq    = 2.0f * sqrtf(A) * alpha;
	float a0    = (A + 1) - (A - 1) * cw + sq;
	au_biquad_t r = {};
	r.b0 = ( A * ((A + 1) + (A - 1) * cw + sq)) / a0;
	r.b1 = (-2 * A * ((A - 1) + (A + 1) * cw)) / a0;
	r.b2 = ( A * ((A + 1) + (A - 1) * cw - sq)) / a0;
	r.a1 = ( 2 *     ((A - 1) - (A + 1) * cw)) / a0;
	r.a2 = (     (A + 1) - (A - 1) * cw - sq ) / a0;
	return r;
}

static void au_speakers_init() {
	memset(&au_decode, 0, sizeof(au_decode));

	// Coherence crossover at c/(2 * ear spacing) - it falls out of the
	// head's size, the same 0.0875m radius the ITD model uses.
	float fc = 343.0f / (4.0f * 0.0875f);
	float RC = 1.0f / (6.2831853f * fc);
	float dt = 1.0f / AU_SAMPLE_RATE;
	au_decode.deco_k = dt / (RC + dt);

	// Coefficients build per-lane in scalar staging, packed at the end.
	float b0[AU_DECODE_STAGES][AU_PATH_COUNT], b1[AU_DECODE_STAGES][AU_PATH_COUNT];
	float b2[AU_DECODE_STAGES][AU_PATH_COUNT], a1[AU_DECODE_STAGES][AU_PATH_COUNT];
	float a2[AU_DECODE_STAGES][AU_PATH_COUNT];
	float shadow[AU_PATH_COUNT] = {};
	for (int32_t s = 0; s < AU_DECODE_STAGES; s++) {
		for (int32_t p = 0; p < AU_PATH_COUNT; p++) {
			b0[s][p] = 1; // Identity biquad for unused stages
			b1[s][p] = 0; b2[s][p] = 0; a1[s][p] = 0; a2[s][p] = 0;
		}
	}

	// Cube corners: az +-45/+-135, el +-35.26. SK frame, -Z forward.
	const float el_s = 0.5773503f; // sin(35.26)
	const float el_c = 0.8164966f; // cos(35.26)
	int32_t at = 0;
	for (int32_t up = 0; up < 2; up++) {
	for (int32_t corner = 0; corner < 4; corner++) {
		float az  = (45.0f + 90.0f * corner) * (3.14159265f / 180.0f);
		vec3  dir = vec3 {
			sinf(az) * el_c,
			up == 0 ? el_s : -el_s,
			-cosf(az) * el_c };
		au_decode.speaker_dir[at] = dir;

		// Woodworth ITD and head shadow on the contralateral ear, scaled
		// by how lateral the speaker sits. Lane p = ear * 8 + speaker.
		float   lateral = fabsf(dir.x);
		float   theta   = asinf(lateral);
		float   itd     = (0.0875f / 343.0f) * (theta + sinf(theta)) * AU_SAMPLE_RATE;
		int32_t contra  = dir.x > 0 ? 0 : 1;
		au_decode.delay[contra * AU_SPEAKER_COUNT + at] = itd;
		shadow         [contra * AU_SPEAKER_COUNT + at] = 0.6f * lateral;

		for (int32_t ear = 0; ear < 2; ear++) {
			int32_t     lane  = ear * AU_SPEAKER_COUNT + at;
			au_biquad_t f[AU_DECODE_STAGES];
			int32_t     count = 0;
			// Elevation: a pinna-style dip, deeper and higher for up.
			if (up == 0) {
				f[count++] = au_biquad_peaking(8000, 4, -8);
			} else {
				f[count++] = au_biquad_peaking  (6500, 4, -4);
				f[count++] = au_biquad_highshelf(6000, -2);
			}
			// Behind loses a little sparkle.
			if (dir.z > 0)
				f[count++] = au_biquad_highshelf(4000, -3);

			for (int32_t s = 0; s < count; s++) {
				b0[s][lane] = f[s].b0; b1[s][lane] = f[s].b1; b2[s][lane] = f[s].b2;
				a1[s][lane] = f[s].a1; a2[s][lane] = f[s].a2;
			}
		}
		at += 1;
	} }

	for (int32_t g = 0; g < AU_PATH_GROUPS; g++) {
		for (int32_t s = 0; s < AU_DECODE_STAGES; s++) {
			au_decode.b0[s][g] = XMVectorSet(b0[s][g*4], b0[s][g*4+1], b0[s][g*4+2], b0[s][g*4+3]);
			au_decode.b1[s][g] = XMVectorSet(b1[s][g*4], b1[s][g*4+1], b1[s][g*4+2], b1[s][g*4+3]);
			au_decode.b2[s][g] = XMVectorSet(b2[s][g*4], b2[s][g*4+1], b2[s][g*4+2], b2[s][g*4+3]);
			au_decode.a1[s][g] = XMVectorSet(a1[s][g*4], a1[s][g*4+1], a1[s][g*4+2], a1[s][g*4+3]);
			au_decode.a2[s][g] = XMVectorSet(a2[s][g*4], a2[s][g*4+1], a2[s][g*4+2], a2[s][g*4+3]);
		}
		au_decode.shadow_wet[g] = XMVectorSet(shadow[g*4], shadow[g*4+1], shadow[g*4+2], shadow[g*4+3]);
	}
}

// Decodes the FOA bus additively into interleaved stereo. Rotation is
// applied to the pickup directions rather than the field, the ear filters
// stay head-relative - which is also correct under head roll.
static void audio_decode_foa(float* output, ma_uint32 frame_count, pose_t head) {
	// Cardioid pickup gains for the rotated speaker directions, speakers
	// 0-3 and 4-7 as the two lane groups. SK -> ambisonic axes: +X front.
	float gw = (2.0f / AU_SPEAKER_COUNT) * 0.5f;
	float gys[AU_SPEAKER_COUNT], gzs[AU_SPEAKER_COUNT], gxs[AU_SPEAKER_COUNT];
	for (int32_t s = 0; s < AU_SPEAKER_COUNT; s++) {
		vec3 u = head.orientation * au_decode.speaker_dir[s];
		gxs[s] = gw * -u.z;
		gys[s] = gw * -u.x;
		gzs[s] = gw *  u.y;
	}
	XMVECTOR gwv  = XMVectorReplicate(gw);
	XMVECTOR gyA  = XMVectorSet(gys[0], gys[1], gys[2], gys[3]), gyB = XMVectorSet(gys[4], gys[5], gys[6], gys[7]);
	XMVECTOR gzA  = XMVectorSet(gzs[0], gzs[1], gzs[2], gzs[3]), gzB = XMVectorSet(gzs[4], gzs[5], gzs[6], gzs[7]);
	XMVECTOR gxA  = XMVectorSet(gxs[0], gxs[1], gxs[2], gxs[3]), gxB = XMVectorSet(gxs[4], gxs[5], gxs[6], gxs[7]);
	XMVECTOR ones = XMVectorSplatOne();
	XMVECTOR sh_k = XMVectorReplicate(0.178f);

	for (ma_uint32 i = 0; i < frame_count; i++) {
		// All 8 speaker signals from 8 multiply-adds on the broadcast
		// ambisonic channels. The bus frame is W, Y, Z, X.
		XMVECTOR foa  = XMLoadFloat4((XMFLOAT4*)(au_foa + i*4));
		XMVECTOR ambw = XMVectorSplatX(foa), amby = XMVectorSplatY(foa);
		XMVECTOR ambz = XMVectorSplatZ(foa), ambx = XMVectorSplatW(foa);
		XMVECTOR sigA = XMVectorMultiply   (gwv, ambw);
		         sigA = XMVectorMultiplyAdd(gyA, amby, sigA);
		         sigA = XMVectorMultiplyAdd(gzA, ambz, sigA);
		         sigA = XMVectorMultiplyAdd(gxA, ambx, sigA);
		XMVECTOR sigB = XMVectorMultiply   (gwv, ambw);
		         sigB = XMVectorMultiplyAdd(gyB, amby, sigB);
		         sigB = XMVectorMultiplyAdd(gzB, ambz, sigB);
		         sigB = XMVectorMultiplyAdd(gxB, ambx, sigB);

		// Both ears start from the same speaker signal, the paths diverge
		// in the filters. ITD delays run scalar through lane staging.
		XMFLOAT4A st[AU_PATH_GROUPS];
		XMStoreFloat4A(&st[0], sigA); XMStoreFloat4A(&st[1], sigB);
		XMStoreFloat4A(&st[2], sigA); XMStoreFloat4A(&st[3], sigB);
		float*  lanes  = (float*)st;
		int32_t at_idx = au_decode.delay_at;
		for (int32_t p = 0; p < AU_PATH_COUNT; p++) {
			float d = au_decode.delay[p];
			if (d <= 0) continue;
			float* buf = au_decode.delay_buf[p];
			buf[at_idx] = lanes[p];
			float read_at = (float)at_idx - d;
			if (read_at < 0) read_at += AU_EAR_DELAY_MAX;
			int32_t i0 = (int32_t)read_at;
			int32_t i1 = i0 + 1 >= AU_EAR_DELAY_MAX ? 0 : i0 + 1;
			lanes[p] = math_lerp(buf[i0], buf[i1], read_at - (float)i0);
		}
		au_decode.delay_at = at_idx + 1 >= AU_EAR_DELAY_MAX ? 0 : at_idx + 1;

		XMVECTOR sig[AU_PATH_GROUPS];
		for (int32_t g = 0; g < AU_PATH_GROUPS; g++) {
			XMVECTOR x = XMLoadFloat4A(&st[g]);

			// Head shadow one-pole, bypass lanes have wet 0.
			XMVECTOR sh = au_decode.shadow_state[g];
			sh = XMVectorMultiplyAdd(sh_k, XMVectorSubtract(x, sh), sh);
			au_decode.shadow_state[g] = sh;
			x = XMVectorMultiplyAdd(au_decode.shadow_wet[g], XMVectorSubtract(sh, x), x);

			// Three direct-form-I biquad stages, 4 paths per step.
			for (int32_t s = 0; s < AU_DECODE_STAGES; s++) {
				XMVECTOR y = XMVectorMultiply(au_decode.b0[s][g], x);
				y = XMVectorMultiplyAdd             (au_decode.b1[s][g], au_decode.x1[s][g], y);
				y = XMVectorMultiplyAdd             (au_decode.b2[s][g], au_decode.x2[s][g], y);
				y = XMVectorNegativeMultiplySubtract(au_decode.a1[s][g], au_decode.y1[s][g], y);
				y = XMVectorNegativeMultiplySubtract(au_decode.a2[s][g], au_decode.y2[s][g], y);
				au_decode.x2[s][g] = au_decode.x1[s][g]; au_decode.x1[s][g] = x;
				au_decode.y2[s][g] = au_decode.y1[s][g]; au_decode.y1[s][g] = y;
				x = y;
			}
			sig[g] = x;
		}

		// The coherence split: lows pass, highs decorrelate. The allpasses
		// are unity magnitude, so per-ear energy and calibration hold.
		float lr[2] = {
			XMVectorGetX(XMVector4Dot(XMVectorAdd(sig[0], sig[1]), ones)),
			XMVectorGetX(XMVector4Dot(XMVectorAdd(sig[2], sig[3]), ones)) };
		for (int32_t e = 0; e < 2; e++) {
			float lp = au_decode.deco_lp[e] + au_decode.deco_k * (lr[e] - au_decode.deco_lp[e]);
			au_decode.deco_lp[e] = lp;
			float hi = lr[e] - lp;
			for (int32_t a = 0; a < AU_DECO_SECTIONS; a++) {
				float*  buf = au_decode.deco_buf[e][a];
				int32_t at2 = au_decode.deco_at [e][a];
				float   d   = buf[at2];
				float   v   = hi + AU_DECO_GAIN * d;
				hi          = d  - AU_DECO_GAIN * v;
				buf[at2]    = v;
				au_decode.deco_at[e][a] = at2 + 1 >= au_deco_len[e][a] ? 0 : at2 + 1;
			}
			lr[e] = lp + hi;
		}
		output[i*2  ] += lr[0];
		output[i*2+1] += lr[1];
	}
}

///////////////////////////////////////////
// Direct binaural path for point sources: per-voice exact ITD, head
// shadow, and parametric elevation/back voicing. Precision cues the FOA
// bus can't provide - a source's own interaural delay is what makes it
// pointable. Spread crossfades a voice from here onto the bus.

// A/B hook: force everything through the bus for listening comparisons.
static int32_t au_force_bus = 0;
void audio_test_force_bus(bool32_t enable) {
	atomic_store_i32(&au_force_bus, enable ? 1 : 0);
}

// Brown-Duda spherical head per-ear delay in samples, continuous across
// the median plane. cos_e is the cosine of the angle between the source
// direction and that ear's axis; only the difference between ears is
// audible, the common offset just keeps delays causal.
static inline float au_ear_delay(float cos_e) {
	const float r = (0.0875f / 343.0f) * AU_SAMPLE_RATE;
	if (cos_e >= 0) return r * (1.0f - cos_e);
	return r * (1.0f + acosf(fmaxf(-1.0f, cos_e)) - 1.5707963f);
}

// Direct-form-I biquad with caller-owned state, st = [x1 x2 y1 y2].
static inline float au_biquad_apply(const au_biquad_t* c, float* st, float x) {
	float y = c->b0*x + c->b1*st[0] + c->b2*st[1] - c->a1*st[2] - c->a2*st[3];
	st[1] = st[0]; st[0] = x;
	st[3] = st[2]; st[2] = y;
	return y;
}

// Soft clip limiter, transparent below the -1dBFS knee.
static inline float au_limit(float x) {
	const float knee = 0.89f;
	float a = fabsf(x);
	if (a <= knee) return x;
	float y = knee + (1.0f - knee) * tanhf((a - knee) / (1.0f - knee));
	return x < 0 ? -y : y;
}

// Master/bus trims and the output meter, all atomic f32.
static float au_master_volume = 1;
static float au_bus_volumes[AU_BUS_COUNT] = {1, 1, 1, 1};
static float au_output_dbfs  = -120;

///////////////////////////////////////////

void audio_set_volume(float volume) {
	atomic_store_f32(&au_master_volume, fmaxf(0, fminf(1, volume)));
}
float audio_get_volume() {
	return atomic_load_f32(&au_master_volume);
}
void audio_set_bus_volume(sound_bus_ bus, float volume) {
	if (bus < 0 || bus >= AU_BUS_COUNT) { log_errf("audio_set_bus_volume: invalid bus %d", bus); return; }
	atomic_store_f32(&au_bus_volumes[bus], fmaxf(0, fminf(1, volume)));
}
float audio_get_bus_volume(sound_bus_ bus) {
	if (bus < 0 || bus >= AU_BUS_COUNT) return 0;
	return atomic_load_f32(&au_bus_volumes[bus]);
}
float audio_get_output_decibels() {
	return atomic_load_f32(&au_output_dbfs);
}

///////////////////////////////////////////

void audio_listener_publish(pose_t pose) {
	atomic_store_f32(&au_listener_pose.position.x,    pose.position.x);
	atomic_store_f32(&au_listener_pose.position.y,    pose.position.y);
	atomic_store_f32(&au_listener_pose.position.z,    pose.position.z);
	atomic_store_f32(&au_listener_pose.orientation.x, pose.orientation.x);
	atomic_store_f32(&au_listener_pose.orientation.y, pose.orientation.y);
	atomic_store_f32(&au_listener_pose.orientation.z, pose.orientation.z);
	atomic_store_f32(&au_listener_pose.orientation.w, pose.orientation.w);
}

pose_t audio_listener_get() {
	pose_t result;
	result.position.x    = atomic_load_f32(&au_listener_pose.position.x);
	result.position.y    = atomic_load_f32(&au_listener_pose.position.y);
	result.position.z    = atomic_load_f32(&au_listener_pose.position.z);
	result.orientation.x = atomic_load_f32(&au_listener_pose.orientation.x);
	result.orientation.y = atomic_load_f32(&au_listener_pose.orientation.y);
	result.orientation.z = atomic_load_f32(&au_listener_pose.orientation.z);
	result.orientation.w = atomic_load_f32(&au_listener_pose.orientation.w);
	return result;
}

///////////////////////////////////////////

void audio_mix_init(int32_t period_frames) {
	// Sizes are in frames, buffers allocate 4x for interleaved ambisonics.
	au_mix_temp_size = (uint64_t)period_frames + AU_SAMPLE_BUFFER_SIZE * 2;
	au_mix_temp      = sk_malloc_t(float, au_mix_temp_size * 4);
	memset(au_mix_temp, 0, sizeof(float) * au_mix_temp_size * 4);

	au_resample_temp_size = ((uint64_t)(period_frames * AU_PITCH_MAX) + AU_SAMPLE_BUFFER_SIZE * 2 + 8) * 4;
	au_resample_temp      = sk_malloc_t(float, au_resample_temp_size);
	memset(au_resample_temp, 0, sizeof(float) * au_resample_temp_size);

	au_foa = sk_malloc_t(float, au_mix_temp_size * 4);
	memset(au_foa, 0, sizeof(float) * au_mix_temp_size * 4);

	au_speakers_init();
	audio_listener_publish(pose_identity);
}

///////////////////////////////////////////

static void voice_free_resources(sound_t sound, ma_decoder* decoder, ma_pcm_rb* rb, float* ring_data) {
	if (decoder != nullptr) {
		ma_decoder_uninit(decoder);
		sk_free(decoder);
	}
	if (rb != nullptr) {
		ma_pcm_rb_uninit(rb);
		sk_free(rb);
	}
	sk_free(ring_data);
	sound_release(sound);
}

///////////////////////////////////////////

// Estimated audibility of a voice at the listener's ear. Sounds that
// don't scale with distance - ambisonic fields and head-locked audio -
// are always exactly as audible as declared, so they rank as infinite:
// nothing displaces them, and they're never dormant.
static float voice_audibility(const au_voice_t* voice, vec3 listener_pos) {
	// An atomic read because activation writes it on the audio thread.
	// The pointer stays valid regardless: frees happen on this thread's
	// own drain, and can't precede this scan.
	_sound_t* snd = (_sound_t*)atomic_load_ptr(&voice->sound);
	if (snd == nullptr) snd = (_sound_t*)atomic_load_ptr(&voice->pending_sound);
	if (snd == nullptr) return 0;

	int32_t flags = atomic_load_i32(&voice->params.flags);
	if (snd->channels != sound_channels_mono || (flags & sound_flags_head_locked) != 0)
		return 1e30f;

	vec3 pos = {
		atomic_load_f32(&voice->params.pos_x),
		atomic_load_f32(&voice->params.pos_y),
		atomic_load_f32(&voice->params.pos_z) };
	float dist = fmaxf(AU_MIN_DISTANCE, vec3_magnitude(listener_pos - pos));
	float db   = atomic_load_f32(&snd->decibels);
	return decibels_to_signal(db - 20.0f*log10f(dist))
	     * atomic_load_f32(&voice->params.volume);
}

static float sound_play_audibility(sound_t sound, vec3 at, float volume, vec3 listener_pos) {
	if (sound->channels != sound_channels_mono) return 1e30f;
	float dist = fmaxf(AU_MIN_DISTANCE, vec3_magnitude(listener_pos - at));
	return decibels_to_signal(atomic_load_f32(&sound->decibels) - 20.0f*log10f(dist)) * volume;
}

int16_t audio_voice_reserve(sound_t sound, vec3 at, float volume) {
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		if (atomic_cas_i32(&au_voices[i].state, au_voice_free, au_voice_reserved)) {
			au_voices[i].id += 1;
			return i;
		}
	}

	// No free slot - finished voices may just be waiting on their drain,
	// which is main-thread work, and this is the main thread.
	audio_mix_drain_returns();
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		if (atomic_cas_i32(&au_voices[i].state, au_voice_free, au_voice_reserved)) {
			au_voices[i].id += 1;
			return i;
		}
	}

	// Genuinely full. Stealing is about audibility: find the quietest
	// playing voice, and take its slot only if the incoming sound would
	// be *more* audible - otherwise the incoming sound is the least
	// audible thing here, and refusing it is the least-bad choice.
	// Distance-less voices rank infinite and are never victims.
	pose_t  head      = audio_listener_get();
	int16_t best_slot = -1;
	float   best_gain = 1e30f;
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		if (atomic_load_i32(&au_voices[i].state) != au_voice_playing) continue;
		float gain = voice_audibility(&au_voices[i], head.position);
		if (gain < best_gain) { best_gain = gain; best_slot = i; }
	}
	if (best_slot < 0 || sound_play_audibility(sound, at, volume, head.position) <= best_gain)
		return -1;

	if (!atomic_cas_i32(&au_voices[best_slot].state, au_voice_playing, au_voice_reserved))
		return -1; // Lost a race with the audio thread, just refuse this play.

	// The displaced play's resources stay untouched until the audio thread
	// hands them back through the return ring at activation.
	au_voices[best_slot].id += 1;
	return best_slot;
}

///////////////////////////////////////////

// Main thread, once per frame: rank the active voices by audibility and
// grant the top AU_MIX_VOICES the mix budget. Everything else goes
// dormant in place - still alive, still owning its slot and handle, its
// cursor frozen - and resumes seamlessly when it ranks back in.
void audio_voice_rank() {
	pose_t  head = audio_listener_get();
	float   gains[AU_VOICE_COUNT];
	int32_t active = 0;

	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		int32_t state = atomic_load_i32(&au_voices[i].state);
		if (state != au_voice_playing && state != au_voice_reserved) {
			gains[i] = -1;
			continue;
		}
		gains[i] = voice_audibility(&au_voices[i], head.position);
		active  += 1;
	}

	// Under budget: everyone plays, skip the ranking work.
	if (active <= AU_MIX_VOICES) {
		for (int16_t i = 0; i < AU_VOICE_COUNT; i++)
			if (gains[i] >= 0) atomic_store_i32(&au_voices[i].audible, 1);
		return;
	}

	// The budget's threshold gain: the AU_MIX_VOICES'th loudest. A simple
	// selection over 128 entries, this is small.
	float sorted[AU_VOICE_COUNT];
	int32_t n = 0;
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++)
		if (gains[i] >= 0) sorted[n++] = gains[i];
	for (int32_t i = 1; i < n; i++) {
		float   v = sorted[i];
		int32_t j = i - 1;
		while (j >= 0 && sorted[j] < v) { sorted[j+1] = sorted[j]; j -= 1; }
		sorted[j+1] = v;
	}
	float threshold = sorted[AU_MIX_VOICES - 1];

	int32_t granted = 0;
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		if (gains[i] < 0) continue;
		bool in = gains[i] >= threshold && granted < AU_MIX_VOICES;
		if (in) granted += 1;
		atomic_store_i32(&au_voices[i].audible, in ? 1 : 0);
	}
}

///////////////////////////////////////////

// The command ring publish is the release point for every pending_* field
// main wrote during reservation, the audio thread acquires them via head.
bool audio_voice_submit(int16_t slot) {
	uint32_t head = (uint32_t)atomic_load_i32    (&au_cmd_head);
	uint32_t tail = (uint32_t)atomic_load_i32_acq(&au_cmd_tail);
	if (head - tail >= AU_CMD_RING_SIZE) return false;

	au_cmd_t cmd = {};
	cmd.slot = slot;
	cmd.id   = au_voices[slot].id;
	au_cmd_ring[head % AU_CMD_RING_SIZE] = cmd;
	atomic_store_i32_rel(&au_cmd_head, (int32_t)(head + 1));
	return true;
}

///////////////////////////////////////////

// Audio thread. Hand a voice's resources to main for freeing.
static void audio_ret_push(sound_t sound, ma_decoder* decoder, ma_pcm_rb* rb, float* ring_data, int16_t slot) {
	uint32_t head = (uint32_t)atomic_load_i32    (&au_ret_head);
	uint32_t tail = (uint32_t)atomic_load_i32_acq(&au_ret_tail);
	if (head - tail >= AU_RET_RING_SIZE) {
		// Can't free from this thread, and by design this can't happen: the
		// ring outsizes every event source combined.
		return;
	}
	au_ret_t* entry = &au_ret_ring[head % AU_RET_RING_SIZE];
	entry->sound     = sound;
	entry->decoder   = decoder;
	entry->rb        = rb;
	entry->ring_data = ring_data;
	entry->slot      = slot;
	atomic_store_i32_rel(&au_ret_head, (int32_t)(head + 1));
}

// Audio thread. The common finish: hand off and mark for the drain.
static void voice_finish(au_voice_t* voice, int16_t slot) {
	if (atomic_cas_i32(&voice->state, au_voice_playing, au_voice_finished))
		audio_ret_push(voice->sound, voice->stream_decoder, voice->stream_ring, voice->stream_ring_data, slot);
	// A failed CAS means the slot was just stolen, activation cleans up.
}

// Main thread, called from audio_step and shutdown.
void audio_mix_drain_returns() {
	uint32_t tail = (uint32_t)atomic_load_i32    (&au_ret_tail);
	uint32_t head = (uint32_t)atomic_load_i32_acq(&au_ret_head);
	while (tail != head) {
		au_ret_t* entry = &au_ret_ring[tail % AU_RET_RING_SIZE];
		voice_free_resources(entry->sound, entry->decoder, entry->rb, entry->ring_data);
		if (entry->slot >= 0) {
			// Finished voices are off-limits to the audio thread, clearing
			// the freed pointers here is safe and keeps shutdown single-path.
			au_voice_t* voice = &au_voices[entry->slot];
			voice->sound            = nullptr;
			voice->stream_decoder   = nullptr;
			voice->stream_ring      = nullptr;
			voice->stream_ring_data = nullptr;
			atomic_cas_i32(&voice->state, au_voice_finished, au_voice_free);
		}
		tail += 1;
	}
	atomic_store_i32_rel(&au_ret_tail, (int32_t)tail);
}

///////////////////////////////////////////

// Audio thread. Swap the pending play in, handing any displaced resources
// (a stolen slot's previous play) back to main first.
static void voice_activate(au_voice_t* voice) {
	if (voice->sound != nullptr)
		audio_ret_push(voice->sound, voice->stream_decoder, voice->stream_ring, voice->stream_ring_data, -1);

	// Atomic stores, the audibility scans on main read both pointers.
	atomic_store_ptr(&voice->sound, voice->pending_sound);
	atomic_store_ptr(&voice->pending_sound, nullptr);
	voice->cursor           = voice->pending_cursor;
	voice->stream_decoder   = voice->pending_decoder;
	voice->stream_ring      = voice->pending_ring;
	voice->stream_ring_data = voice->pending_ring_data;
	voice->pending_decoder  = nullptr;
	voice->pending_ring     = nullptr;
	voice->pending_ring_data= nullptr;
	voice->delay_left       = voice->pending_delay;
	voice->bus              = voice->pending_bus;
	voice->resample_frac    = 0;
	memset(voice->resample_last, 0, sizeof(voice->resample_last));
	memset(voice->lpf_state,     0, sizeof(voice->lpf_state));
	memset(voice->dir_ring,   0, sizeof(voice->dir_ring));
	memset(voice->dir_filter, 0, sizeof(voice->dir_filter));
	voice->dir_ring_at      = 0;
	voice->dir_delay[0]     = -1;
	voice->dir_delay[1]     = -1;
	voice->dir_shadow[0]    = 0;
	voice->dir_shadow[1]    = 0;
	atomic_store_i32(&voice->audible, 1);
	// Release so main's prefetch sees the stream fields once it's playing.
	atomic_store_i32_rel(&voice->state, au_voice_playing);
}

///////////////////////////////////////////

// Audio thread, start of each block.
static void audio_drain_commands() {
	uint32_t tail = (uint32_t)atomic_load_i32    (&au_cmd_tail);
	uint32_t head = (uint32_t)atomic_load_i32_acq(&au_cmd_head);
	while (tail != head) {
		au_cmd_t*   cmd   = &au_cmd_ring[tail % AU_CMD_RING_SIZE];
		au_voice_t* voice = &au_voices[cmd->slot];
		if (voice->id == cmd->id && atomic_load_i32(&voice->state) == au_voice_reserved)
			voice_activate(voice);
		tail += 1;
	}
	atomic_store_i32_rel(&au_cmd_tail, (int32_t)tail);
}

///////////////////////////////////////////

// Audio thread. Reads source-rate *frames* for a voice into dest
// (interleaved for multi-channel sounds), honoring looping. Underruns on
// a live source zero-fill so the voice stays alive, a short return means
// the source is done for good. The cursor is stored atomically for the
// main thread's getter.
static ma_uint64 voice_read_source(au_voice_t* voice, float* dest, ma_uint64 frames, bool loop) {
	_sound_t* sound = voice->sound;
	int32_t   ch    = sound_channel_count(sound->channels);
	switch (sound->data_type) {
	case sound_data_pcm: {
		ma_uint64 total  = 0;
		uint64_t  cursor = voice->cursor;
		while (total < frames) {
			if (cursor >= sound->pcm_count) {
				if (!loop || sound->pcm_count == 0) break;
				cursor = 0;
			}
			ma_uint64 read = mini(frames - total, sound->pcm_count - cursor);
			memcpy(dest + total*ch, sound->pcm + cursor*ch, (size_t)(read * ch) * sizeof(float));
			cursor += read;
			total  += read;
		}
		atomic_store_u64(&voice->cursor, cursor);
		return total;
	}
	case sound_data_stream_file: {
		// Looping streamed files is handled by the prefetch re-seeking the
		// decoder, from here it's just a ring that never runs eof. The
		// ring is channel-aware, its counts are frames.
		ma_uint64 read = 0;
		while (read < frames) {
			ma_uint32 request = (ma_uint32)(frames - read);
			void*     from    = nullptr;
			if (ma_pcm_rb_acquire_read(voice->stream_ring, &request, &from) != MA_SUCCESS || request == 0) break;
			memcpy(dest + read*ch, from, (size_t)(request * ch) * sizeof(float));
			ma_pcm_rb_commit_read(voice->stream_ring, request);
			read += request;
		}
		atomic_store_u64(&voice->cursor, voice->cursor + read);
		if (read < frames && atomic_load_i32_acq(&voice->stream_eof) == 0) {
			// Prefetch underrun, not the end of the file. Pad with silence.
			memset(dest + read*ch, 0, (size_t)((frames - read) * ch) * sizeof(float));
			read = frames;
		}
		return read;
	}
	case sound_data_ring: {
		// The public stream API counts samples, convert to frames.
		ma_uint64 read = sound_read_samples(sound, dest, frames * ch) / ch;
		atomic_store_u64(&voice->cursor, voice->cursor + read);
		// Live streams idle at the end of their data instead of finishing.
		memset(dest + read*ch, 0, (size_t)((frames - read) * ch) * sizeof(float));
		return frames;
	}
	default: return 0;
	}
}

///////////////////////////////////////////

// 4-point Catmull-Rom, interpolating between p1 and p2. Flat to near
// Nyquist where linear interpolation shaves the top octave and folds
// images down as inharmonic aliasing.
static inline float au_catmull(float p0, float p1, float p2, float p3, float t) {
	return p1 + 0.5f*t*(p2 - p0 + t*(2*p0 - 5*p1 + 4*p2 - p3 + t*(3*(p1 - p2) + p3 - p0)));
}

// Audio thread. voice_read_source plus Catmull-Rom resampling, covering
// both pitch and source sample rate. resample_last carries the final three
// consumed source frames across blocks so the 4-point window stays
// continuous at block edges. `rate` is source frames per output frame.
static ma_uint64 voice_read(au_voice_t* voice, float* dest, ma_uint64 frames, float rate, bool loop) {
	int32_t ch = sound_channel_count(voice->sound->channels);
	if (rate == 1.0f) {
		ma_uint64 read = voice_read_source(voice, dest, frames, loop);
		// History rolls forward anyway, a live pitch change resumes clean.
		for (int32_t k = 0; k < 3; k++) {
			int64_t f = (int64_t)read - 3 + k;
			for (int32_t c = 0; c < ch; c++)
				voice->resample_last[k][c] = f >= 0
					? dest[f*ch + c]
					: voice->resample_last[k + (int32_t)read][c];
		}
		voice->resample_frac = 0;
		return read;
	}

	ma_uint64 needed = (ma_uint64)(voice->resample_frac + (double)frames * rate) + 2;
	if (needed > au_resample_temp_size / ch - 3)
		needed = au_resample_temp_size / ch - 3;

	// Temp frames 0-2 are history; frame 2 is the last consumed frame, so
	// position 0 interpolates the span from frame 2 to frame 3.
	for (int32_t k = 0; k < 3; k++)
		for (int32_t c = 0; c < ch; c++)
			au_resample_temp[k*ch + c] = voice->resample_last[k][c];
	ma_uint64 src_read = voice_read_source(voice, au_resample_temp + 3*ch, needed, loop);

	// Mono is the hot path - every pitched spatial voice lands here - so it
	// gets its own loop without the per-frame channel iteration.
	ma_uint64 out  = 0;
	float     frac = voice->resample_frac;
	if (ch == 1) {
		// Four outputs per pass: each lane loads its own 4-point window,
		// one transpose turns the windows into p0..p3 across lanes, and
		// the polynomial evaluates 4-wide. The scalar Horner chain is
		// latency-bound, so this wins ~2x despite the shuffles.
		XMVECTOR rate4 = XMVectorSet(0, rate, rate * 2, rate * 3);
		for (; out + 4 <= frames; out += 4) {
			float     base = frac + (float)out * rate;
			ma_uint64 idx3 = (ma_uint64)(base + rate * 3);
			if (idx3 + 2 > src_read) break;

			XMVECTOR p = XMVectorAdd(XMVectorReplicate(base), rate4);
			XMMATRIX m;
			m.r[0] = XMLoadFloat4((XMFLOAT4*)(au_resample_temp + (ma_uint64)(base)            + 1));
			m.r[1] = XMLoadFloat4((XMFLOAT4*)(au_resample_temp + (ma_uint64)(base + rate)     + 1));
			m.r[2] = XMLoadFloat4((XMFLOAT4*)(au_resample_temp + (ma_uint64)(base + rate * 2) + 1));
			m.r[3] = XMLoadFloat4((XMFLOAT4*)(au_resample_temp + idx3                         + 1));
			m = XMMatrixTranspose(m);

			// au_catmull per lane, t = p - floor(p)
			XMVECTOR t    = XMVectorSubtract(p, XMVectorFloor(p));
			XMVECTOR poly = XMVectorSubtract(
				XMVectorAdd(XMVectorScale(m.r[0], 2), XMVectorScale(m.r[2], 4)),
				XMVectorAdd(XMVectorScale(m.r[1], 5), m.r[3]));
			poly = XMVectorMultiplyAdd(t, XMVectorSubtract(XMVectorAdd(XMVectorScale(XMVectorSubtract(m.r[1], m.r[2]), 3), m.r[3]), m.r[0]), poly);
			poly = XMVectorMultiplyAdd(t, poly, XMVectorSubtract(m.r[2], m.r[0]));
			poly = XMVectorMultiply(XMVectorScale(t, 0.5f), poly);
			XMStoreFloat4((XMFLOAT4*)(dest + out), XMVectorAdd(m.r[1], poly));
		}
		for (; out < frames; out++) {
			float     p   = frac + (float)out * rate;
			ma_uint64 idx = (ma_uint64)p;
			if (idx + 2 > src_read) break;
			const float* s = au_resample_temp + idx;
			dest[out] = au_catmull(s[1], s[2], s[3], s[4], p - (float)idx);
		}
	} else {
		for (; out < frames; out++) {
			float     p   = frac + (float)out * rate;
			ma_uint64 idx = (ma_uint64)p;
			if (idx + 2 > src_read) break;
			float t = p - (float)idx;
			for (int32_t c = 0; c < ch; c++) {
				const float* s = au_resample_temp + (idx+1)*ch + c;
				dest[out*ch + c] = au_catmull(s[0], s[ch], s[ch*2], s[ch*3], t);
			}
		}
	}

	float     adv   = frac + (float)out * rate;
	ma_uint64 adv_i = (ma_uint64)adv;
	voice->resample_frac = adv - (float)adv_i;
	if (adv_i <= src_read)
		for (int32_t k = 0; k < 3; k++)
			for (int32_t c = 0; c < ch; c++)
				voice->resample_last[k][c] = au_resample_temp[(adv_i + k)*ch + c];
	return out;
}

///////////////////////////////////////////

// Audio thread. Per-voice mono processing: read at pitch, gain from the
// decibel model, air absorption low-pass, then either a direct add for
// head-locked voices or an encode into the FOA bus at frame_offset. All
// localization happens later at the decode, per-voice spatial cost is
// just the four encode multiply-adds.
static ma_uint32 voice_mix(au_voice_t* voice, pose_t head, float* output, ma_uint32 frame_offset, ma_uint64 frame_count) {
	int32_t flags  = atomic_load_i32(&voice->params.flags);
	float   volume = atomic_load_f32(&voice->params.volume);
	float   pitch  = atomic_load_f32(&voice->params.pitch);
	pitch = pitch <= 0 ? 1 : fminf(AU_PITCH_MAX, fmaxf(AU_PITCH_MIN, pitch));
	bool loop = (flags & sound_flags_loop) != 0;

	// The gain model: the sound declares real-world loudness at 1m, heard
	// loudness attenuates -6dB per distance doubling, and the calibration
	// maps that to signal. Normalization makes declared decibels truthful,
	// and volume/bus/master are plain 0-1 trims.
	float decibels  = atomic_load_f32(&voice->sound->decibels);
	float trim_gain = voice->sound->norm_gain * volume
	                * atomic_load_f32(&au_bus_volumes[voice->bus])
	                * atomic_load_f32(&au_master_volume);

	// Source rate and pitch resample through the same interpolator.
	float rate = pitch * ((float)voice->sound->sample_rate / (float)AU_SAMPLE_RATE);

	// Stereo plays head-locked and untouched, position is ignored.
	if (voice->sound->channels == sound_channels_stereo) {
		float     gain = decibels_to_signal(decibels) * trim_gain;
		ma_uint64 read = voice_read(voice, au_mix_temp, frame_count, rate, loop);
		float     peak = 0;
		float*    out  = output + frame_offset * 2;
		for (ma_uint64 i = 0; i < read; i++) {
			float l = au_mix_temp[i*2], r = au_mix_temp[i*2+1];
			if (peak < l) peak = l;
			if (peak < r) peak = r;
			out[i*2  ] += l*gain;
			out[i*2+1] += r*gain;
		}
		if (peak > atomic_load_f32(&voice->intensity))
			atomic_store_f32(&voice->intensity, peak);
		return (ma_uint32)read;
	}

	// Ambisonic sounds are already a sound field, they add straight onto
	// the bus - world-fixed, so the decode counter-rotates them against
	// the head. Spread still works, fading the directional channels out.
	if (voice->sound->channels == sound_channels_ambisonic1) {
		float     gain      = decibels_to_signal(decibels) * trim_gain;
		float     spread    = fmaxf(0, fminf(1, atomic_load_f32(&voice->params.spread)));
		float     dir_scale = (1.0f - spread) * gain;
		ma_uint64 read      = voice_read(voice, au_mix_temp, frame_count, rate, loop);
		float     peak      = 0;
		XMVECTOR  enc       = XMVectorSet(gain, dir_scale, dir_scale, dir_scale);
		for (ma_uint64 i = 0; i < read; i++) {
			float w = au_mix_temp[i*4];
			if (peak < w) peak = w;
			float* bus = au_foa + (frame_offset+i)*4;
			XMStoreFloat4((XMFLOAT4*)bus, XMVectorMultiplyAdd(
				XMLoadFloat4((XMFLOAT4*)(au_mix_temp + i*4)), enc,
				XMLoadFloat4((XMFLOAT4*)bus)));
		}
		if (read > 0) au_foa_touched = true;
		if (peak > atomic_load_f32(&voice->intensity))
			atomic_store_f32(&voice->intensity, peak);
		return (ma_uint32)read;
	}

	// Head-locked voices skip spatialization entirely: no attenuation, no
	// panning, no filtering, both ears get the sound as-is.
	if (flags & sound_flags_head_locked) {
		float     gain = decibels_to_signal(decibels) * trim_gain;
		ma_uint64 read = voice_read(voice, au_mix_temp, frame_count, rate, loop);
		float     peak = 0;
		float*    out  = output + frame_offset * 2;
		for (ma_uint64 i = 0; i < read; i++) {
			float s = au_mix_temp[i];
			if (peak < s) peak = s;
			out[i*2  ] += s*gain;
			out[i*2+1] += s*gain;
		}
		if (peak > atomic_load_f32(&voice->intensity))
			atomic_store_f32(&voice->intensity, peak);
		return (ma_uint32)read;
	}

	vec3   position = {
		atomic_load_f32(&voice->params.pos_x),
		atomic_load_f32(&voice->params.pos_y),
		atomic_load_f32(&voice->params.pos_z) };

	// Volume from distance is modeled on amplitude's 1/d falloff, not
	// intensity's 1/d^2, since perceived loudness tracks pressure. The
	// minimum distance clamp stands in for "the source is inside your
	// head".
	vec3  dir    = position - head.position;
	float dist2  = vec3_magnitude_sq(dir);
	float dist   = fmaxf(AU_MIN_DISTANCE, sqrtf(dist2));
	float gain   = decibels_to_signal(decibels - 20.0f*log10f(dist)) * trim_gain;

	// Sources at (or inside) the head have no direction, they go diffuse.
	float spread = fmaxf(0, fminf(1, atomic_load_f32(&voice->params.spread)));
	vec3  u      = vec3{0,0,0};
	if (dist2 > 0.0001f) u = dir / sqrtf(dist2);
	else                 spread = 1;

	// Point sources render direct binaural for per-source ITD precision,
	// and crossfade onto the FOA bus as spread widens - diffuse width is
	// what the bus renders well.
	float k_bus = fminf(1.0f, spread / AU_DIRECT_SPREAD);
	if (atomic_load_i32(&au_force_bus)) k_bus = 1;

	ma_uint64 read = voice_read(voice, au_mix_temp, frame_count, rate, loop);

	// Distance rolls off high frequencies, air absorption as four cascaded
	// poles - a single pole's shallow skirt leaks enough far hiss that
	// dense distant fields sound synthetic. Cutoff from distance:
	// https://www.desmos.com/calculator/h5tssewqbl
	// A per-voice override replaces the automatic model. The old behind-
	// the-listener muffling is gone, the back voicing filters cover it.
	float override = atomic_load_f32(&voice->params.cutoff);
	float cutoff   = override > 0 ? override
		: fmaxf(1000, -4000.f * logf(fmaxf(1, dist-3.5f)) + 22200);
	// Poles sit above the requested knee, sqrt(2^(1/4)-1), so the cascade's
	// -3dB point lands at `cutoff` with the skirt falling ~24dB/oct.
	float RC    = 0.43496f / (2.0f * 3.14159265359f * cutoff);
	float dt    = 1.0f / AU_SAMPLE_RATE;
	float alpha = dt / (RC + dt);
	float lp0   = voice->lpf_state[0], lp1 = voice->lpf_state[1];
	float lp2   = voice->lpf_state[2], lp3 = voice->lpf_state[3];
	float peak  = 0;

	// Bus encode is in the *world* ambisonic frame, head rotation applies
	// at decode. SK -> ambisonic axes: +X front, +Y left, +Z up.
	float    dir_scale = (1.0f - spread) * k_bus;
	XMVECTOR enc       = XMVectorSet(gain * k_bus, gain * -u.x * dir_scale,
	                                 gain * u.y * dir_scale, gain * -u.z * dir_scale);

	if (k_bus >= 1.0f) {
		for (ma_uint64 i = 0; i < read; i++) {
			float s = au_mix_temp[i];
			if (peak < s) peak = s;
			lp0 += alpha * (s   - lp0);
			lp1 += alpha * (lp0 - lp1);
			lp2 += alpha * (lp1 - lp2);
			lp3 += alpha * (lp2 - lp3);
			s = lp3;

			float* bus = au_foa + (frame_offset+i)*4;
			XMStoreFloat4((XMFLOAT4*)bus, XMVectorMultiplyAdd(XMVectorReplicate(s), enc, XMLoadFloat4((XMFLOAT4*)bus)));
		}
		voice->lpf_state[0] = lp0; voice->lpf_state[1] = lp1;
		voice->lpf_state[2] = lp2; voice->lpf_state[3] = lp3;
		if (read > 0) au_foa_touched = true;
		// Direct state is stale after bus-only rendering, snap on return.
		voice->dir_delay[0] = -1;
	} else if (read > 0) {
		// The ear model works in head-relative space. Ears sit on +-X.
		vec3  uh   = quat_inverse(head.orientation) * u;
		float cosL = -uh.x;
		float cosR =  uh.x;

		// Per-ear delays slew across the block, so a moving source sweeps
		// its ITD continuously instead of stepping. Only the interaural
		// *difference* is audible - the near ear is normalized to zero so
		// onsets stay sample exact.
		float tgt_l = au_ear_delay(cosL);
		float tgt_r = au_ear_delay(cosR);
		float base  = fminf(tgt_l, tgt_r);
		tgt_l -= base;
		tgt_r -= base;
		if (voice->dir_delay[0] < 0) { voice->dir_delay[0] = tgt_l; voice->dir_delay[1] = tgt_r; }
		float delay_l = voice->dir_delay[0], step_l = (tgt_l - delay_l) / (float)read;
		float delay_r = voice->dir_delay[1], step_r = (tgt_r - delay_r) / (float)read;

		// Head shadow on the far side, scaled by how far around it sits.
		float wet_l = 0.6f * fmaxf(0, -cosL);
		float wet_r = 0.6f * fmaxf(0, -cosR);

		// Elevation/back voicing, shared mono before the ear split. The
		// pinna dip rises and deepens with elevation, below and behind
		// lose sparkle - same voicing the 8-speaker decode had at its
		// poles, now continuous per source. Rebuilt each block.
		float       el01  = uh.y * 0.5f + 0.5f;
		float       dull  = -2.0f * (1.0f - el01) - 3.0f * fmaxf(0, uh.z);
		au_biquad_t pinna = au_biquad_peaking(math_lerp(6500, 8000, el01), 4, math_lerp(-4, -8, el01));
		au_biquad_t shelf = au_biquad_highshelf(5000, dull);

		float    g_direct = gain * (1.0f - k_bus);
		float    shadow_l = voice->dir_shadow[0];
		float    shadow_r = voice->dir_shadow[1];
		float*   ring     = voice->dir_ring;
		int32_t  ring_at  = voice->dir_ring_at;
		float*   out      = output + frame_offset * 2;
		for (ma_uint64 i = 0; i < read; i++) {
			float s = au_mix_temp[i];
			if (peak < s) peak = s;
			lp0 += alpha * (s   - lp0);
			lp1 += alpha * (lp0 - lp1);
			lp2 += alpha * (lp1 - lp2);
			lp3 += alpha * (lp2 - lp3);
			s = lp3;

			if (k_bus > 0) {
				float* bus = au_foa + (frame_offset+i)*4;
				XMStoreFloat4((XMFLOAT4*)bus, XMVectorMultiplyAdd(XMVectorReplicate(s), enc, XMLoadFloat4((XMFLOAT4*)bus)));
			}

			s = au_biquad_apply(&pinna, voice->dir_filter[0], s);
			s = au_biquad_apply(&shelf, voice->dir_filter[1], s);
			ring[ring_at] = s;

			// Fractional ear taps, then the far-side shadow one-pole.
			float rp_l = (float)ring_at - delay_l; if (rp_l < 0) rp_l += AU_ITD_MAX;
			float rp_r = (float)ring_at - delay_r; if (rp_r < 0) rp_r += AU_ITD_MAX;
			int32_t l0 = (int32_t)rp_l, l1 = l0 + 1 >= AU_ITD_MAX ? 0 : l0 + 1;
			int32_t r0 = (int32_t)rp_r, r1 = r0 + 1 >= AU_ITD_MAX ? 0 : r0 + 1;
			float   vl = math_lerp(ring[l0], ring[l1], rp_l - (float)l0);
			float   vr = math_lerp(ring[r0], ring[r1], rp_r - (float)r0);
			shadow_l += 0.178f * (vl - shadow_l);
			shadow_r += 0.178f * (vr - shadow_r);
			vl += wet_l * (shadow_l - vl);
			vr += wet_r * (shadow_r - vr);

			out[i*2  ] += vl * g_direct;
			out[i*2+1] += vr * g_direct;

			delay_l += step_l;
			delay_r += step_r;
			ring_at  = ring_at + 1 >= AU_ITD_MAX ? 0 : ring_at + 1;
		}
		voice->lpf_state[0]  = lp0; voice->lpf_state[1] = lp1;
		voice->lpf_state[2]  = lp2; voice->lpf_state[3] = lp3;
		voice->dir_delay[0]  = tgt_l;
		voice->dir_delay[1]  = tgt_r;
		voice->dir_shadow[0] = shadow_l;
		voice->dir_shadow[1] = shadow_r;
		voice->dir_ring_at   = ring_at;
		if (k_bus > 0) au_foa_touched = true;
	}

	// Track the block's peak for the intensity getter. Main zeroes this
	// each frame, a lost reset here just makes the meter sticky one frame.
	if (peak > atomic_load_f32(&voice->intensity))
		atomic_store_f32(&voice->intensity, peak);

	return (ma_uint32)read;
}

///////////////////////////////////////////

// Pure point sources - mono, spatialized, zero spread, outside the head -
// all take the direct binaural path, so they can render 4-wide.
static bool voice_is_direct(const au_voice_t* voice, vec3 listener_pos) {
	if (atomic_load_i32(&au_force_bus))                        return false;
	if (voice->sound->channels != sound_channels_mono)         return false;
	if (atomic_load_i32(&voice->params.flags) & sound_flags_head_locked) return false;
	if (atomic_load_f32(&voice->params.spread) > 0)            return false;
	vec3 d = vec3{
		atomic_load_f32(&voice->params.pos_x),
		atomic_load_f32(&voice->params.pos_y),
		atomic_load_f32(&voice->params.pos_z) } - listener_pos;
	return vec3_magnitude_sq(d) > 0.0001f;
}

// Audio thread. Four pure-direct voices rendered as SIMD lanes across one
// block: the air cascade, voicing biquads, and ear shadows run 4 voices
// wide with per-voice coefficients in the lanes, and the batch sums into
// the output once instead of four read-modify-write passes. Only the
// fractional ear taps stay scalar - per-voice ring indexing. Lanes hold
// lockstep by zero-padding outside each voice's live span; filters of
// zeros are exact silence, so onsets and finishes match the scalar path.
static void voice_mix_direct4(au_voice_t** vs, pose_t head, const ma_uint32* offsets, const ma_uint32* blocks, ma_uint32* out_read, float* output, ma_uint32 frame_count) {
	quat qinv = quat_inverse(head.orientation);

	float       gains[4], alphas[4], wets_l[4], wets_r[4];
	float       dlys_l[4], dlys_r[4], stps_l[4], stps_r[4], tgts_l[4], tgts_r[4];
	au_biquad_t pinnas[4], shelfs[4];
	float*      srcs[4];
	int32_t     rats[4];

	for (int32_t v = 0; v < 4; v++) {
		au_voice_t* voice  = vs[v];
		float       volume = atomic_load_f32(&voice->params.volume);
		float       pitch  = atomic_load_f32(&voice->params.pitch);
		pitch = pitch <= 0 ? 1 : fminf(AU_PITCH_MAX, fmaxf(AU_PITCH_MIN, pitch));
		bool  loop = (atomic_load_i32(&voice->params.flags) & sound_flags_loop) != 0;
		float rate = pitch * ((float)voice->sound->sample_rate / (float)AU_SAMPLE_RATE);

		float decibels  = atomic_load_f32(&voice->sound->decibels);
		float trim_gain = voice->sound->norm_gain * volume
		                * atomic_load_f32(&au_bus_volumes[voice->bus])
		                * atomic_load_f32(&au_master_volume);

		vec3 position = {
			atomic_load_f32(&voice->params.pos_x),
			atomic_load_f32(&voice->params.pos_y),
			atomic_load_f32(&voice->params.pos_z) };
		vec3  dir   = position - head.position;
		float dist2 = vec3_magnitude_sq(dir);
		float dist  = fmaxf(AU_MIN_DISTANCE, sqrtf(dist2));
		// The classification can go stale for one block if main moves the
		// source into the head mid-frame - render it frontal, not NaN.
		vec3  u     = dist2 > 0.0001f ? dir / sqrtf(dist2) : vec3{0, 0, -1};
		gains[v]    = decibels_to_signal(decibels - 20.0f*log10f(dist)) * trim_gain;

		float override = atomic_load_f32(&voice->params.cutoff);
		float cutoff   = override > 0 ? override
			: fmaxf(1000, -4000.f * logf(fmaxf(1, dist-3.5f)) + 22200);
		float RC  = 0.43496f / (2.0f * 3.14159265359f * cutoff);
		float dt  = 1.0f / AU_SAMPLE_RATE;
		alphas[v] = dt / (RC + dt);

		vec3  uh   = qinv * u;
		float tgt_l = au_ear_delay(-uh.x);
		float tgt_r = au_ear_delay( uh.x);
		float base  = fminf(tgt_l, tgt_r);
		tgt_l -= base;
		tgt_r -= base;
		if (voice->dir_delay[0] < 0) { voice->dir_delay[0] = tgt_l; voice->dir_delay[1] = tgt_r; }
		tgts_l[v] = tgt_l;                  tgts_r[v] = tgt_r;
		dlys_l[v] = voice->dir_delay[0];    dlys_r[v] = voice->dir_delay[1];
		stps_l[v] = (tgt_l - dlys_l[v]) / (float)frame_count;
		stps_r[v] = (tgt_r - dlys_r[v]) / (float)frame_count;
		wets_l[v] = 0.6f * fmaxf(0,  uh.x);
		wets_r[v] = 0.6f * fmaxf(0, -uh.x);

		float el01 = uh.y * 0.5f + 0.5f;
		float dull = -2.0f * (1.0f - el01) - 3.0f * fmaxf(0, uh.z);
		pinnas[v]  = au_biquad_peaking(math_lerp(6500, 8000, el01), 4, math_lerp(-4, -8, el01));
		shelfs[v]  = au_biquad_highshelf(5000, dull);
		rats[v]    = voice->dir_ring_at;

		// Read into this lane's slice at its onset offset, zeros elsewhere.
		float* src = au_mix_temp + (uint64_t)v * au_mix_temp_size;
		memset(src, 0, sizeof(float) * frame_count);
		out_read[v] = (ma_uint32)voice_read(voice, src + offsets[v], blocks[v], rate, loop);
		srcs[v] = src;
	}

	// Voice state gathered into lanes, [lane] = voice.
	XMVECTOR lp0 = XMVectorSet(vs[0]->lpf_state[0], vs[1]->lpf_state[0], vs[2]->lpf_state[0], vs[3]->lpf_state[0]);
	XMVECTOR lp1 = XMVectorSet(vs[0]->lpf_state[1], vs[1]->lpf_state[1], vs[2]->lpf_state[1], vs[3]->lpf_state[1]);
	XMVECTOR lp2 = XMVectorSet(vs[0]->lpf_state[2], vs[1]->lpf_state[2], vs[2]->lpf_state[2], vs[3]->lpf_state[2]);
	XMVECTOR lp3 = XMVectorSet(vs[0]->lpf_state[3], vs[1]->lpf_state[3], vs[2]->lpf_state[3], vs[3]->lpf_state[3]);
	XMVECTOR px1 = XMVectorSet(vs[0]->dir_filter[0][0], vs[1]->dir_filter[0][0], vs[2]->dir_filter[0][0], vs[3]->dir_filter[0][0]);
	XMVECTOR px2 = XMVectorSet(vs[0]->dir_filter[0][1], vs[1]->dir_filter[0][1], vs[2]->dir_filter[0][1], vs[3]->dir_filter[0][1]);
	XMVECTOR py1 = XMVectorSet(vs[0]->dir_filter[0][2], vs[1]->dir_filter[0][2], vs[2]->dir_filter[0][2], vs[3]->dir_filter[0][2]);
	XMVECTOR py2 = XMVectorSet(vs[0]->dir_filter[0][3], vs[1]->dir_filter[0][3], vs[2]->dir_filter[0][3], vs[3]->dir_filter[0][3]);
	XMVECTOR sx1 = XMVectorSet(vs[0]->dir_filter[1][0], vs[1]->dir_filter[1][0], vs[2]->dir_filter[1][0], vs[3]->dir_filter[1][0]);
	XMVECTOR sx2 = XMVectorSet(vs[0]->dir_filter[1][1], vs[1]->dir_filter[1][1], vs[2]->dir_filter[1][1], vs[3]->dir_filter[1][1]);
	XMVECTOR sy1 = XMVectorSet(vs[0]->dir_filter[1][2], vs[1]->dir_filter[1][2], vs[2]->dir_filter[1][2], vs[3]->dir_filter[1][2]);
	XMVECTOR sy2 = XMVectorSet(vs[0]->dir_filter[1][3], vs[1]->dir_filter[1][3], vs[2]->dir_filter[1][3], vs[3]->dir_filter[1][3]);
	XMVECTOR shl = XMVectorSet(vs[0]->dir_shadow[0], vs[1]->dir_shadow[0], vs[2]->dir_shadow[0], vs[3]->dir_shadow[0]);
	XMVECTOR shr = XMVectorSet(vs[0]->dir_shadow[1], vs[1]->dir_shadow[1], vs[2]->dir_shadow[1], vs[3]->dir_shadow[1]);

	XMVECTOR alpha4 = XMVectorSet(alphas[0], alphas[1], alphas[2], alphas[3]);
	XMVECTOR gain4  = XMVectorSet(gains [0], gains [1], gains [2], gains [3]);
	XMVECTOR wetl4  = XMVectorSet(wets_l[0], wets_l[1], wets_l[2], wets_l[3]);
	XMVECTOR wetr4  = XMVectorSet(wets_r[0], wets_r[1], wets_r[2], wets_r[3]);
	XMVECTOR pb0 = XMVectorSet(pinnas[0].b0, pinnas[1].b0, pinnas[2].b0, pinnas[3].b0);
	XMVECTOR pb1 = XMVectorSet(pinnas[0].b1, pinnas[1].b1, pinnas[2].b1, pinnas[3].b1);
	XMVECTOR pb2 = XMVectorSet(pinnas[0].b2, pinnas[1].b2, pinnas[2].b2, pinnas[3].b2);
	XMVECTOR pa1 = XMVectorSet(pinnas[0].a1, pinnas[1].a1, pinnas[2].a1, pinnas[3].a1);
	XMVECTOR pa2 = XMVectorSet(pinnas[0].a2, pinnas[1].a2, pinnas[2].a2, pinnas[3].a2);
	XMVECTOR sb0 = XMVectorSet(shelfs[0].b0, shelfs[1].b0, shelfs[2].b0, shelfs[3].b0);
	XMVECTOR sb1 = XMVectorSet(shelfs[0].b1, shelfs[1].b1, shelfs[2].b1, shelfs[3].b1);
	XMVECTOR sb2 = XMVectorSet(shelfs[0].b2, shelfs[1].b2, shelfs[2].b2, shelfs[3].b2);
	XMVECTOR sa1 = XMVectorSet(shelfs[0].a1, shelfs[1].a1, shelfs[2].a1, shelfs[3].a1);
	XMVECTOR sa2 = XMVectorSet(shelfs[0].a2, shelfs[1].a2, shelfs[2].a2, shelfs[3].a2);
	XMVECTOR shk   = XMVectorReplicate(0.178f);
	XMVECTOR peak4 = XMVectorZero();
	XMVECTOR ones  = XMVectorSplatOne();

	for (ma_uint32 i = 0; i < frame_count; i++) {
		XMVECTOR s = XMVectorSet(srcs[0][i], srcs[1][i], srcs[2][i], srcs[3][i]);
		peak4 = XMVectorMax(peak4, s);

		// Air absorption cascade, 4 voices per op.
		lp0 = XMVectorMultiplyAdd(alpha4, XMVectorSubtract(s,   lp0), lp0);
		lp1 = XMVectorMultiplyAdd(alpha4, XMVectorSubtract(lp0, lp1), lp1);
		lp2 = XMVectorMultiplyAdd(alpha4, XMVectorSubtract(lp1, lp2), lp2);
		lp3 = XMVectorMultiplyAdd(alpha4, XMVectorSubtract(lp2, lp3), lp3);
		s   = lp3;

		// Pinna dip, then the dull shelf.
		XMVECTOR y = XMVectorMultiply(pb0, s);
		y = XMVectorMultiplyAdd             (pb1, px1, y);
		y = XMVectorMultiplyAdd             (pb2, px2, y);
		y = XMVectorNegativeMultiplySubtract(pa1, py1, y);
		y = XMVectorNegativeMultiplySubtract(pa2, py2, y);
		px2 = px1; px1 = s; py2 = py1; py1 = y; s = y;

		y = XMVectorMultiply(sb0, s);
		y = XMVectorMultiplyAdd             (sb1, sx1, y);
		y = XMVectorMultiplyAdd             (sb2, sx2, y);
		y = XMVectorNegativeMultiplySubtract(sa1, sy1, y);
		y = XMVectorNegativeMultiplySubtract(sa2, sy2, y);
		sx2 = sx1; sx1 = s; sy2 = sy1; sy1 = y; s = y;

		// Per-voice fractional ear taps, the one scalar stage.
		XMFLOAT4A sf, vlf, vrf;
		XMStoreFloat4A(&sf, s);
		float* sfp = (float*)&sf;
		float* vlp = (float*)&vlf;
		float* vrp = (float*)&vrf;
		for (int32_t v = 0; v < 4; v++) {
			float* ring = vs[v]->dir_ring;
			int32_t rat = rats[v];
			ring[rat] = sfp[v];
			float rp_l = (float)rat - dlys_l[v]; if (rp_l < 0) rp_l += AU_ITD_MAX;
			float rp_r = (float)rat - dlys_r[v]; if (rp_r < 0) rp_r += AU_ITD_MAX;
			int32_t l0 = (int32_t)rp_l, l1 = l0 + 1 >= AU_ITD_MAX ? 0 : l0 + 1;
			int32_t r0 = (int32_t)rp_r, r1 = r0 + 1 >= AU_ITD_MAX ? 0 : r0 + 1;
			vlp[v] = math_lerp(ring[l0], ring[l1], rp_l - (float)l0);
			vrp[v] = math_lerp(ring[r0], ring[r1], rp_r - (float)r0);
			dlys_l[v] += stps_l[v];
			dlys_r[v] += stps_r[v];
			rats[v] = rat + 1 >= AU_ITD_MAX ? 0 : rat + 1;
		}

		// Far-side shadow one-pole and gain, then one sum into the output.
		XMVECTOR vl = XMLoadFloat4A(&vlf);
		XMVECTOR vr = XMLoadFloat4A(&vrf);
		shl = XMVectorMultiplyAdd(shk, XMVectorSubtract(vl, shl), shl);
		shr = XMVectorMultiplyAdd(shk, XMVectorSubtract(vr, shr), shr);
		vl  = XMVectorMultiplyAdd(wetl4, XMVectorSubtract(shl, vl), vl);
		vr  = XMVectorMultiplyAdd(wetr4, XMVectorSubtract(shr, vr), vr);
		output[i*2  ] += XMVectorGetX(XMVector4Dot(XMVectorMultiply(vl, gain4), ones));
		output[i*2+1] += XMVectorGetX(XMVector4Dot(XMVectorMultiply(vr, gain4), ones));
	}

	// Scatter the lane state back to the voices.
	XMFLOAT4A st;
	float*    stp = (float*)&st;
	XMStoreFloat4A(&st, lp0); for (int32_t v = 0; v < 4; v++) vs[v]->lpf_state[0]     = stp[v];
	XMStoreFloat4A(&st, lp1); for (int32_t v = 0; v < 4; v++) vs[v]->lpf_state[1]     = stp[v];
	XMStoreFloat4A(&st, lp2); for (int32_t v = 0; v < 4; v++) vs[v]->lpf_state[2]     = stp[v];
	XMStoreFloat4A(&st, lp3); for (int32_t v = 0; v < 4; v++) vs[v]->lpf_state[3]     = stp[v];
	XMStoreFloat4A(&st, px1); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[0][0] = stp[v];
	XMStoreFloat4A(&st, px2); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[0][1] = stp[v];
	XMStoreFloat4A(&st, py1); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[0][2] = stp[v];
	XMStoreFloat4A(&st, py2); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[0][3] = stp[v];
	XMStoreFloat4A(&st, sx1); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[1][0] = stp[v];
	XMStoreFloat4A(&st, sx2); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[1][1] = stp[v];
	XMStoreFloat4A(&st, sy1); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[1][2] = stp[v];
	XMStoreFloat4A(&st, sy2); for (int32_t v = 0; v < 4; v++) vs[v]->dir_filter[1][3] = stp[v];
	XMStoreFloat4A(&st, shl); for (int32_t v = 0; v < 4; v++) vs[v]->dir_shadow[0]    = stp[v];
	XMStoreFloat4A(&st, shr); for (int32_t v = 0; v < 4; v++) vs[v]->dir_shadow[1]    = stp[v];
	XMStoreFloat4A(&st, peak4);
	for (int32_t v = 0; v < 4; v++) {
		vs[v]->dir_delay[0] = tgts_l[v];
		vs[v]->dir_delay[1] = tgts_r[v];
		vs[v]->dir_ring_at  = rats[v];
		if (stp[v] > atomic_load_f32(&vs[v]->intensity))
			atomic_store_f32(&vs[v]->intensity, stp[v]);
	}
}

static void audio_mix_block(float* output, ma_uint32 frame_count) {
	audio_drain_commands();

	// One listener snapshot for the whole block, so classification, mixing,
	// and the FOA decode all agree even if main republishes mid-block.
	pose_t head = audio_listener_get();

	if (frame_count + AU_SAMPLE_BUFFER_SIZE*2 > au_mix_temp_size) {
		if (!au_mix_truncate_warned) {
			au_mix_truncate_warned = true;
			log_errf("Audio block of %u frames exceeds the mix buffer, truncating!", frame_count);
		}
		frame_count = (ma_uint32)(au_mix_temp_size - AU_SAMPLE_BUFFER_SIZE*2);
	}

	memset(au_foa, 0, sizeof(float) * frame_count * 4);
	au_foa_touched = false;

	// Pure point sources gather here during the sweep and render 4-wide
	// after it; everything else mixes inline.
	au_voice_t* direct       [AU_VOICE_COUNT];
	ma_uint32   direct_offset[AU_VOICE_COUNT];
	ma_uint32   direct_block [AU_VOICE_COUNT];
	int16_t     direct_slot  [AU_VOICE_COUNT];
	int32_t     direct_count = 0;

	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		au_voice_t* voice = &au_voices[i];
		if (atomic_load_i32_acq(&voice->state) != au_voice_playing)
			continue;

		if (atomic_load_i32(&voice->params.stop_request) != 0) {
			voice_finish(voice, i);
			continue;
		}

		// Seeks only make sense for in-memory sounds, streams read forward.
		uint64_t seek = atomic_exchange_u64(&voice->params.seek_request, AU_SEEK_NONE);
		if (seek != AU_SEEK_NONE && voice->sound->data_type == sound_data_pcm) {
			atomic_store_u64(&voice->cursor, mini(seek, voice->sound->pcm_count));
			voice->resample_frac = 0;
			memset(voice->lpf_state, 0, sizeof(voice->lpf_state));
		}

		if (atomic_load_i32(&voice->params.paused) != 0)
			continue;

		// Dormant voices ranked out of the mix budget freeze in place:
		// no read, no cursor movement, ready to resume.
		if (atomic_load_i32(&voice->audible) == 0)
			continue;

		// Onset delay is sample accurate: a partial delay starts the voice
		// mid-block at a frame offset.
		ma_uint32 offset = 0;
		ma_uint32 block  = frame_count;
		if (voice->delay_left >= block) {
			voice->delay_left -= block;
			continue;
		}
		if (voice->delay_left > 0) {
			offset = (ma_uint32)voice->delay_left;
			block -= offset;
			voice->delay_left = 0;
		}

		if (voice_is_direct(voice, head.position)) {
			direct       [direct_count] = voice;
			direct_offset[direct_count] = offset;
			direct_block [direct_count] = block;
			direct_slot  [direct_count] = i;
			direct_count += 1;
			continue;
		}

		ma_uint32 mixed = voice_mix(voice, head, output, offset, block);
		if (mixed < block && voice->sound->data_type != sound_data_ring)
			voice_finish(voice, i);
	}

	// Full groups of 4 render batched, the remainder takes the scalar path.
	int32_t b = 0;
	for (; b + 4 <= direct_count; b += 4) {
		ma_uint32 reads[4];
		voice_mix_direct4(&direct[b], head, &direct_offset[b], &direct_block[b], reads, output, frame_count);
		for (int32_t v = 0; v < 4; v++) {
			if (reads[v] < direct_block[b+v] && direct[b+v]->sound->data_type != sound_data_ring)
				voice_finish(direct[b+v], direct_slot[b+v]);
		}
	}
	for (; b < direct_count; b++) {
		ma_uint32 mixed = voice_mix(direct[b], head, output, direct_offset[b], direct_block[b]);
		if (mixed < direct_block[b] && direct[b]->sound->data_type != sound_data_ring)
			voice_finish(direct[b], direct_slot[b]);
	}

	// An untouched bus still decodes briefly - the ear filters and delays
	// need to ring down - then the whole decode turns off until the next
	// spatial voice arrives.
	if (au_foa_touched) au_foa_tail = AU_SAMPLE_RATE / 10;
	if (au_foa_tail > 0) {
		au_foa_tail -= mini((int32_t)frame_count, au_foa_tail);
		audio_decode_foa(output, frame_count, head);
	}

	// The limiter replaces per-voice clamping: transparent until -1dBFS,
	// then a smooth squash, so a hot mix distorts gently instead of
	// hard clipping. The meter reads the post-limit result.
	double sum = 0;
	for (ma_uint32 i = 0; i < frame_count*2; i++) {
		output[i] = au_limit(output[i]);
		sum += (double)output[i] * output[i];
	}
	float rms = frame_count > 0 ? sqrtf((float)(sum / (frame_count*2))) : 0;
	atomic_store_f32(&au_output_dbfs, rms <= 0.000001f ? -120.0f : 20.0f*log10f(rms));
}

///////////////////////////////////////////

void data_callback(ma_device*, void* output, const void*, ma_uint32 frame_count) {
	audio_mix_block((float*)output, frame_count);
}

void audio_render_block(float* out_stereo, int32_t frame_count) {
	memset(out_stereo, 0, (size_t)frame_count * 2 * sizeof(float));
	audio_mix_block(out_stereo, (ma_uint32)frame_count);
}

void audio_test_offline(bool32_t enable) {
	au_offline = enable;
}

// The main-thread half of a frame for offline tests. The listener stays
// at identity unless the test steers it through audio_set_listener, and
// shapes step with a fixed dt for determinism.
void audio_test_step() {
	pose_t pose = au_listener_has_override ? au_listener_override : pose_identity;
	audio_listener_publish(pose);
	audio_mix_drain_returns();
	audio_voice_prefetch();
	audio_voice_shapes_step(pose.position, 0.016f);
	audio_voice_rank();

	for (int32_t i = 0; i < AU_VOICE_COUNT; i++) {
		au_voices[i].intensity_frame = atomic_load_f32(&au_voices[i].intensity);
		atomic_store_f32(&au_voices[i].intensity, 0);
	}
}

///////////////////////////////////////////
// Shaped emitters. All main-thread: the shape lives in main-owned voice
// fields, gets evaluated once per frame, and the audio thread only sees
// the resolved position + spread params.

// Closest point on the shape's core to `from`. One point is a sphere
// center, more make a polyline.
static vec3 shape_closest(const vec3* points, int32_t count, vec3 from) {
	if (count == 1) return points[0];

	vec3  best      = points[0];
	float best_dist = 1e30f;
	for (int32_t i = 0; i < count - 1; i++) {
		vec3  seg   = points[i+1] - points[i];
		float len2  = vec3_magnitude_sq(seg);
		float t     = len2 > 0 ? vec3_dot(from - points[i], seg) / len2 : 0;
		t = fmaxf(0, fminf(1, t));
		vec3  at    = points[i] + seg * t;
		float dist2 = vec3_magnitude_sq(from - at);
		if (dist2 < best_dist) { best_dist = dist2; best = at; }
	}
	return best;
}

// Emit position is the closest point on the shape, apparent size is how
// much of the view the shape fills - up to fully diffuse when inside it.
// Both are smoothed so polyline corners and equidistant flips glide
// instead of popping.
static void voice_shape_eval(au_voice_t* voice, vec3 listener_pos, float dt) {
	vec3  closest = shape_closest(voice->shape_points, voice->shape_count, listener_pos);
	float dist    = vec3_magnitude(listener_pos - closest);

	// Subtended half angle of the tube, 90deg (asin(1)) once inside.
	float alpha  = asinf(fminf(1, voice->shape_radius / fmaxf(dist, 0.0001f)));
	float spread = alpha / (3.14159265f * 0.5f);

	// A polyline that stretches past both ends of view is wide no matter
	// how far its closest point is.
	if (voice->shape_count >= 2) {
		vec3 to_a = voice->shape_points[0]                      - listener_pos;
		vec3 to_b = voice->shape_points[voice->shape_count - 1] - listener_pos;
		float la = vec3_magnitude(to_a), lb = vec3_magnitude(to_b);
		if (la > 0.0001f && lb > 0.0001f) {
			float ends = acosf(fmaxf(-1, fminf(1, vec3_dot(to_a, to_b) / (la * lb)))) / 3.14159265f;
			spread = fmaxf(spread, ends);
		}
	}
	spread = fmaxf(spread, voice->base_spread);

	// Inside the tube the sound surrounds you: emit from the head, fully
	// diffuse. The asin hits 1 exactly at the boundary, so this continues
	// the outside curve rather than jumping.
	vec3 emit = closest;
	if (dist < voice->shape_radius) {
		emit   = listener_pos;
		spread = 1;
	}

	if (!voice->smooth_init) {
		voice->smooth_init   = true;
		voice->smooth_pos    = emit;
		voice->smooth_spread = spread;
	} else {
		float blend = 1.0f - expf(-dt / AU_SMOOTH_TIME);
		voice->smooth_pos    = vec3_lerp(voice->smooth_pos, emit, blend);
		voice->smooth_spread = math_lerp(voice->smooth_spread, spread, blend);
	}

	atomic_store_f32(&voice->params.pos_x,  voice->smooth_pos.x);
	atomic_store_f32(&voice->params.pos_y,  voice->smooth_pos.y);
	atomic_store_f32(&voice->params.pos_z,  voice->smooth_pos.z);
	atomic_store_f32(&voice->params.spread, voice->smooth_spread);
}

void audio_voice_shape_set(au_voice_t* voice, const vec3* points, int32_t count, float radius, vec3 listener_pos) {
	if (count > AU_SHAPE_MAX_POINTS) {
		log_errf("Audio emitter shapes are limited to %d points, truncating %d!", AU_SHAPE_MAX_POINTS, count);
		count = AU_SHAPE_MAX_POINTS;
	}
	memcpy(voice->shape_points, points, sizeof(vec3) * count);
	voice->shape_count  = count;
	voice->shape_radius = radius;
	voice->smooth_init  = false;
	voice_shape_eval(voice, listener_pos, 0);
}

// Main thread, once per frame.
void audio_voice_shapes_step(vec3 listener_pos, float dt) {
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		au_voice_t* voice = &au_voices[i];
		if (voice->shape_count == 0) continue;
		int32_t state = atomic_load_i32(&voice->state);
		if (state != au_voice_reserved && state != au_voice_playing) continue;
		voice_shape_eval(voice, listener_pos, dt);
	}
}

///////////////////////////////////////////

// Main thread, from audio_step, strictly after the return drain. Tops up
// streaming voices' prefetch rings. The decoder is main-owned, the audio
// thread only ever consumes the ring. A voice finishing concurrently is
// safe: its resources aren't freed until the *next* frame's drain.
void audio_voice_prefetch() {
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		au_voice_t* voice = &au_voices[i];
		if (atomic_load_i32_acq(&voice->state) != au_voice_playing ||
		    voice->stream_decoder == nullptr ||
		    atomic_load_i32(&voice->stream_eof) != 0)
			continue;

		while (ma_pcm_rb_available_write(voice->stream_ring) > AU_STREAM_PREFETCH / 4) {
			ma_uint32 request = ma_pcm_rb_available_write(voice->stream_ring);
			void*     into    = nullptr;
			if (ma_pcm_rb_acquire_write(voice->stream_ring, &request, &into) != MA_SUCCESS || request == 0)
				break;
			ma_uint64 decoded = 0;
			ma_result result  = ma_decoder_read_pcm_frames(voice->stream_decoder, into, request, &decoded);
			ma_pcm_rb_commit_write(voice->stream_ring, (ma_uint32)decoded);
			if (result != MA_SUCCESS || decoded < request) {
				// Looping streams rewind the decoder and keep filling, the
				// ring never sees an eof.
				if ((atomic_load_i32(&voice->params.flags) & sound_flags_loop) != 0 &&
				    ma_decoder_seek_to_pcm_frame(voice->stream_decoder, 0) == MA_SUCCESS)
					continue;
				atomic_store_i32_rel(&voice->stream_eof, 1);
				break;
			}
		}
	}
}

///////////////////////////////////////////

// Main thread, with the audio device already stopped. Everything left in
// the pool and rings gets released inline. Queued plays activate first so
// each voice's resources are freed exactly once, through one path.
void audio_mix_shutdown() {
	audio_drain_commands(); // No audio thread anymore, safe to run here
	audio_mix_drain_returns();
	for (int16_t i = 0; i < AU_VOICE_COUNT; i++) {
		au_voice_t* voice = &au_voices[i];
		// Covers playing voices, and freed slots still holding a displaced
		// play's resources after a steal's submit was refused.
		if (voice->sound != nullptr)
			voice_free_resources(voice->sound, voice->stream_decoder, voice->stream_ring, voice->stream_ring_data);
		memset(voice, 0, sizeof(au_voice_t));
	}
	au_cmd_head = au_cmd_tail = 0;
	au_ret_head = au_ret_tail = 0;
	sk_free(au_mix_temp);
	au_mix_temp      = nullptr;
	au_mix_temp_size = 0;
	sk_free(au_resample_temp);
	au_resample_temp      = nullptr;
	au_resample_temp_size = 0;
	sk_free(au_foa);
	au_foa = nullptr;
	memset(&au_decode, 0, sizeof(au_decode));
	au_mix_truncate_warned = false;
	atomic_store_f32(&au_output_dbfs, -120);
}

} // namespace sk
