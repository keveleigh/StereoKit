#include "sound.h"
#include "assets.h"
#include "../stereokit.h"
#include "../sk_memory.h"
#include "../sk_math.h"
#include "../platforms/platform.h"
#include "../systems/audio.h"
#include "../libraries/ferr_thread.h"
#include "../libraries/atomic_util.h"

#include <string.h>
#include <math.h>

namespace sk {

///////////////////////////////////////////

sound_t sound_find(const char *id) {
	sound_t result = (sound_t)assets_find(id, asset_type_sound);
	if (result != nullptr) {
		sound_addref(result);
		return result;
	}
	return nullptr;
}

///////////////////////////////////////////

void sound_set_id(sound_t sound, const char *id) {
	assets_set_id(&sound->header, id);
}

///////////////////////////////////////////

const char* sound_get_id(const sound_t sound) {
	return sound->header.id_text;
}

///////////////////////////////////////////

struct sound_load_t {
	void*    file_data;
	size_t   file_size;
	bool32_t require_ambisonic;
};

// Loudness normalization makes declared decibels truthful: the waveform
// is the *shape* of a sound, decibels are how loud it is, and measuring
// the content is what enforces that separation. Measurement is gated RMS
// in the style of EBU R128: 50ms blocks, and blocks more than 30dB below
// the loudest are ignored - silence and quiet tails don't dilute the
// measure, so padding a sound can't make it louder. The result scales
// content loudness to unity, decibels_to_signal provides the rest.
// Intra-sound dynamics are always preserved, this is one gain per asset.
// A generous 18dB crest allowance backstops pathological content (a
// near-silent file with one hot sample), while musical crest like drum
// or thunder transients rides declared loudness into the limiter instead.
// measure_ch selects one interleaved channel (ambisonics measure W, the
// omni loudness), or -1 for all of them.
static float sound_norm_gain(const float* samples, uint64_t frames, int32_t channels, int32_t measure_ch) {
	if (frames == 0) return 1;

	const uint64_t block_size = AU_SAMPLE_RATE / 20; // 50ms, in frames
	int32_t ch_start = measure_ch < 0 ? 0        : measure_ch;
	int32_t ch_end   = measure_ch < 0 ? channels : measure_ch + 1;
	int32_t ch_count = ch_end - ch_start;

	// Pass 1: the loudest block's mean square, and the content peak.
	double loudest = 0;
	float  peak    = 0;
	for (uint64_t at = 0; at < frames; at += block_size) {
		uint64_t end = mini(at + block_size, frames);
		double   ms  = 0;
		for (uint64_t i = at; i < end; i++) {
			for (int32_t c = ch_start; c < ch_end; c++) {
				float s = samples[i*channels + c];
				ms += (double)s * s;
				if (fabsf(s) > peak) peak = fabsf(s);
			}
		}
		ms /= (double)((end - at) * ch_count);
		if (ms > loudest) loudest = ms;
	}
	if (loudest <= 0) return 1;

	// Pass 2: average the blocks within 30dB of the loudest.
	double   gate = loudest * 0.001;
	double   sum  = 0;
	uint64_t n    = 0;
	for (uint64_t at = 0; at < frames; at += block_size) {
		uint64_t end = mini(at + block_size, frames);
		double   ms  = 0;
		for (uint64_t i = at; i < end; i++)
			for (int32_t c = ch_start; c < ch_end; c++)
				ms += (double)samples[i*channels + c] * samples[i*channels + c];
		ms /= (double)((end - at) * ch_count);
		if (ms >= gate) { sum += ms; n += 1; }
	}

	float norm = 1.0f / (float)sqrt(sum / (double)n);
	if (peak * norm > 8.0f) norm = 8.0f / peak;
	return fmaxf(0.3f, fminf(300.f, norm));
}

static bool32_t sound_load_decode(asset_task_t*, asset_header_t* asset, void* job_data) {
	sound_t       sound = (sound_t)asset;
	sound_load_t* data  = (sound_load_t*)job_data;

	// Probe with the file's own channel count, then constrain: ambisonic
	// loads require exactly 4, everything else keeps 1-2 and downmixes
	// surround to stereo.
	ma_decoder        decoder;
	ma_decoder_config config = ma_decoder_config_init(AU_SAMPLE_FORMAT, 0, AU_SAMPLE_RATE);
	if (ma_decoder_init_memory(data->file_data, data->file_size, &config, &decoder) != MA_SUCCESS) {
		log_errf("Failed to parse sound '%s'.", sound->header.id_text);
		atomic_store_i32_rel((int32_t*)&sound->header.state, asset_state_error_unsupported);
		return false;
	}
	ma_uint32 file_ch = decoder.outputChannels;
	if (data->require_ambisonic) {
		if (file_ch != 4) {
			log_errf("Ambisonic sound '%s' needs exactly 4 channels, found %u.", sound->header.id_text, file_ch);
			ma_decoder_uninit(&decoder);
			atomic_store_i32_rel((int32_t*)&sound->header.state, asset_state_error_unsupported);
			return false;
		}
		sound->channels = sound_channels_ambisonic1;
	} else if (file_ch >= 2) {
		if (file_ch > 2) {
			log_warnf("Sound '%s' has %u channels, downmixing to stereo. 4 channel ambiX content should use sound_create_ambisonic.", sound->header.id_text, file_ch);
			ma_decoder_uninit(&decoder);
			config = ma_decoder_config_init(AU_SAMPLE_FORMAT, 2, AU_SAMPLE_RATE);
			if (ma_decoder_init_memory(data->file_data, data->file_size, &config, &decoder) != MA_SUCCESS) {
				atomic_store_i32_rel((int32_t*)&sound->header.state, asset_state_error_unsupported);
				return false;
			}
		}
		sound->channels = sound_channels_stereo;
	} else {
		sound->channels = sound_channels_mono;
	}
	int32_t ch = sound_channel_count(sound->channels);

	ma_uint64 frames = 0;
	if (ma_decoder_get_length_in_pcm_frames(&decoder, &frames) != MA_SUCCESS)
		frames = 0;

	// data_type and state are the publication points for main thread
	// readers: their release stores pair with acquire loads in the getters,
	// making the plain field writes above them visible.
	if (frames > 0 && frames <= AU_PREDECODE_MAX) {
		float*    pcm  = sk_malloc_t(float, (size_t)(frames * ch));
		ma_uint64 read = 0;
		ma_decoder_read_pcm_frames(&decoder, pcm, frames, &read);
		ma_decoder_uninit(&decoder);

		sound->pcm       = pcm;
		sound->pcm_count = read;
		sound->norm_gain = sound_norm_gain(pcm, read, ch, sound->channels == sound_channels_ambisonic1 ? 0 : -1);
		atomic_store_i32_rel((int32_t*)&sound->data_type, sound_data_pcm);
	} else {
		// Long or unknown-length sounds keep their compressed bytes, and
		// each playing voice gets its own decoder over them.
		ma_decoder_uninit(&decoder);
		sound->file_data   = data->file_data;
		sound->file_size   = data->file_size;
		sound->file_frames = frames;
		data->file_data    = nullptr; // Ownership moved to the sound
		atomic_store_i32_rel((int32_t*)&sound->data_type, sound_data_stream_file);
	}
	atomic_store_i32_rel((int32_t*)&sound->header.state, asset_state_loaded);
	return true;
}

// The loading task publishes these fields from an asset thread, so readers
// that may touch a file sound mid-load go through this acquire.
static sound_data_ sound_data_type(sound_t sound) {
	return (sound_data_)atomic_load_i32_acq((int32_t*)&sound->data_type);
}

static void sound_load_free(asset_header_t*, void* job_data) {
	sound_load_t* data = (sound_load_t*)job_data;
	sk_free(data->file_data);
	sk_free(data);
}

static void sound_load_failure(asset_header_t* asset, void*) {
	if (asset->state >= asset_state_none)
		atomic_store_i32_rel((int32_t*)&asset->state, asset_state_error);
}

static asset_load_action_t sound_load_actions[] = {
	{ sound_load_decode, asset_thread_asset },
};

///////////////////////////////////////////

// Shared async-decode creation, takes ownership of the data buffer.
static sound_t sound_create_data(const char *id, void *data, size_t length, bool32_t require_ambisonic) {
	sound_t result = (_sound_t*)assets_allocate(asset_type_sound);
	result->decibels     = 80;
	result->norm_gain    = 1;
	result->sample_rate  = AU_SAMPLE_RATE;
	result->header.state = asset_state_loading;
	sound_set_id(result, id);

	sound_load_t* load = sk_malloc_zero_t(sound_load_t, 1);
	load->file_data         = data;
	load->file_size         = length;
	load->require_ambisonic = require_ambisonic;

	asset_task_t task = {};
	task.asset        = &result->header;
	task.load_data    = load;
	task.free_data    = sound_load_free;
	task.on_failure   = sound_load_failure;
	task.actions      = sound_load_actions;
	task.action_count = 1;
	task.sort         = asset_sort(0, asset_complexity_bytes(length));
	assets_add_task(task);
	return result;
}

///////////////////////////////////////////

sound_t sound_create(const char *filename) {
	sound_t result = sound_find(filename);
	if (result != nullptr)
		return result;

	// A synchronous read keeps the null-on-missing-file contract, the
	// expensive decode happens on the asset threads.
	void*  data;
	size_t length;
	if (!platform_read_file(filename, &data, &length)) {
		log_warnf("Sound file failed to load: %s", filename);
		return nullptr;
	}
	return sound_create_data(filename, data, length, false);
}

///////////////////////////////////////////

sound_t sound_create_ambisonic(const char *filename) {
	sound_t result = sound_find(filename);
	if (result != nullptr)
		return result;

	void*  data;
	size_t length;
	if (!platform_read_file(filename, &data, &length)) {
		log_warnf("Sound file failed to load: %s", filename);
		return nullptr;
	}
	return sound_create_data(filename, data, length, true);
}

///////////////////////////////////////////

sound_t sound_create_mem(const char *id, const void *data, size_t data_size) {
	sound_t result = sound_find(id);
	if (result != nullptr)
		return result;

	void* copy = sk_malloc(data_size);
	memcpy(copy, data, data_size);
	return sound_create_data(id, copy, data_size, false);
}

///////////////////////////////////////////

sound_t sound_create_stream(float buffer_duration, sound_channels_ channels, sound_sample_rate_ sample_rate) {
	// 0, or a nonsense negative from a bad enum cast, means the native mix
	// rate. Any other positive rate is honored - the mixer resamples as
	// needed - so we don't clamp to a range here.
	int32_t rate = sample_rate <= 0 ? AU_SAMPLE_RATE : (int32_t)sample_rate;
	int32_t ch   = sound_channel_count(channels);

	sound_t result = (_sound_t*)assets_allocate(asset_type_sound);
	result->data_type     = sound_data_ring;
	result->channels      = channels;
	result->decibels      = 80;
	result->norm_gain     = 1;
	result->sample_rate   = rate;
	result->ring_lock     = ft_mutex_create();
	result->ring_capacity = (uint64_t)((double)buffer_duration * rate); // Frames
	result->ring_data     = sk_malloc_t(float, (size_t)(result->ring_capacity * ch));
	memset(result->ring_data, 0, (size_t)(result->ring_capacity * ch * sizeof(float)));
	ma_pcm_rb_init(AU_SAMPLE_FORMAT, (ma_uint32)ch, (ma_uint32)result->ring_capacity, result->ring_data, nullptr, &result->ring);

	result->header.state = asset_state_loaded;
	return result;
}

///////////////////////////////////////////

sound_t sound_create_samples(const float *samples_at_48000s, uint64_t sample_count, sound_channels_ channels) {
	int32_t ch = sound_channel_count(channels);

	sound_t result = (_sound_t*)assets_allocate(asset_type_sound);
	result->data_type   = sound_data_pcm;
	result->channels    = channels;
	result->decibels    = 80;
	result->sample_rate = AU_SAMPLE_RATE;
	result->pcm_count   = sample_count / ch; // Interleaved samples -> frames
	result->pcm         = sk_malloc_t(float, (size_t)(result->pcm_count * ch));
	memcpy(result->pcm, samples_at_48000s, (size_t)(result->pcm_count * ch * sizeof(float)));
	result->norm_gain   = sound_norm_gain(result->pcm, result->pcm_count, ch, channels == sound_channels_ambisonic1 ? 0 : -1);

	result->header.state = asset_state_loaded;
	return result;
}

///////////////////////////////////////////

// The generator's counts are frames: fill frame_count * channel-count
// interleaved samples. For mono that's the same numbers as before.
sound_t sound_generate(void (*audio_generator)(float *out_samples, uint64_t frame_start, uint64_t frame_count), float duration, sound_channels_ channels) {
	int32_t ch = sound_channel_count(channels);

	sound_t result = (_sound_t*)assets_allocate(asset_type_sound);
	result->data_type   = sound_data_pcm;
	result->channels    = channels;
	result->decibels    = 80;
	result->sample_rate = AU_SAMPLE_RATE;
	result->pcm_count   = (uint64_t)((double)duration * AU_SAMPLE_RATE);
	result->pcm         = sk_malloc_t(float, (size_t)(result->pcm_count * ch));
	audio_generator(result->pcm, 0, result->pcm_count);
	result->norm_gain   = sound_norm_gain(result->pcm, result->pcm_count, ch, channels == sound_channels_ambisonic1 ? 0 : -1);

	result->header.state = asset_state_loaded;
	return result;
}

///////////////////////////////////////////

void sound_write_samples(sound_t sound, const float *samples, uint64_t sample_count) {
	if (sound->data_type != sound_data_ring) { log_err("Sound read/write is only supported for streaming type sounds!"); return; }

	// Counts are interleaved samples, the ring works in frames.
	int32_t  ch     = sound_channel_count(sound->channels);
	uint64_t frames = sample_count / ch;

	// Writes larger than the whole ring only keep the freshest frames that fit.
	if (frames > sound->ring_capacity) {
		samples += (frames - sound->ring_capacity) * ch;
		frames   = sound->ring_capacity;
	}

	ma_uint32 written  = 0;
	ft_mutex_lock(sound->ring_lock);

	// A full ring overwrites the oldest frames, new data always lands.
	ma_uint32 available = ma_pcm_rb_available_write(&sound->ring);
	if (available < frames)
		ma_pcm_rb_seek_read(&sound->ring, (ma_uint32)frames - available);

	ma_uint32 writable = 0;
	void*     write_to = nullptr;
	while (written < frames) {
		writable = (ma_uint32)frames - written;

		ma_result res = ma_pcm_rb_acquire_write(&sound->ring, &writable, &write_to);
		if (res != MA_SUCCESS) { break; }
		memcpy(write_to, samples + (uint64_t)written*ch, (size_t)(writable * ch * sizeof(float)));

		res = ma_pcm_rb_commit_write(&sound->ring, writable);
		if (res != MA_SUCCESS) { break; }

		written += writable;
	}
	ft_mutex_unlock(sound->ring_lock);

	sound->ring_written = mini(sound->ring_written + written, sound->ring_capacity);
}

///////////////////////////////////////////

uint64_t sound_read_samples(sound_t sound, float *out_samples, uint64_t sample_count) {
	if (sound->data_type != sound_data_ring) { log_err("Sound read/write is only supported for streaming type sounds!"); return 0; }

	int32_t   ch        = sound_channel_count(sound->channels);
	ma_uint32 available = ma_pcm_rb_available_read(&sound->ring);
	uint64_t  frames    = mini((uint32_t)(sample_count / ch), available);

	ma_uint32 read  = 0;
	ft_mutex_lock(sound->ring_lock);
	ma_uint32 readable  = 0;
	void*     read_from = nullptr;
	while (read < frames) {
		readable = (ma_uint32)frames - read;

		ma_result res = ma_pcm_rb_acquire_read(&sound->ring, &readable, &read_from);
		if (res != MA_SUCCESS) { break; }

		memcpy(out_samples + (uint64_t)read*ch, read_from, (size_t)(readable * ch * sizeof(float)));

		res = ma_pcm_rb_commit_read(&sound->ring, readable);
		if (res != MA_SUCCESS && res != MA_AT_END) { break; }

		read += readable;
	}
	ft_mutex_unlock(sound->ring_lock);

	return (uint64_t)read * ch;
}

///////////////////////////////////////////

uint64_t sound_unread_samples(sound_t sound) {
	return sound->data_type == sound_data_ring
		? (uint64_t)ma_pcm_rb_available_read(&sound->ring) * sound_channel_count(sound->channels)
		: 0;
}

///////////////////////////////////////////

float sound_get_decibels(sound_t sound) {
	return atomic_load_f32(&sound->decibels);
}

///////////////////////////////////////////

void sound_set_decibels(sound_t sound, float decibels) {
	// Atomic because the mixer reads this per block while playing.
	atomic_store_f32(&sound->decibels, decibels);
}

///////////////////////////////////////////

float decibels_to_signal(float decibel) {
	// North American standard uses reference levels of 83db as -20dbfs, which
	// basically means that an 83db sound will result in a dbfs of -20, which
	// in turn is a digital signal value of 0.1. A dbfs of 0 will result in a
	// digital signal value of 1, and a -inf will result in a digital signal of
	// 0. The -20 reference level then provides a 20db headroom above 83 before
	// clipping starts happening.
	const float reference_dbfs = -20;
	const float reference_db   =  83;
	float dbfs = reference_dbfs + (decibel - reference_db);

	// This converts dbfs to a digital signal +-1 representation
	// return powf(10, dbfs / 20.0f);

	// This is the same as the powf call, but faster
	const float LN10_DIV_20 = 0.115129254f;
	return expf(dbfs * LN10_DIV_20);
}

///////////////////////////////////////////

static sound_inst_t sound_play_settings(sound_t sound, vec3 at, float volume_trim, const sound_play_t* settings) {
	sound_inst_t result;
	result._id   = 0;
	result._slot = -1;

	if (atomic_load_i32_acq((int32_t*)&sound->header.state) < asset_state_loaded) {
		log_diagf("sound_play: '%s' isn't loaded yet, or failed to load.", sound->header.id_text ? sound->header.id_text : "(unnamed)");
		return result;
	}

	// Multi-channel sounds don't spatialize, but the API always takes a
	// position - a zero position is the idiomatic "none", only a real one
	// suggests the caller expected placement to work.
	if (sound->channels != sound_channels_mono && (at.x != 0 || at.y != 0 || at.z != 0))
		log_diagf("sound_play: '%s' is multi-channel, its position is ignored.", sound->header.id_text ? sound->header.id_text : "(unnamed)");

	// A full pool refusing the least audible sound is normal operation,
	// the caller sees it in the returned handle.
	int16_t slot = audio_voice_reserve(sound, at, volume_trim);
	if (slot < 0)
		return result;
	au_voice_t* voice = &au_voices[slot];

	// Shapes evaluate immediately so the play command carries complete
	// initial state, and propagation delay measures from the real emit
	// point rather than the nominal position.
	vec3 emit_at         = at;
	voice->shape_count   = 0;
	voice->base_spread   = settings->spread;
	voice->smooth_init   = false;
	if (settings->shape_points != nullptr && settings->shape_point_count > 0) {
		audio_voice_shape_set(voice, settings->shape_points, settings->shape_point_count, settings->shape_radius, audio_listener_get().position);
		emit_at = voice->smooth_pos;
	}

	uint64_t delay_frames = (uint64_t)(fmaxf(0, settings->delay) * AU_SAMPLE_RATE + 0.5f);
	if (settings->flags & sound_flags_propagation_delay) {
		float dist = vec3_magnitude(audio_listener_get().position - emit_at);
		delay_frames += (uint64_t)((dist / 343.0f) * AU_SAMPLE_RATE + 0.5f);
	}

	sound_addref(sound);
	voice->pending_sound     = sound;
	voice->pending_cursor    = 0;
	voice->pending_delay     = delay_frames;
	voice->pending_bus       = settings->bus;
	voice->pending_decoder   = nullptr;
	voice->pending_ring      = nullptr;
	voice->pending_ring_data = nullptr;
	atomic_store_i32(&voice->stream_eof, 0);

	// Streaming sounds get a per-voice decoder and prefetch ring, prefilled
	// here so onset doesn't begin with underrun silence.
	if (sound->data_type == sound_data_stream_file) {
		int32_t           ch      = sound_channel_count(sound->channels);
		ma_decoder*       decoder = sk_malloc_t(ma_decoder, 1);
		ma_decoder_config config  = ma_decoder_config_init(AU_SAMPLE_FORMAT, (ma_uint32)ch, AU_SAMPLE_RATE);
		if (ma_decoder_init_memory(sound->file_data, sound->file_size, &config, decoder) != MA_SUCCESS) {
			log_errf("Failed to make a play decoder for '%s'.", sound->header.id_text);
			sk_free(decoder);
			sound_release(sound);
			voice->pending_sound = nullptr;
			atomic_store_i32_rel(&voice->state, au_voice_free);
			return result;
		}
		ma_pcm_rb* rb  = sk_malloc_t(ma_pcm_rb, 1);
		float*     buf = sk_malloc_t(float, AU_STREAM_PREFETCH * ch);
		ma_pcm_rb_init(AU_SAMPLE_FORMAT, (ma_uint32)ch, AU_STREAM_PREFETCH, buf, nullptr, rb);

		ma_uint32 request = ma_pcm_rb_available_write(rb);
		void*     into    = nullptr;
		if (ma_pcm_rb_acquire_write(rb, &request, &into) == MA_SUCCESS && request > 0) {
			ma_uint64 decoded = 0;
			ma_result res     = ma_decoder_read_pcm_frames(decoder, into, request, &decoded);
			ma_pcm_rb_commit_write(rb, (ma_uint32)decoded);
			if (res != MA_SUCCESS || decoded < request)
				atomic_store_i32(&voice->stream_eof, 1);
		}

		voice->pending_decoder   = decoder;
		voice->pending_ring      = rb;
		voice->pending_ring_data = buf;
	}

	// Shaped voices already wrote their evaluated position and spread.
	if (voice->shape_count == 0) {
		atomic_store_f32(&voice->params.pos_x,  at.x);
		atomic_store_f32(&voice->params.pos_y,  at.y);
		atomic_store_f32(&voice->params.pos_z,  at.z);
		atomic_store_f32(&voice->params.spread, settings->spread);
	}
	atomic_store_f32(&voice->params.volume, volume_trim);
	atomic_store_f32(&voice->params.pitch,  settings->pitch);
	atomic_store_f32(&voice->params.cutoff, settings->cutoff);
	atomic_store_i32(&voice->params.flags,  settings->flags);
	atomic_store_i32(&voice->params.paused,       0);
	atomic_store_i32(&voice->params.stop_request, 0);
	atomic_store_u64(&voice->params.seek_request, AU_SEEK_NONE);

	if (!audio_voice_submit(slot)) {
		log_err("Audio command ring overflow, refusing to play sound!");
		if (voice->pending_decoder != nullptr) {
			ma_decoder_uninit(voice->pending_decoder);
			sk_free(voice->pending_decoder);
			ma_pcm_rb_uninit(voice->pending_ring);
			sk_free(voice->pending_ring);
			sk_free(voice->pending_ring_data);
		}
		voice->pending_sound     = nullptr;
		voice->pending_decoder   = nullptr;
		voice->pending_ring      = nullptr;
		voice->pending_ring_data = nullptr;
		sound_release(sound);
		// A stolen slot keeps its displaced resources in the audio-owned
		// fields, they're recovered on the slot's next activation/shutdown.
		atomic_store_i32_rel(&voice->state, au_voice_free);
		return result;
	}

	result._id   = voice->id;
	result._slot = slot;
	return result;
}

///////////////////////////////////////////

sound_inst_t sound_play(sound_t sound, vec3 at, const sound_play_t *opt_settings) {
	static const sound_play_t defaults = {};
	if (opt_settings == nullptr) opt_settings = &defaults;

	// A zeroed volume means "default", which is full trim.
	float trim = opt_settings->volume == 0 ? 1 : opt_settings->volume;
	return sound_play_settings(sound, at, trim, opt_settings);
}

///////////////////////////////////////////

static uint64_t sound_total_frames(sound_t sound) {
	switch (sound_data_type(sound)) {
	case sound_data_pcm:         return sound->pcm_count;
	case sound_data_stream_file: return sound->file_frames;
	case sound_data_ring:        return sound->ring_written;
	default:                     return 0;
	}
}

uint64_t sound_total_samples(sound_t sound) {
	return sound_total_frames(sound) * sound_channel_count(sound->channels);
}

///////////////////////////////////////////

uint64_t sound_cursor_samples(sound_t sound) {
	// With per-voice playback state, an asset level cursor only means
	// something for live streams: how far reads lag behind writes.
	return sound_data_type(sound) == sound_data_ring
		? (sound->ring_written - ma_pcm_rb_available_read(&sound->ring)) * sound_channel_count(sound->channels)
		: 0;
}

///////////////////////////////////////////

float sound_duration(sound_t sound) {
	return (float)sound_total_frames(sound) / (float)sound->sample_rate;
}

///////////////////////////////////////////

sound_channels_ sound_get_channels(sound_t sound) {
	return sound->channels;
}

///////////////////////////////////////////

void sound_addref(sound_t sound) {
	assets_addref(&sound->header);
}

///////////////////////////////////////////

void sound_release(sound_t sound) {
	if (sound == nullptr)
		return;

	assets_releaseref(&sound->header);
}

///////////////////////////////////////////

void sound_destroy(sound_t sound) {
	sk_free(sound->pcm);
	sk_free(sound->file_data);
	if (sound->data_type == sound_data_ring) {
		ma_pcm_rb_uninit(&sound->ring);
		sk_free          (sound->ring_data);
		ft_mutex_destroy(&sound->ring_lock);
	}
	memset(sound, 0, sizeof(_sound_t));
}

///////////////////////////////////////////

// Voice handles stay valid through reserved and playing, a generation
// mismatch or terminal state means the handle is dead.
static au_voice_t* sound_inst_voice(sound_inst_t sound_inst) {
	if (sound_inst._slot < 0 || sound_inst._slot >= AU_VOICE_COUNT) return nullptr;
	au_voice_t* voice = &au_voices[sound_inst._slot];
	if (voice->id != sound_inst._id) return nullptr;
	int32_t state = atomic_load_i32(&voice->state);
	if (state != au_voice_reserved && state != au_voice_playing) return nullptr;
	return voice;
}

///////////////////////////////////////////

// The mixer's cursor and seeks count frames, the public API counts
// interleaved samples. This resolves the voice's channel count for the
// conversion, reading whichever of the live or pending sound is set.
static int32_t sound_inst_channels(const au_voice_t* voice) {
	_sound_t* snd = (_sound_t*)atomic_load_ptr(&voice->sound);
	if (snd == nullptr) snd = (_sound_t*)atomic_load_ptr(&voice->pending_sound);
	return snd ? sound_channel_count(snd->channels) : 1;
}

///////////////////////////////////////////

void sound_inst_stop(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	// The mixer checks this at the top of every block, and a steal of this
	// slot supersedes it, so a stop can never be lost or overflow anything.
	atomic_store_i32(&voice->params.stop_request, 1);
}

///////////////////////////////////////////

bool32_t sound_inst_is_playing(sound_inst_t sound_inst) {
	return sound_inst_voice(sound_inst) != nullptr;
}

///////////////////////////////////////////

void sound_inst_set_pos(sound_inst_t sound_inst, vec3 pos) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	// An explicit position turns a shaped emitter back into a point.
	voice->shape_count = 0;
	atomic_store_f32(&voice->params.pos_x, pos.x);
	atomic_store_f32(&voice->params.pos_y, pos.y);
	atomic_store_f32(&voice->params.pos_z, pos.z);
}

///////////////////////////////////////////

vec3 sound_inst_get_pos(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return vec3_zero;

	return vec3 {
		atomic_load_f32(&voice->params.pos_x),
		atomic_load_f32(&voice->params.pos_y),
		atomic_load_f32(&voice->params.pos_z) };
}

///////////////////////////////////////////

void sound_inst_set_volume(sound_inst_t sound_inst, float volume_pct) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	atomic_store_f32(&voice->params.volume, volume_pct);
}

///////////////////////////////////////////

float sound_inst_get_volume(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return 0;

	return atomic_load_f32(&voice->params.volume);
}

///////////////////////////////////////////

float sound_inst_get_intensity(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return 0;

	return voice->intensity_frame;
}

///////////////////////////////////////////

void sound_inst_set_pitch(sound_inst_t sound_inst, float pitch_mult) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	atomic_store_f32(&voice->params.pitch, pitch_mult);
}

///////////////////////////////////////////

float sound_inst_get_pitch(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return 0;

	// Report the rate the mixer will actually use.
	float pitch = atomic_load_f32(&voice->params.pitch);
	return pitch <= 0 ? 1 : fminf(AU_PITCH_MAX, fmaxf(AU_PITCH_MIN, pitch));
}

///////////////////////////////////////////

void sound_inst_set_spread(sound_inst_t sound_inst, float spread_pct) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	// For shaped voices this sets the floor the shape's own width sits on.
	voice->base_spread = spread_pct;
	atomic_store_f32(&voice->params.spread, spread_pct);
}

///////////////////////////////////////////

float sound_inst_get_spread(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return 0;

	return fmaxf(0, fminf(1, atomic_load_f32(&voice->params.spread)));
}

///////////////////////////////////////////

void sound_inst_set_cutoff(sound_inst_t sound_inst, float cutoff_hz) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	atomic_store_f32(&voice->params.cutoff, cutoff_hz);
}

///////////////////////////////////////////

void sound_inst_set_paused(sound_inst_t sound_inst, bool32_t paused) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	atomic_store_i32(&voice->params.paused, paused ? 1 : 0);
}

///////////////////////////////////////////

bool32_t sound_inst_get_paused(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return false;

	return atomic_load_i32(&voice->params.paused) != 0;
}

///////////////////////////////////////////

void sound_inst_seek(sound_inst_t sound_inst, uint64_t sample) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;

	// Streams only read forward, the mixer ignores this for them. The
	// public sample position converts to the mixer's frame cursor.
	atomic_store_u64(&voice->params.seek_request, sample / sound_inst_channels(voice));
}

///////////////////////////////////////////

uint64_t sound_inst_get_cursor(sound_inst_t sound_inst) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return 0;

	// Frame cursor back out to the public interleaved-sample position.
	return atomic_load_u64(&voice->cursor) * sound_inst_channels(voice);
}

///////////////////////////////////////////

void sound_inst_set_shape(sound_inst_t sound_inst, const vec3 *in_arr_points, int32_t point_count, float radius) {
	au_voice_t* voice = sound_inst_voice(sound_inst);
	if (voice == nullptr) return;
	if (in_arr_points == nullptr || point_count <= 0) { voice->shape_count = 0; return; }

	audio_voice_shape_set(voice, in_arr_points, point_count, radius, audio_listener_get().position);
}

}
