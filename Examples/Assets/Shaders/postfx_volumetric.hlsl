#include "stereokit.hlsli"

//--name = app/postfx_volumetric
//--density   = 0.25
//--intensity = 0.8
//--max_dist  = 6

// Volumetric fog as a tile-friendly post-process! Each pixel reconstructs
// its world-space view ray from the depth input attachment, then marches it
// through the shadow demo's shadow map - fog in lit air scatters light
// toward the camera, fog in shadow doesn't, which makes light shafts.

// Ray march count. A specialization constant, so it's baked into the
// pipeline where the loop can unroll - tune it with SetInt("STEPS", n).
[[vk::constant_id(0)]] const int STEPS = 4;

float density;
float intensity;
float max_dist;

// The shadow demo's globally-bound shadow constants, see DemoShadows.cs
cbuffer shadow_buffer : register(b13) {
	float4x4 shadowmap_transform;
	float3   light_direction;
	float    shadowmap_bias;
	float3   light_color;
	float    shadowmap_pixel_size;
};
// Small prefiltered average-depth map, rebuilt from the shadow map each
// frame by fog_shadow_blur.hlsl. It's cache-resident (256² r16f), so
// marching it is several times faster than the raw shadow map, and its
// soft visibility keeps low step counts smooth.
Texture2D    fog_shadow   : register(t0);
SamplerState fog_shadow_s : register(s0);

[[vk::input_attachment_index(0)]] SubpassInput<float4> color;
[[vk::input_attachment_index(1)]] SubpassInput<float>  depth;

struct psIn {
	float4 pos : SV_POSITION;
	float2 uv  : TEXCOORD0;
};

psIn vs(uint id : SV_VertexID) {
	psIn o;
	o.uv  = float2(id & 2, (id << 1) & 2);
	o.pos = float4(o.uv * float2(2, -2) + float2(-1, 1), 0, 1);
	return o;
}

// Shadow map uv + compare depth for a world position. The light is
// directional (orthographic transform, w = 1), so these coordinates
// interpolate linearly along a ray.
float3 shadow_coord(float3 world) {
	float4 p = mul(float4(world, 1), shadowmap_transform);
	return float3(p.xy * float2(0.5, -0.5) + 0.5, p.z);
}

// Soft visibility from the blurred average depth: a linear ramp instead of
// a hard compare, roughly the fraction of the blurred region that's nearer
// the light. The ramp width is in light-space depth, ~10cm here. Visibility
// only needs a few bits, so the math runs at relaxed (fp16) precision.
min16float lit_at(float3 uvz) {
	min16float avg = (min16float)fog_shadow.SampleLevel(fog_shadow_s, uvz.xy, 0).r;
	return saturate((avg - (min16float)uvz.z) * (min16float)200 + (min16float)0.5);
}

float4 ps(psIn input, uint view_id : SV_ViewID) : SV_TARGET {
	float4 scene = color.SubpassLoad();
	float  d     = depth.SubpassLoad();

	// With intensity zeroed this pass is a plain copy - handy for measuring
	// the fixed cost of the pass itself, separate from the ray march.
	if (intensity <= 0) return scene;

	// Unproject to a view-space position, then to world space using this
	// eye's camera: world = view_pos * R^-1 + camera_pos. The view matrix's
	// rotation is orthonormal, so its inverse is just the transpose. uv is
	// texture-space (0,0 top-left) while NDC has y up, so v inverts.
	float2 ndc  = float2(input.uv.x * 2 - 1, 1 - input.uv.y * 2);
	float4 vpos = mul(float4(ndc, d, 1), sk_proj_inv[view_id]);
	vpos /= vpos.w;
	float3 cam   = sk_camera_pos[view_id].xyz;
	float3 world = mul(vpos.xyz, transpose((float3x3)sk_view[view_id])) + cam;

	// March from the camera to the surface (or max_dist for sky) entirely in
	// shadow map space - orthographic shadow coordinates are linear along the
	// ray, so it's one matrix multiply per endpoint and a lerp per step.
	float3 ray   = world - cam;
	float  len   = max(length(ray), 0.0001);
	float  t_max = min(len, max_dist);
	float3 uvz0  = shadow_coord(cam);
	float3 uvz1  = shadow_coord(cam + ray * (t_max / len));

	// Clip the segment to the shadow map's uv bounds - air outside the map
	// is lit by definition and needs no samples, and every step lands where
	// shadow data actually exists. Rays that miss the map skip the loop
	// entirely. (Light-space z is left unclipped, the light's clip range
	// covers the whole scene.)
	float2 delta = uvz1.xy - uvz0.xy;
	delta = float2(abs(delta.x) < 1e-6 ? 1e-6 : delta.x,
	               abs(delta.y) < 1e-6 ? 1e-6 : delta.y);
	float2 ta    = (0 - uvz0.xy) / delta;
	float2 tb    = (1 - uvz0.xy) / delta;
	float2 tlo   = min(ta, tb);
	float2 thi   = max(ta, tb);
	float  t_in  = saturate(max(tlo.x, tlo.y));
	float  t_out = saturate(min(thi.x, thi.y));
	float  inside = max(0, t_out - t_in);

	// The part of the ray outside the shadow map is lit by definition.
	min16float lit = (min16float)(1 - inside);
	if (inside > 0) {
		// Copied to a local so only the plain OpSpecConstant reaches downstream
		// math - glslang emits invalid spec-op chains for direct vector use.
		int    steps    = STEPS;
		float3 seg      = uvz1 - uvz0;
		float3 uvz_step = seg * (inside / steps);

		// Interleaved gradient noise offsets each ray's start, so a low step
		// count shows as fine grain instead of banding.
		float      dither = frac(52.9829189 * frac(dot(input.pos.xy, float2(0.06711056, 0.00583715))));
		float3     s      = uvz0 + seg * t_in + uvz_step * dither;
		min16float march  = 0;
		for (int i = 0; i < steps; i++) {
			march += lit_at(s);
			s     += uvz_step;
		}
		lit += (min16float)inside * (march / (min16float)steps);
	}

	// Simple homogeneous in-scattering, no extinction - shafts add light,
	// the scene keeps its color. Color math is fp16-friendly too.
	min16float  scatter = (min16float)(1 - exp(-t_max * density)) * lit * (min16float)intensity;
	min16float3 col     = (min16float3)scene.rgb + (min16float3)light_color * scatter;
	return float4(col, scene.a);
}
