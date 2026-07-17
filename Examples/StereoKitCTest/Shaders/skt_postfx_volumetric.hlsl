#include "stereokit.hlsli"

//--name = skt/postfx_volumetric
//--density   = 0.25
//--intensity = 0.8
//--max_dist  = 6

// Volumetric fog as a tile-friendly post-process! Each pixel reconstructs
// its world-space view ray from the depth input attachment, then marches it
// through the shadow demo's shadow map - fog in lit air scatters light
// toward the camera, fog in shadow doesn't, which makes light shafts.
float density;
float intensity;
float max_dist;

// The shadow demo's globally-bound shadow resources, see demo_shadows.cpp
cbuffer shadow_buffer : register(b13) {
	float4x4 shadowmap_transform;
	float3   light_direction;
	float    shadowmap_bias;
	float3   light_color;
	float    shadowmap_pixel_size;
};
Texture2D              shadow_map   : register(t13);
SamplerComparisonState shadow_map_s : register(s13);

[[vk::input_attachment_index(0)]] SubpassInput<float4> color;
[[vk::input_attachment_index(1)]] SubpassInput<float>  depth;

struct psIn {
	float4 pos : SV_POSITION;
	float2 uv  : TEXCOORD0;
};

psIn vs(uint id : SV_VertexID) {
	psIn o;
	float2 uv = float2(id & 2, (id << 1) & 2);
	o.pos = float4(uv * float2(2, -2) + float2(-1, 1), 0, 1);
	o.uv  = uv;
	return o;
}

// 1 where the world position can see the light, 0 in shadow. Positions
// outside the shadow map count as lit.
float lit_at(float3 world) {
	float4 shadow_pos = mul(float4(world, 1), shadowmap_transform);
	float3 uv         = float3(shadow_pos.xy * float2(0.5, -0.5) + 0.5, shadow_pos.z / shadow_pos.w);
	if (any(saturate(uv.xy) != uv.xy)) return 1;
	return shadow_map.SampleCmpLevelZero(shadow_map_s, uv.xy, uv.z);
}

float4 ps(psIn input, uint view_id : SV_ViewID) : SV_TARGET {
	float4 scene = color.SubpassLoad();
	float  d     = depth.SubpassLoad();

	// Unproject to a view-space position, then to world space using this
	// eye's camera: world = view_pos * R^-1 + camera_pos. The view matrix's
	// rotation is orthonormal, so its inverse is just the transpose. uv is
	// texture-space (0,0 top-left) while NDC has y up, so v inverts.
	float2 ndc  = float2(input.uv.x * 2 - 1, 1 - input.uv.y * 2);
	float4 vpos = mul(float4(ndc, d, 1), sk_proj_inv[view_id]);
	vpos /= vpos.w;
	float3 cam   = sk_camera_pos[view_id].xyz;
	float3 world = mul(vpos.xyz, transpose((float3x3)sk_view[view_id])) + cam;

	// March the ray from the camera to the surface (or max_dist for sky),
	// accumulating how much of the air along it is lit.
	float3 ray = world - cam;
	float  len = min(length(ray), max_dist);
	float3 dir = normalize(ray);

	// Interleaved gradient noise offsets each ray, trading banding for noise
	const int STEPS  = 24;
	float dither     = frac(52.9829189 * frac(dot(input.pos.xy, float2(0.06711056, 0.00583715))));
	float step_len   = len / STEPS;
	float t          = step_len * dither;
	float lit        = 0;
	[loop]
	for (int i = 0; i < STEPS; i++) {
		lit += lit_at(cam + dir * t);
		t   += step_len;
	}
	lit /= STEPS;

	// Simple homogeneous in-scattering, no extinction - shafts add light,
	// the scene keeps its color.
	float scatter = (1 - exp(-len * density)) * lit * intensity;
	return float4(scene.rgb + light_color * scatter, scene.a);
}
