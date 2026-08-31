#include "stereokit.hlsli"

//--name = sk/visibility_mask
//--color:color = 0, 0, 0, 1

float4 color;

struct vsIn {
	float4 pos  : SV_Position;
	float3 norm : NORMAL0;
	float2 uv   : TEXCOORD0;
	float4 col  : COLOR0;
};

struct psIn {
	float4 pos   : SV_POSITION;
};

psIn vs(vsIn input, sk_ids_t ids) {
	psIn o;
	
	// OpenXR visibility mask vertices are populated with the target view index in the red channel of the vertex color.
	// If the current view ID doesn't match the target view index, cull the triangle with a NaN position.
	if (round(input.col.r * 255.0) != ids.view) {
		o.pos = asfloat(0x7FC00000);
	} else {
		// Per the OpenXR spec, visibility mask vertices are 2D points on the
		// view-space z=-1 plane (tangents), and must be transformed by the view's
		// projection matrix to account for asymmetric FOV.
		float4 view_pos = float4(input.pos.x, input.pos.y, -1, 1);
		o.pos = mul(view_pos, sk_proj[ids.view]);
		
		// Force the depth to the near plane (0) so it successfully culls all
		// subsequent geometry (which uses depth_test_less_or_eq).
		o.pos.z = 0;
	}
	
	return o;
}

float4 ps(psIn input) : SV_TARGET {
	return color;
}
