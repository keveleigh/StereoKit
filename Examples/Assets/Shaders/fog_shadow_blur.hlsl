//--name = app/fog_shadow_blur

// Downsamples the shadow map into a small average-depth texture for the
// volumetric fog. A 256² r16f target is 128KB - cache-resident with cheap
// 2-byte-texel filtering - so marching it is far faster than the raw 2MB
// shadow map, and the soft ALU visibility test it enables is what keeps a
// low step count from looking noisy.
Texture2D source : register(t0);

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

float4 ps(psIn input) : SV_TARGET {
	uint width, height;
	source.GetDimensions(width, height);

	int2  base = int2(input.uv * float2(width, height)) - 4;
	float avg  = 0;
	for (int y = 0; y < 8; y++) {
		for (int x = 0; x < 8; x++) {
			int2 p = clamp(base + int2(x, y), int2(0, 0), int2(width - 1, height - 1));
			avg += source.Load(int3(p, 0)).r;
		}
	}
	return float4(avg / 64, 0, 0, 1);
}
