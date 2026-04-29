#pragma once

namespace penguinPT {
	class CPU_float_texture {
	public:
		// constructors
		CPU_float_texture() : tex_data(nullptr), width(0), height(0) {}
		~CPU_float_texture() {}

		// samplers
		__hostdev__ float sample_int(int x, int y) const { return tex_data[CLAMP(x, 0, width - 1) + CLAMP(y,0, height - 1) * width]; }
		__hostdev__ float sample_float(float u, float v) const { int x = (int)(u * width), y = (int)(v * height); return tex_data[CLAMP(x, 0, width - 1) + CLAMP(y, 0, height - 1) * width]; }
	
		// texture data
		float* tex_data;
		int width;
		int height;
	};
	class CPU_float4_texture {
	public:
		// constructors
		CPU_float4_texture() : tex_data(nullptr), width(0), height(0) {}
		~CPU_float4_texture() {}

		// samplers
		__hostdev__ float4 sample_int(int x, int y) const;
		__hostdev__ float4 sample_float(float u, float v) const;
	
		// texture data
		float* tex_data;
		int width;
		int height;
	};

	float4 CPU_float4_texture::sample_int(int x, int y) const{
		x = CLAMP(x, 0, width - 1);
		y = CLAMP(y, 0, height - 1);

		int idx = (x + y * width) * 4;
		return make_float4(tex_data[idx], tex_data[idx + 1], tex_data[idx + 2], tex_data[idx + 3]);
	}
	float4 CPU_float4_texture::sample_float(float u, float v) const{
		int x = CLAMP(u * width, 0, width - 1);
		int y = CLAMP(v * height, 0, height - 1);

		int idx = (x + y * width) * 4;
		return make_float4(tex_data[idx], tex_data[idx + 1], tex_data[idx + 2], tex_data[idx + 3]);
	}
}