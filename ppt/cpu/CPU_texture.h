// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <curand_kernel.h>

// main libraries
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <fstream>
#include <filesystem>
#include <exception>

#include <ppt/util/Utility.h>

namespace penguinPT {
	class CPU_float_texture {
	public:
		// constructors
		CPU_float_texture() : tex_data(nullptr), width(0), height(0) {}
		~CPU_float_texture() {}

		// samplers
		__hostdev__ float sample_int(int x, int y) const;
		__hostdev__ float sample_float(float u, float v) const;
	
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
		x = util::clamp((x + width) % (width - 1), 0, width - 1);
		y = util::clamp((y + height) % (height - 1), 0, height - 1);

		int idx = (x + y * width) * 4;
		return make_float4(tex_data[idx], tex_data[idx + 1], tex_data[idx + 2], tex_data[idx + 3]);
	}
	float4 CPU_float4_texture::sample_float(float u, float v) const{
		int x = util::clamp(((int)(u * width) + width) % (width - 1), 0, width - 1);
		int y = util::clamp(((int)(v * height) + height) % (height - 1), 0, height - 1);

		int idx = (x + y * width) * 4;
		return make_float4(tex_data[idx], tex_data[idx + 1], tex_data[idx + 2], tex_data[idx + 3]);
	}
	float CPU_float_texture::sample_int(int x, int y) const { 
		x = util::clamp((x + width) % (width - 1), 0, width - 1);
		y = util::clamp((y + height) % (height - 1), 0, height - 1);

		return tex_data[x + y * width]; 
	}
	float CPU_float_texture::sample_float(float u, float v) const { 
		int x = util::clamp(((int)(u * width) + width) % (width - 1), 0, width - 1);
		int y = util::clamp(((int)(v * height) + height) % (height - 1), 0, height - 1);

		return tex_data[x + y  * width]; 
	}

}