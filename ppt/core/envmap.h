// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT
#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/cpu/CPU_texture.h>
#include <ppt/loaders/hdr_loader.h>

namespace penguinPT {
	class Envmap {
	public:
		__hostdev__ Envmap() : width(0), height(0), strength(1.f), sum(0.f), angle(0.f) {}
		__hostdev__ ~Envmap() {}

		// GPU stuff
		cudaTextureObject_t image = 0;
		cudaTextureObject_t cdf = 0;

		// cpu stuff
		CPU_float4_texture image_cpu;
		CPU_float_texture cdf_cpu;

		unsigned int width, height;
		float strength;
		float angle;

		float sum;

		__device__ nanovdb::Vec3f eval_envmap(nanovdb::Vec3f wi, float& pdf);
		__device__ nanovdb::Vec3f sample_envmap(nanovdb::Vec3f& L, float& pdf, Rand_state& state);
		__device__ float2 binarySearch(float xi);

		__host__ nanovdb::Vec3f eval_envmap_host(nanovdb::Vec3f wi, float& pdf);
		__host__ nanovdb::Vec3f sample_envmap_host(nanovdb::Vec3f& L, float& pdf, Rand_state& state);
		__host__ float2 binarySearch_host(float xi);
	};

	namespace loader {
		class envmap_loader {
		public:
			float* data;
			float* cdf;
			unsigned int width, height;
			float sum;

			__hostdev__ envmap_loader() : data(nullptr), width(0), height(0), sum(0.f), cdf(nullptr) {}
			__hostdev__ ~envmap_loader() {}

			bool load_from_file(std::string path);

			// This function isn't really only sending data to GPU but also sets CPU data,
			// so it needs to be called even if the GPU is not used.
			bool send_to_gpu(Envmap& source, cudaTextureFilterMode filter);
			void build_CDF();

			// keep arrays in hand to delete them later
			cudaArray_t data_cuda = 0;
			cudaArray_t data_cuda_cdf = 0;

			void clean();
		};
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool penguinPT::loader::envmap_loader::load_from_file(std::string path) {
	if (!load_hdr_float4(&data, &width, &height, path.c_str())) {
		return false;
	}
	printf("Envmap %s has been loaded correctly.\n", path.c_str());

	build_CDF();
	printf("Envmap CDF built correctly.\n");

	std::cout << "Envmap characteristics : \n";
	std::cout << " - Height : " << height << "\n";
	std::cout << " - Width : " << width << "\n";
	std::cout << " - Sum : " << sum << "\n";
	std::cout << "-------------------------------------------------\n\n";
	return true;
}
__hostdev__ inline float Luminance(float r, float g, float b)
{
	return 0.212671 * r + 0.715160 * g + 0.072169 * b;
}
void penguinPT::loader::envmap_loader::build_CDF() {
	float* weights = new float[width * height];
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int idx = x * 4 + y * width * 4;
			weights[x + y * width] = Luminance(data[idx], data[idx + 1], data[idx + 2]);
		}
	}

	cdf = new float[width * height];
	cdf[0] = weights[0];
	for (int i = 1; i < width * height; i++) {
		cdf[i] = cdf[i - 1] + weights[i];
	}
	sum = cdf[width * height - 1];

	delete[] weights;
}

void penguinPT::loader::envmap_loader::clean() {
	if (height == 0) return; // envmap is already cleaned
	try
	{
		if (cudaFreeArray(data_cuda) != cudaSuccess || cudaFreeArray(data_cuda_cdf) != cudaSuccess)
			throw std::exception("Warning : Error while freeing CUDA evmap, this message is normal if your device doesn't support CUDA.");
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << "\"" << std::endl;
	}
	
	free(data);
	free(cdf);
	data = nullptr, width = 0, height = 0, sum = 0.f, cdf = nullptr;
}
bool penguinPT::loader::envmap_loader::send_to_gpu(Envmap& source, cudaTextureFilterMode filter) {
	source.width = this->width;
	source.height = this->height;
	source.sum = sum;

	// GPU
	try {
		// generate pixel texture
		{
			cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float4>();
			size_t spitch = width * sizeof(float4);
			cudaMallocArray(&data_cuda, &channelDesc, width, height);
			cudaMemcpy2DToArray(data_cuda, 0, 0, this->data, spitch, this->width * sizeof(float4), this->height, cudaMemcpyHostToDevice);

			cudaResourceDesc resDesc;
			memset(&resDesc, 0, sizeof(resDesc));
			resDesc.resType = cudaResourceTypeArray;
			resDesc.res.array.array = data_cuda;

			// default parameters
			cudaTextureDesc texDesc;
			memset(&texDesc, 0, sizeof(texDesc));
			texDesc.addressMode[0] = cudaAddressModeWrap;
			texDesc.addressMode[1] = cudaAddressModeWrap;
			texDesc.filterMode = filter;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = true;

			if (cudaCreateTextureObject(&source.image, &resDesc, &texDesc, NULL) != cudaSuccess) throw std::exception("Failed to create value texture.");
		}

		// generate cdf texture
		{
			cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float>();
			size_t spitch = width * sizeof(float);
			cudaMallocArray(&data_cuda_cdf, &channelDesc, width, height);

			cudaMemcpy2DToArray(data_cuda_cdf, 0, 0, this->cdf, spitch, this->width * sizeof(float), this->height, cudaMemcpyHostToDevice);

			cudaResourceDesc resDesc;
			memset(&resDesc, 0, sizeof(resDesc));
			resDesc.resType = cudaResourceTypeArray;
			resDesc.res.array.array = data_cuda_cdf;

			// default parameters
			cudaTextureDesc texDesc;
			memset(&texDesc, 0, sizeof(texDesc));
			texDesc.addressMode[0] = cudaAddressModeWrap;
			texDesc.addressMode[1] = cudaAddressModeWrap;
			texDesc.filterMode = cudaFilterModeLinear;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = false;

			if(cudaCreateTextureObject(&source.cdf, &resDesc, &texDesc, NULL) != cudaSuccess) throw std::exception("Failed to create cdf texture.");
		}
	}
	catch (const std::exception& e) {
		std::cerr << "Error : " << e.what() << "\"" << std::endl;
	}

	// CPU
	source.image_cpu.tex_data = data;
	source.cdf_cpu.tex_data = cdf;
	source.image_cpu.width = width, source.image_cpu.height = height;
	source.cdf_cpu.width = width, source.cdf_cpu.height = height;

	return true;
}

__device__ nanovdb::Vec3f penguinPT::Envmap::eval_envmap(nanovdb::Vec3f wi, float& pdf) {
	float theta = acosf(CLAMP(wi[1], -1.0f, 1.0f));
	float2 uv = make_float2((PI + atan2f(wi[2], wi[0])) * INV_TWO_PI, theta * INV_PI);
	uv.x += angle;

	float4 read = tex2D<float4>(this->image, uv.x, uv.y);
	
	float pdfNorm = (float)this->width * (float)this->height * INV_TWO_PI * INV_PI / this->sum;

	pdf = Luminance(read.x, read.y, read.z) * pdfNorm;
	float sin_theta = sinf(theta);

	if (sin_theta == 0.f) pdf = 0.f;
	else pdf /= sin_theta;

	return { read.x * strength, read.y * strength, read.z * strength };
}
__device__ inline float2 penguinPT::Envmap::binarySearch(float xi) {
	int lower = 0;
	int upper = this->height - 1;
	while (lower < upper) {
		int mid = (lower + upper) >> 1;
		if (xi < tex2D<float>(cdf, this->width - 1, mid)) {
			upper = mid;
		}
		else {
			lower = mid + 1;
		}
	}
	int y = lower;

	lower = 0;
	upper = this->width - 1;
	while (lower < upper) {
		int mid = (lower + upper) >> 1;
		if (xi < tex2D<float>(cdf, mid, y)) {
			upper = mid;
		}
		else {
			lower = mid + 1;
		}
	}
	int x = lower;
	return make_float2((float)x / (float)this->width, (float)y / (float)this->height);

}

__device__ inline nanovdb::Vec3f penguinPT::Envmap::sample_envmap(nanovdb::Vec3f& L, float& pdf, Rand_state& state) {
	float2 uv = binarySearch(randC(&state) * this->sum);
	
	uv.x -= angle;

	float phi = uv.x * TWO_PI;
	float theta = uv.y * PI;

	float sin_theta = sinf(theta);

	L = nanovdb::Vec3f(-sin_theta * cosf(phi), cosf(theta), -sin_theta * sinf(phi));

	return eval_envmap(L, pdf);
}

__host__ nanovdb::Vec3f penguinPT::Envmap::eval_envmap_host(nanovdb::Vec3f wi, float& pdf) {
	float theta = acosf(CLAMP(wi[1], -1.0f, 1.0f));
	float2 uv = make_float2((PI + atan2f(wi[2], wi[0])) * INV_TWO_PI, theta * INV_PI);
	uv.x += angle;

	float4 read = image_cpu.sample_float(uv.x, uv.y);

	float pdfNorm = (float)this->width * (float)this->height * INV_TWO_PI * INV_PI / this->sum;

	pdf = Luminance(read.x, read.y, read.z) * pdfNorm;
	float sin_theta = sinf(theta);

	if (sin_theta == 0.f) pdf = 0.f;
	else pdf /= sin_theta;

	return { read.x * strength, read.y * strength, read.z * strength };
}
__host__ inline float2 penguinPT::Envmap::binarySearch_host(float xi) {
	int lower = 0;
	int upper = this->height - 1;
	while (lower < upper) {
		int mid = (lower + upper) >> 1;
		if (xi < cdf_cpu.sample_int(this->width - 1, mid)) {
			upper = mid;
		}
		else {
			lower = mid + 1;
		}
	}
	int y = lower;;

	lower = 0;
	upper = this->width - 1;
	while (lower < upper) {
		int mid = (lower + upper) >> 1;
		if (xi < cdf_cpu.sample_int(mid, y)) {
			upper = mid;
		}
		else {
			lower = mid + 1;
		}
	}
	int x = lower;
	return make_float2((float)x / (float)this->width, (float)y / (float)this->height);
}

__host__ inline nanovdb::Vec3f penguinPT::Envmap::sample_envmap_host(nanovdb::Vec3f& L, float& pdf, Rand_state& state) {
	float2 uv = binarySearch_host(rand01 * this->sum);

	uv.x -= angle;

	float phi = uv.x * TWO_PI;
	float theta = uv.y * PI;

	float sin_theta = sinf(theta);

	L = nanovdb::Vec3f(sin_theta * cosf(phi), cosf(theta), sin_theta * sinf(phi));

	return eval_envmap_host(L, pdf);
}