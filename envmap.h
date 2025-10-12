#pragma once

namespace penguinPT {
	class Envmap {
	public:
		__hostdev__ Envmap() : width(0), height(0), strength(1.f) {}
		__hostdev__ ~Envmap() {}

		cudaTextureObject_t image = 0;
		

		unsigned int width, height;
		float strength;

		__device__ nanovdb::Vec3f eval_envmap(nanovdb::Vec3f wi, float& pdf);
	};

	namespace loader {
		class envmap_loader {
		public:
			float* data;
			unsigned int width, height;

			__hostdev__ envmap_loader() : data(nullptr), width(0), height(0) {}
			__hostdev__ ~envmap_loader() {
				free(data);
			}

			bool load_from_file(std::string path);
			bool send_to_gpu(Envmap& source, cudaTextureFilterMode filter) const;
		};
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool penguinPT::loader::envmap_loader::load_from_file(std::string path) {
	if (!load_hdr_float4(&data, &width, &height, path.c_str())) {
		return false;
	}
	printf("Envmap %s has been loaded correctly.\n", path.c_str());
	return true;
}

bool penguinPT::loader::envmap_loader::send_to_gpu(Envmap& source, cudaTextureFilterMode filter) const {
	source.width = this->width;
	source.height = this->height;

	cudaArray_t data_cuda = 0;

	try {
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

		cudaCreateTextureObject(&source.image, &resDesc, &texDesc, NULL);
	}
	catch (const std::exception& e) {
		std::cerr << "FAILED TO SEND HDR TEXTURE TO ENVMAP OBJECT.\"" << e.what() << "\"" << std::endl;
		return false;
	}
	return true;
}

nanovdb::Vec3f penguinPT::Envmap::eval_envmap(nanovdb::Vec3f wi, float& pdf) {
	float theta = acosf(CLAMP(wi[1], -1.0f, 1.0f));
	float2 uv = make_float2((PI + atan2f(wi[2], wi[0])) * INV_TWO_PI, theta * INV_PI);

	float4 read = tex2D<float4>(this->image, uv.x, uv.y);
	return { read.x * strength, read.y * strength, read.z * strength };
	//return nanovdb::Vec3f(1.f);

	// skip pdf part now
	/*float pdfNorm = (float)r_Data.envmap.width * (float)r_Data.envmap.height * INV_TWO_PI * INV_PI / r_Data.envmap.sum;

	float pdf = Luminance(color[0], color[1], color[2]) * pdfNorm;
	float sin_theta = sinf(theta);

	pdf /= sin_theta;

	if (sin_theta == 0.f) pdf = 0.f;
	return { color[0], color[1], color[2], pdf };*/
}