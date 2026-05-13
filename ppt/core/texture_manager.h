// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <SFML/Graphics.hpp>

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/cpu/CPU_texture.h>

namespace penguinPT {
	class texture_manager {
	public:
		texture_manager() {}
		~texture_manager() {}
		
		// remind every texture path so we don't need to allocate twice the same texture on memory
		std::vector<std::string> texture_names;
		std::vector<sf::Vector2u> texture_sizes;

		// texture data, saved here to be able to delete it later when texture aren't used anymore
		std::vector<cudaArray_t> texture_data_GPU;
		std::vector<float*> texture_data_CPU;

		// Load a texture from a file path and binds it to a CUDA texture object, 
		// the texture data (CUDA array) lives on the texture manager.
		//  - float4 version
		bool load_texture_GPU_float4(std::string path, cudaTextureObject_t& tex);

		// Load a texture from a file path and binds it to a CUDA texture object, 
		// the texture data (CUDA array) lives on the texture manager.
		//  - float version
		bool load_texture_GPU_float(std::string path, cudaTextureObject_t& tex);

		// Load a texture from a file path and binds it to a CPU texture object,
		// the texture data (float pointer) lives on the texture manager.
		//  - float4 version
		bool load_texture_CPU_float4(std::string path, CPU_float4_texture& tex);

		// Load a texture from a file path and binds it to a CPU texture object,
		// the texture data (float pointer) lives on the texture manager.
		//  - float version
		bool load_texture_CPU_float(std::string path, CPU_float_texture& tex);

		void clean();
	};

	// texture manager functions
	bool texture_manager::load_texture_GPU_float4(std::string path, cudaTextureObject_t& tex) {

		cudaResourceDesc resDesc;
		int i;
		if (file_util::is_word_in_list(path, texture_names, i)) {
			std::cout << "Reuse texture to save memory.\n";
			int width = texture_sizes.at(i).x, height = texture_sizes.at(i).y;
			cudaArray_t& temp_array = texture_data_GPU.at(i);

			memset(&resDesc, 0, sizeof(resDesc));
			resDesc.resType = cudaResourceTypeArray;
			resDesc.res.array.array = temp_array;

			// default parameters
			cudaTextureDesc texDesc;
			memset(&texDesc, 0, sizeof(texDesc));
			texDesc.addressMode[0] = cudaAddressModeWrap;
			texDesc.addressMode[1] = cudaAddressModeWrap;
			texDesc.filterMode = cudaFilterModeLinear;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = true;
			
			cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
		}
		else {
			std::cout << "Initialize new texture.\n";
			float* tex_data;
			int width, height;
			//if (!get_texture_array(path, tex_data, width, height)) return false;

			sf::Image temp_sfImage;
			if (!temp_sfImage.loadFromFile(path)) {
				std::cout << "ERROR - Failed to load texture " << path << "\n";
				return false;
			}

			sf::Vector2u size = temp_sfImage.getSize();

			tex_data = new float[size.x * size.y * 4];
			for (int x = 0; x < size.x; x++) {
				for (int y = 0; y < size.y; y++) {
					sf::Color px_color = temp_sfImage.getPixel(x, size.y - y - 1);

					int idx = x + y * size.x;
					tex_data[idx * 4] = (float)px_color.r / 255.f;
					tex_data[idx * 4 + 1] = (float)px_color.g / 255.f;
					tex_data[idx * 4 + 2] = (float)px_color.b / 255.f;
					tex_data[idx * 4 + 3] = (float)px_color.a / 255.f;
				}
			}

			width = size.x;
			height = size.y;

			// add texture to array list
			texture_names.push_back(path);
			texture_sizes.push_back(sf::Vector2u(width, height));

			// create new cudaArray_t
			texture_data_GPU.push_back(0);
			cudaArray_t& temp_array = texture_data_GPU.back();

			// send texture directly to GPU
			cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float4>();
			size_t spitch = width * sizeof(float4);
			cudaMallocArray(&temp_array, &channelDesc, width, height);
			CUDA_CHECK(cudaMemcpy2DToArray(temp_array, 0, 0, tex_data, spitch, width * sizeof(float4), height, cudaMemcpyHostToDevice));
			
			memset(&resDesc, 0, sizeof(resDesc));
			resDesc.resType = cudaResourceTypeArray;
			resDesc.res.array.array = temp_array;

			// default parameters
			cudaTextureDesc texDesc;
			memset(&texDesc, 0, sizeof(texDesc));
			texDesc.addressMode[0] = cudaAddressModeWrap;
			texDesc.addressMode[1] = cudaAddressModeWrap;
			texDesc.filterMode = cudaFilterModeLinear;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = true;
			
			cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
		}
		
		return true;
	}
	bool texture_manager::load_texture_GPU_float(std::string path, cudaTextureObject_t& tex) {

		cudaResourceDesc resDesc;
		int i;
		if (file_util::is_word_in_list(path, texture_names, i)) {
			std::cout << "Reuse texture to save memory.\n";
			int width = texture_sizes.at(i).x, height = texture_sizes.at(i).y;
			cudaArray_t& temp_array = texture_data_GPU.at(i);

			memset(&resDesc, 0, sizeof(resDesc));
			resDesc.resType = cudaResourceTypeArray;
			resDesc.res.array.array = temp_array;

			// default parameters
			cudaTextureDesc texDesc;
			memset(&texDesc, 0, sizeof(texDesc));
			texDesc.addressMode[0] = cudaAddressModeWrap;
			texDesc.addressMode[1] = cudaAddressModeWrap;
			texDesc.filterMode = cudaFilterModeLinear;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = true;

			cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
		}
		else {
			std::cout << "Initialize new texture.\n";
			float* tex_data;
			int width, height;
			
			sf::Image temp_sfImage;
			if (!temp_sfImage.loadFromFile(path)) {
				std::cout << "ERROR - Failed to load texture " << path << "\n";
				return false;
			}

			sf::Vector2u size = temp_sfImage.getSize();

			tex_data = new float[size.x * size.y];
			for (int x = 0; x < size.x; x++) {
				for (int y = 0; y < size.y; y++) {
					sf::Color px_color = temp_sfImage.getPixel(x, size.y - y - 1);

					int idx = x + y * size.x;
					tex_data[idx] = (float)px_color.r / 255.f;
				}
			}

			width = size.x;
			height = size.y;

			// add texture to array list
			texture_names.push_back(path);
			texture_sizes.push_back(sf::Vector2u(width, height));

			// create new cudaArray_t
			texture_data_GPU.push_back(0);
			cudaArray_t& temp_array = texture_data_GPU.back();

			// send texture directly to GPU
			cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float>();
			size_t spitch = width * sizeof(float);
			cudaMallocArray(&temp_array, &channelDesc, width, height);
			CUDA_CHECK(cudaMemcpy2DToArray(temp_array, 0, 0, tex_data, spitch, width * sizeof(float), height, cudaMemcpyHostToDevice));

			memset(&resDesc, 0, sizeof(resDesc));
			resDesc.resType = cudaResourceTypeArray;
			resDesc.res.array.array = temp_array;

			// default parameters
			cudaTextureDesc texDesc;
			memset(&texDesc, 0, sizeof(texDesc));
			texDesc.addressMode[0] = cudaAddressModeWrap;
			texDesc.addressMode[1] = cudaAddressModeWrap;
			texDesc.filterMode = cudaFilterModeLinear;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = true;

			cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
		}

		return true;
	}

	bool texture_manager::load_texture_CPU_float4(std::string path, CPU_float4_texture& tex) {
		int i;
		if (file_util::is_word_in_list(path, texture_names, i)) {
			std::cout << "Reuse texture to save memory.\n";
			int width = texture_sizes.at(i).x, height = texture_sizes.at(i).y;

			tex.width = width;
			tex.height = height;
			tex.tex_data = texture_data_CPU.at(i);
		}
		else {
			std::cout << "Initialize new texture.\n";
			float* tex_data;
			int width, height;
			
			sf::Image temp_sfImage;
			if (!temp_sfImage.loadFromFile(path)) {
				std::cout << "ERROR - Failed to load texture " << path << "\n";
				return false;
			}

			sf::Vector2u size = temp_sfImage.getSize();

			tex_data = new float[size.x * size.y * 4];
			for (int x = 0; x < size.x; x++) {
				for (int y = 0; y < size.y; y++) {
					sf::Color px_color = temp_sfImage.getPixel(x, size.y - y - 1);

					int idx = x + y * size.x;
					tex_data[idx * 4] = (float)px_color.r / 255.f;
					tex_data[idx * 4 + 1] = (float)px_color.g / 255.f;
					tex_data[idx * 4 + 2] = (float)px_color.b / 255.f;
					tex_data[idx * 4 + 3] = (float)px_color.a / 255.f;
				}
			}

			width = size.x;
			height = size.y;

			// add texture to array list
			texture_names.push_back(path);
			texture_sizes.push_back(sf::Vector2u(width, height));

			texture_data_CPU.push_back(tex_data);

			tex.width = width;
			tex.height = height;
			tex.tex_data = texture_data_CPU.back();
		}
		return true;
	}
	bool texture_manager::load_texture_CPU_float(std::string path, CPU_float_texture& tex) {
		int i;
		if (file_util::is_word_in_list(path, texture_names, i)) {
			std::cout << "Reuse texture to save memory.\n";
			int width = texture_sizes.at(i).x, height = texture_sizes.at(i).y;

			tex.width = width;
			tex.height = height;
			tex.tex_data = texture_data_CPU.at(i);
		}
		else {
			std::cout << "Initialize new texture.\n";
			float* tex_data;
			int width, height;

			sf::Image temp_sfImage;
			if (!temp_sfImage.loadFromFile(path)) {
				std::cout << "ERROR - Failed to load texture " << path << "\n";
				return false;
			}

			sf::Vector2u size = temp_sfImage.getSize();

			tex_data = new float[size.x * size.y * 4];
			for (int x = 0; x < size.x; x++) {
				for (int y = 0; y < size.y; y++) {
					sf::Color px_color = temp_sfImage.getPixel(x, size.y - y - 1);

					int idx = x + y * size.x;
					tex_data[idx] = (float)px_color.r / 255.f;
				}
			}

			width = size.x;
			height = size.y;

			// add texture to array list
			texture_names.push_back(path);
			texture_sizes.push_back(sf::Vector2u(width, height));

			texture_data_CPU.push_back(tex_data);

			tex.width = width;
			tex.height = height;
			tex.tex_data = texture_data_CPU.back();
		}
		return true;
	}

	void texture_manager::clean() {
		texture_names.clear();
		texture_sizes.clear();

		for (cudaArray_t& t : texture_data_GPU) CUDA_CHECK(cudaFreeArray(t));

		texture_data_GPU.clear();
		texture_data_CPU.clear();
	}
}