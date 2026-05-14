// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/core/camera.h>
#include <ppt/core/intersectors.h>
#include <ppt/core/scene.h>

#define WINDOW_RES_X 1920
#define WINDOW_RES_Y 1080

namespace penguinPT {
	class renderer_services {
	public:
		unsigned int width = WINDOW_RES_X * (2.f / 3.f), height = WINDOW_RES_Y * (2.f / 3.f);

		bool CUDA_CAPABLE_GPU = false;

		// pixel buffer
		uint8_t* host_pixel_buffer = nullptr;
		unsigned int frame_index = 0;
		float delta_time = 1.f;
		bool is_rendering = false;
		Rand_state rng_state = Rand_state();

		camera mainCam;

		//Scene_data scene;
		Scene scene;
		
		__hostdev__ renderer_services() {}
		__hostdev__ ~renderer_services() {}

		void fill_host_pixel_buffer() {
			host_pixel_buffer = new uint8_t[width * height * 4];
			for (int i = 0; i < width * height * 4; i += 4) {
				host_pixel_buffer[i] = 255;
				host_pixel_buffer[i + 1] = 0;
				host_pixel_buffer[i + 2] = 0;
				host_pixel_buffer[i + 3] = 255;
			}
		}

		void check_CUDA_AVAILABLITY() {
			int GPU_count;
			cudaError_t device_check = cudaGetDeviceCount(&GPU_count);

			if (device_check != CUDA_SUCCESS) {
				std::cout << "No CUDA capable device has been detected. Switch to CPU, please expect low performance.\n";

				CUDA_CAPABLE_GPU = false;

			}
			else {
				std::cout << GPU_count << " CUDA capable device(s) has been detected. Continue.\n";
				
				std::cout << "Please select which device you want to use : \n";
				for (int i = 0; i < GPU_count; i++) {
					cudaDeviceProp current_properties;
					CUDA_CHECK(cudaGetDeviceProperties(&current_properties, i));
					std::cout << "    * Device " << i << " : " << current_properties.name << "\n";
				}
				
				std::cout << "Please select device number : ";
				int device_choice;
				std::cin >> device_choice;

				if (device_choice >= GPU_count) {
					std::cout << "Invalid device ordinal, swiching to CPU render mode, please expect low performance.\n";
					CUDA_CAPABLE_GPU = false;
				}
				else
				{
					CUDA_CAPABLE_GPU = true;
					CUDA_CHECK(cudaSetDevice(device_choice));
				}
			}
			std::cout << "-------------------------------------------------\n\n";
		}

		// partial cleaning : only cleans solid scene data
		void clean_solid() {
			scene.clearTLAS(CUDA_CAPABLE_GPU);
			scene.clearBSDFList(CUDA_CAPABLE_GPU);
		}

		// partial cleaning : only cleans volume scene data
		void clean_volumes() {
			scene.clearVolumeList(CUDA_CAPABLE_GPU);
		}

		// partial cleaning : only cleans envmap scene data
		// this function only affects CUDA and CPU data need to be deleted from envmap_loader::clean()
		void clean_envmap() {
			scene.clearEnvmap(CUDA_CAPABLE_GPU);
		}

		void clean_all() {
			clean_solid();
			clean_volumes();
			clean_envmap();
			free(host_pixel_buffer);
			std::cout << "Renderer service freed.\n";
		}
	};
}
