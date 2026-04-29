#pragma once

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

		Scene_data scene;
		
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

		void send_to_GPU_data() {
			if (!CUDA_CAPABLE_GPU) return;
			scene.send_to_gpu_solid();
			scene.send_to_gpu_volumes();
			scene.send_to_gpu_BSDF();
		}

		void clean() {
			if (CUDA_CAPABLE_GPU) {
				if (scene.num_of_bsdf != 0) { 
					// destroy every texture object
					//for (int i = 0; i < scene.num_of_bsdf; i++) if (scene.bsdf_list[i].use_albedo_tex) CUDA_CHECK(cudaDestroyTextureObject(scene.bsdf_list[i].albedo_tex));
					CUDA_CHECK(cudaFree(scene.bsdf_list)); 
				}
				if (scene.num_of_triangles != 0) CUDA_CHECK(cudaFree(scene.triangles), free(scene.triangle_indicies));
				if (scene.nodes_used != 0) CUDA_CHECK(cudaFree(scene.nodes));
				if (scene.num_of_volumes != 0) CUDA_CHECK(cudaFree(scene.volumes));

				// free envmap
				CUDA_CHECK(cudaDestroyTextureObject(scene.environnement_map.image));
				CUDA_CHECK(cudaDestroyTextureObject(scene.environnement_map.cdf));
			}
			else {
				if (scene.num_of_bsdf != 0) free(scene.bsdf_list);
				if (scene.num_of_triangles != 0) free(scene.triangles), free(scene.triangle_indicies);
				if (scene.nodes_used != 0) free(scene.nodes);
				if (scene.num_of_volumes != 0) free(scene.volumes);
			}
			free(host_pixel_buffer);
			std::cout << "Renderer service freed.\n";
		}
	};
}
