#pragma once

namespace penguinPT {
	class renderer_services {
	public:
		unsigned int width = 1280, height = 720;

		// pixel buffer
		uint8_t* host_pixel_buffer = nullptr;
		unsigned int frame_index = 0;
		float delta_time = 1.f;
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
	};
}
