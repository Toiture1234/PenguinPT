#pragma once

namespace penguinPT::loader {
	class nanovdb_loader {
	public:
		nanovdb_loader() {}
		~nanovdb_loader() {}

		// to check if a grid has already been loaded, so to not load multiple times the same object
		std::vector<std::string> grid_names;
		std::vector<nanovdb::GridHandle<>> handles_cpu;

		std::vector<Volume> vol_list;
		unsigned int volumes_num = 0;

		bool load_nvdb(std::string path, bool use_gpu);
		void volume_parameters(unsigned int index, nanovdb::Vec3f s_T, nanovdb::Vec3f albedo_, float g_, nanovdb::Vec3f emission_);
		void set_tranforms(unsigned int index, float scale_, nanovdb::Vec3f position_);
		bool send_to_scene(Scene_data& scene);
		void clean();
	};

	void nanovdb_loader::clean() {
		grid_names.clear();
		handles_cpu.clear();
	}
	
	bool nanovdb_loader::load_nvdb(std::string path, bool use_gpu) {
		Volume candidate;

		std::cout << "GPU : " << use_gpu << "\n";
		int i;
		if (file_util::is_word_in_list(path, grid_names, i)) {
			std::cout << "COPYED density grid !\n";
			candidate.density_grid = vol_list.at(i).density_grid;
		}
		else {
			try {
				// create volume
				if (use_gpu) {
					auto handle = nanovdb::io::readGrid<nanovdb::CudaDeviceBuffer>("assets/volumes/" + path);

					handle.deviceUpload(); // Copy the NanoVDB grid to the GPU asynchronously

					candidate.density_grid = handle.deviceGrid<float>(); // get a (raw) pointer to a NanoVDB grid of value type float on the GPU
				}
				else {
					handles_cpu.push_back(nanovdb::io::readGrid("assets/volumes/" + path));
					candidate.density_grid = handles_cpu.back().grid<float>();
				}

				if (!candidate.density_grid)
					throw std::runtime_error("GridHandle did not contain a grid with value type float");
			}
			catch (const std::exception& e) {
				std::cerr << "An exception occurred: \"" << e.what() << "\"" << std::endl;
				return false;
			}
		}
		volumes_num++;
		grid_names.push_back(path);
		vol_list.push_back(candidate);
		return true;
	}

	void nanovdb_loader::volume_parameters(unsigned int index, nanovdb::Vec3f s_T, nanovdb::Vec3f albedo_, float g_, nanovdb::Vec3f emission_) {
		if (index >= volumes_num) return;
		Volume& candidate = vol_list.at(index);

		candidate.sigma_t = s_T;
		candidate.albedo = albedo_;
		candidate.g = g_;
		candidate.emission = emission_;
	}
	void nanovdb_loader::set_tranforms(unsigned int index, float scale_, nanovdb::Vec3f position_) {
		if (index >= volumes_num) return;
		Volume& candidate = vol_list.at(index);

		candidate.scale = scale_;
		candidate.position = position_;
	}
	bool nanovdb_loader::send_to_scene(Scene_data& scene) {
		scene.empty_volumes(volumes_num);
		for (int i = 0; i < volumes_num; i++) {
			scene.volumes[i] = vol_list.at(i);
		}

		vol_list.clear();
	}
}