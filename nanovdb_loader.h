#pragma once

namespace penguinPT::loader {
	class nanovdb_loader {
	public:
		nanovdb_loader() {}
		~nanovdb_loader() {}

		std::vector<Volume> vol_list;
		unsigned int volumes_num = 0;

		bool load_nvdb(std::string path);
		void volume_parameters(unsigned int index, float s_T, nanovdb::Vec3f albedo_, float g_);
		void set_tranforms(unsigned int index, float scale_, nanovdb::Vec3f position_);
		bool send_to_scene(Scene_data& scene);
	};

	bool nanovdb_loader::load_nvdb(std::string path) {
		Volume candidate;
		try {
			// create volume
			auto handle = nanovdb::io::readGrid<nanovdb::CudaDeviceBuffer>("assets/volumes/" + path);

			handle.deviceUpload(); // Copy the NanoVDB grid to the GPU asynchronously

			candidate.density_grid = handle.deviceGrid<float>(); // get a (raw) pointer to a NanoVDB grid of value type float on the GPU

			if (!candidate.density_grid)
				throw std::runtime_error("GridHandle did not contain a grid with value type float");
		}
		catch (const std::exception& e) {
			std::cerr << "An exception occurred: \"" << e.what() << "\"" << std::endl;
			return false;
		}
		volumes_num++;
		vol_list.push_back(candidate);
		return true;
	}

	void nanovdb_loader::volume_parameters(unsigned int index, float s_T, nanovdb::Vec3f albedo_, float g_) {
		Volume& candidate = vol_list.at(index);

		candidate.sigma_t = s_T;
		candidate.albedo = albedo_;
		candidate.g = g_;
	}
	void nanovdb_loader::set_tranforms(unsigned int index, float scale_, nanovdb::Vec3f position_) {
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