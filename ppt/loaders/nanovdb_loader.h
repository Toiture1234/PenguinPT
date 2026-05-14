// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/core/volumes.h>
#include <ppt/core/intersectors.h>
#include <ppt/util/Matrix.h>

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
		void volume_parameters(unsigned int index, nanovdb::Vec3f s_T, nanovdb::Vec3f albedo_, float g_, nanovdb::Vec3f emission_, float dM = 1.f);
		void set_tranforms(unsigned int index, math::Mat4f transform);
		//bool send_to_scene(Scene_data& scene, bool is_gpu_available);
		void clean();
		int gifn(std::string name);
	};

	void nanovdb_loader::clean() {
		grid_names.clear();
		handles_cpu.clear();
		volumes_num = 0;
	}
	
	bool nanovdb_loader::load_nvdb(std::string path, bool use_gpu) {
		Volume candidate;

		int i;
		if (file_util::is_word_in_list(path, grid_names, i)) {
			std::cout << "Same volume data already in memory.\n";
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
				std::cerr << "An exception occurred : \"" << e.what() << "\"" << std::endl;
				return false;
			}
		}
		volumes_num++;
		grid_names.push_back(path);
		vol_list.push_back(candidate);
		return true;
	}

	void nanovdb_loader::volume_parameters(unsigned int index, nanovdb::Vec3f s_T, nanovdb::Vec3f albedo_, float g_, nanovdb::Vec3f emission_, float dM) {
		if (index >= volumes_num) return;
		Volume& candidate = vol_list.at(index);

		candidate.sigma_t = s_T;
		candidate.albedo = albedo_;
		candidate.g = g_;
		candidate.emission = emission_;
		candidate.density_mult = dM;
	}
	void nanovdb_loader::set_tranforms(unsigned int index, math::Mat4f transform) {
		if (index >= volumes_num) return;
		Volume& candidate = vol_list.at(index);

		candidate.transform_matrix = transform;
		candidate.transform_matrix_inverse = candidate.transform_matrix.inverse();
	}
	/*bool nanovdb_loader::send_to_scene(Scene_data& scene, bool is_gpu_available) {
		scene.empty_volumes(volumes_num, is_gpu_available);
		for (int i = 0; i < volumes_num; i++) {
			scene.volumes[i] = vol_list.at(i);
		}

		vol_list.clear();
		return true;
	}*/
	int nanovdb_loader::gifn(std::string name) {
		for (int i = 0; i < grid_names.size(); i++) {
			if (name == grid_names.at(i)) return i;
		}
		return -1;
	}
}