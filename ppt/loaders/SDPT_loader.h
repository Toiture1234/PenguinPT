// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/loaders/obj_loader.h>
#include <ppt/loaders/nanovdb_loader.h>
#include <ppt/core/envmap.h>

namespace penguinPT::loader {
	class SDPT_Loader {
	public:
		SDPT_Loader() {}
		~SDPT_Loader() {}

		bool load_sdpt(std::string path, obj_loader* obj_ref, nanovdb_loader* nvdb_ref, envmap_loader* envmap_ref);
		void forceLoad(std::string path, obj_loader* obj_ref, nanovdb_loader* nvdb_ref, envmap_loader* envmap_ref);
		void set_GPU_Compat(bool gpu) { GPU_available = gpu; };
	private:
		bool GPU_available = true;
	};
}

bool penguinPT::loader::SDPT_Loader::load_sdpt(std::string path, obj_loader* obj_ref, nanovdb_loader* nvdb_ref, envmap_loader* envmap_ref) {
	std::ifstream file("assets/scenes/" + path);

	if (file.is_open()) {
		std::string line;

		while (std::getline(file, line)) {
			std::vector<std::string> tokens = file_util::get_line_tokens(line, { ' ' });
			
			if (!tokens.empty()) {
				if (tokens.at(0) == "solid") {
					std::string name = file_util::remove_quote(tokens.at(1));
					nanovdb::Vec3f position = nanovdb::Vec3f(std::stof(tokens.at(2)), std::stof(tokens.at(3)), std::stof(tokens.at(4)));
					nanovdb::Vec3f size = nanovdb::Vec3f(std::stof(tokens.at(5)), std::stof(tokens.at(6)), std::stof(tokens.at(7)));
					nanovdb::Vec3f rotation = nanovdb::Vec3f(std::stof(tokens.at(8)), std::stof(tokens.at(9)), std::stof(tokens.at(10)));

					if (!obj_ref->load_obj(name, position, size[0], GPU_available)) return false;
				}
				else if (tokens.at(0) == "volume") {
					std::string name = file_util::remove_quote(tokens.at(1));
					nanovdb::Vec3f position = nanovdb::Vec3f(std::stof(tokens.at(2)), std::stof(tokens.at(3)), std::stof(tokens.at(4)));
					nanovdb::Vec3f size = nanovdb::Vec3f(std::stof(tokens.at(5)), std::stof(tokens.at(6)), std::stof(tokens.at(7)));
					nanovdb::Vec3f rotation = nanovdb::Vec3f(std::stof(tokens.at(8)), std::stof(tokens.at(9)), std::stof(tokens.at(10)));

					nanovdb::Vec3f sigma_t = nanovdb::Vec3f(std::stof(tokens.at(11)), std::stof(tokens.at(12)), std::stof(tokens.at(13)));
					nanovdb::Vec3f albedo = nanovdb::Vec3f(std::stof(tokens.at(14)), std::stof(tokens.at(15)), std::stof(tokens.at(16)));
					float g = std::stof(tokens.at(17));
					nanovdb::Vec3f emission = nanovdb::Vec3f(std::stof(tokens.at(18)), std::stof(tokens.at(19)), std::stof(tokens.at(20)));
					float d = std::stof(tokens.at(21));

					if (!nvdb_ref->load_nvdb(name, GPU_available)) return false;
					nvdb_ref->set_tranforms(nvdb_ref->volumes_num - 1, size[0], position);
					nvdb_ref->volume_parameters(nvdb_ref->volumes_num - 1, sigma_t, albedo, g, emission, d);
				}
				else if (tokens.at(0) == "envmap") {
					std::string name = file_util::remove_quote(tokens.at(1));
					
					if (!envmap_ref->load_from_file("assets/hdris/" + name)) return false;
				}

			}
		}
		return true;
	}
	return false;
}
void penguinPT::loader::SDPT_Loader::forceLoad(std::string path, obj_loader* obj_ref, nanovdb_loader* nvdb_ref, envmap_loader* envmap_ref) {
	if (!load_sdpt(path, obj_ref, nvdb_ref, envmap_ref)) {
		std::cout << "Failed to load scene " << path << ".\n";
		obj_ref->clean();
		nvdb_ref->clean();
		envmap_ref->clean();

		std::cout << "Choose scene to load (with extension) : ";
		std::cin >> path;

		forceLoad(path, obj_ref, nvdb_ref, envmap_ref);
	}
}