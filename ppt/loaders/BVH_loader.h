// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/util/Matrix.h>

#include <ppt/core/Mesh.h>

namespace penguinPT::loader {
	class BVHLoader {
	public:
		BVHLoader() {}
		~BVHLoader() {}

		// Loads a 3D model and stores the data in a BVH,
		// does nothing if the BVH already exists.
		void loadModel(std::string file_path, bool is_gpu_availble = false);

		// Loads a 3D model and stores the data in a BVH,
		// returns a pointer to the model's BVH.
		// Only creates a new BVH if the model hasn't been previouly loaded
		BVH* getModelBVH(std::string file_path, bool is_gpu_availble = false);

		void clearBVHLoader(bool is_gpu_availble = false);

		// Get a pointer to the BVH 
		BVH* getBVH(unsigned int index);
		BVH* getBVH(std::string BVH_name);

		bool doesBVHExists(std::string BVH_name);
	private:
		// Stores the data needed for BVH on the CPU or the GPU.
		//  -> CPU : element is created with malloc() and released with free()
		//  -> GPU : element is created with cudaMallocMananged() (accesible from CPU) and released with cudaFree()
		// NB : the GPU version will not work on devices without CUDA.
		std::vector<BVH*> BVH_data_list;

		// Each BVH is assigned a name so we can know if a BVH has already been loaded,
		// the i-th name corresponds to the i-th BVH.
		std::vector<std::string> BVH_name_list;

		// load an array of triangles and data associated from and .obj file,
		// does not load any material information associated with the .obj file.
		bool loadObj(std::string file_path, std::vector<Triangle>* dst, std::vector<Triangle_data>* dst_data);
	};
}

bool penguinPT::loader::BVHLoader::doesBVHExists(std::string BVH_name) {
	int i;
	return file_util::is_word_in_list(BVH_name, BVH_name_list, i);
}

penguinPT::BVH* penguinPT::loader::BVHLoader::getBVH(unsigned int index) {
	if (index < 0 || index >= BVH_data_list.size()) return nullptr;
	return BVH_data_list.at(index);
}
penguinPT::BVH* penguinPT::loader::BVHLoader::getBVH(std::string BVH_name) {
	int i;
	if (!file_util::is_word_in_list(BVH_name, BVH_name_list, i)) return nullptr;
	return getBVH(i);
}

void penguinPT::loader::BVHLoader::clearBVHLoader(bool is_gpu_availble) {
	for (BVH* ptr : BVH_data_list) {
		if (is_gpu_availble) cudaFree(ptr);
		else free(ptr);
	}

	BVH_data_list.clear();
	BVH_name_list.clear();
}

bool penguinPT::loader::BVHLoader::loadObj(std::string file_path, std::vector<Triangle>* dst, std::vector<Triangle_data>* dst_data) {
	bool normals = false;
	bool uvs = false;

	std::vector<nanovdb::Vec3f> vertex_list;
	std::vector<nanovdb::Vec3f> normal_list;
	std::vector<float2> uv_list;

	std::ifstream file("assets/models/" + file_path);

	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line)) {
			std::vector<std::string> tokens = file_util::get_line_tokens(line, { ' ', '/' });

			if (!tokens.empty()) {
				if (tokens.at(0) == "v") {
					vertex_list.push_back({ std::stof(tokens.at(1)), std::stof(tokens.at(2)), std::stof(tokens.at(3))});
				}
				else if (tokens.at(0) == "vn") {
					normal_list.push_back({ std::stof(tokens.at(1)), std::stof(tokens.at(2)), std::stof(tokens.at(3)) });
					normals = true;
				}
				else if (tokens.at(0) == "vt") {
					uv_list.push_back(make_float2(std::stof(tokens.at(1)), std::stof(tokens.at(2))));
					uvs = true;
				}
				else if (tokens.at(0) == "f") {
					
					int verticeA, verticeB, verticeC;
					int normalA, normalB, normalC;
					int uvA, uvB, uvC;

					if (normals && uvs) { 
						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(4)) - 1;
						verticeC = std::stoi(tokens.at(7)) - 1;

						normalA = std::stoi(tokens.at(3)) - 1;
						normalB = std::stoi(tokens.at(6)) - 1;
						normalC = std::stoi(tokens.at(9)) - 1;

						uvA = std::stoi(tokens.at(2)) - 1;
						uvB = std::stoi(tokens.at(5)) - 1;
						uvC = std::stoi(tokens.at(8)) - 1;

						Triangle tr(vertex_list.at(verticeA), vertex_list.at(verticeB), vertex_list.at(verticeC));
						Triangle_data tr_dat;
						tr_dat.nA = normal_list.at(normalA);
						tr_dat.nB = normal_list.at(normalB);
						tr_dat.nC = normal_list.at(normalC);

						tr_dat.uvA = uv_list.at(uvA);
						tr_dat.uvB = uv_list.at(uvB);
						tr_dat.uvC = uv_list.at(uvC);
#if MODE_TRIANGLE == 0
						tr.nA = normal_list.at(normalA);
						tr.nB = normal_list.at(normalB);
						tr.nC = normal_list.at(normalC);

						tr.BSDF_index = BASE_BSDF;
#endif
#if BSDF_DATA_HOLDER == 0
						tr_dat.BSDF_index = BASE_BSDF;
#endif

						dst->push_back(tr);
						dst_data->push_back(tr_dat);
					}
					else if (normals) { 
						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(3)) - 1;
						verticeC = std::stoi(tokens.at(5)) - 1;

						normalA = std::stoi(tokens.at(2)) - 1;
						normalB = std::stoi(tokens.at(4)) - 1;
						normalC = std::stoi(tokens.at(6)) - 1;

						Triangle tr(vertex_list.at(verticeA), vertex_list.at(verticeB), vertex_list.at(verticeC));
						Triangle_data tr_dat;
						tr_dat.nA = normal_list.at(normalA);
						tr_dat.nB = normal_list.at(normalB);
						tr_dat.nC = normal_list.at(normalC);
#if MODE_TRIANGLE == 0
						tr.nA = normal_list.at(normalA);
						tr.nB = normal_list.at(normalB);
						tr.nC = normal_list.at(normalC);

						tr.BSDF_index = BASE_BSDF;
#endif
#if BSDF_DATA_HOLDER == 0
						tr_dat.BSDF_index = BASE_BSDF;
#endif

						dst->push_back(tr);
						dst_data->push_back(tr_dat);
					}
					else if (uvs) { 
						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(3)) - 1;
						verticeC = std::stoi(tokens.at(5)) - 1;

						uvA = std::stoi(tokens.at(2)) - 1;
						uvB = std::stoi(tokens.at(4)) - 1;
						uvC = std::stoi(tokens.at(6)) - 1;

						Triangle tr(vertex_list.at(verticeA), vertex_list.at(verticeB), vertex_list.at(verticeC));
						Triangle_data tr_dat;

						tr_dat.uvA = uv_list.at(uvA);
						tr_dat.uvB = uv_list.at(uvB);
						tr_dat.uvC = uv_list.at(uvC);

#if BSDF_DATA_HOLDER == 0
						tr_dat.BSDF_index = BASE_BSDF;
#endif
#if MODE_TRIANGLE == 0
						tr.BSDF_index = BASE_BSDF;
#endif

						dst->push_back(tr);
						dst_data->push_back(tr_dat);
					}
					else { 
						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(2)) - 1;
						verticeC = std::stoi(tokens.at(3)) - 1;

						Triangle tr(vertex_list.at(verticeA), vertex_list.at(verticeB), vertex_list.at(verticeC));
						Triangle_data tr_dat;
#if BSDF_DATA_HOLDER == 0
						tr_dat.BSDF_index = BASE_BSDF;
#endif
#if MODE_TRIANGLE == 0
						tr.BSDF_index = BASE_BSDF;
#endif

						dst->push_back(tr);
						dst_data->push_back(tr_dat);
					}
				}
			}
		}

		file.close();

		vertex_list.clear();
		normal_list.clear();
		uv_list.clear();

		return true;
	}
	else {
		std::cout << "Unable to load object " << file_path << "\n";
		return false;
	}
	return true;
}

void penguinPT::loader::BVHLoader::loadModel(std::string file_path, bool is_gpu_availble) {
	if (doesBVHExists(file_path)) {
		std::cout << "BVH already exits.\n";
		return;
	}

	BVH_name_list.push_back(file_path);

	// create new BVH ptr
	BVH_data_list.push_back(nullptr);

	BVH*& temp_ptr = BVH_data_list.back();
	
	// allocate memory
	if (is_gpu_availble) {
		CUDA_CHECK(cudaMallocManaged((void**)&temp_ptr, sizeof(BVH)));
	}
	else {
		temp_ptr = (BVH*)malloc(sizeof(BVH));
	}

	temp_ptr->zeroValues();

	std::vector<Triangle> src;
	std::vector<Triangle_data> src_data;

	if (!loadObj(file_path, &src, &src_data)) {
		if (is_gpu_availble) cudaFree(temp_ptr);
		else free(temp_ptr);

		BVH_data_list.pop_back();
	}

	temp_ptr->fillBVH(src, src_data, is_gpu_availble);
	temp_ptr->buildBVH();

	src.clear();
	src_data.clear();
}

penguinPT::BVH* penguinPT::loader::BVHLoader::getModelBVH(std::string file_path, bool is_gpu_availble) {
	loadModel(file_path, is_gpu_availble);
	return getBVH(file_path);
}
