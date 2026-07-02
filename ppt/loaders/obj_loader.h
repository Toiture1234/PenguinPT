// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/core/Material.h>
#include <ppt/core/intersectors.h>
#include <ppt/core/texture_manager.h>
#include <ppt/core/Mesh.h>

namespace penguinPT::loader {
	class BSDF_loader {
	public:

		BSDF_loader() {
			principled_BSDF zero; zero.alpha = 0.f; BSDF_list.push_back(zero); BSDF_name.push_back("zero");

			principled_BSDF base_BSDF; BSDF_list.push_back(base_BSDF); BSDF_name.push_back("base_BSDF");
		}
		~BSDF_loader() {}

		std::vector<principled_BSDF> BSDF_list;
		std::vector<std::string> BSDF_name;

		void create_new_BSDF(std::string name,
			float roughness,
			nanovdb::Vec3f albedo,
			nanovdb::Vec3f emission,
			nanovdb::Vec3f absorption,
			float metalness,
			float ior,
			float transparency);
		void create_new_BSDF(std::string name, nanovdb::Vec3f albedo, float roughness);
		
		int gifn(std::string name);

		void clean();
	};

	class obj_loader {
	public:
		obj_loader() : bsdf_loader() {}
		~obj_loader() {}

		std::vector<Triangle> triangle_list;
		std::vector<Triangle_data> tr_data_list;
		std::vector<nanovdb::Vec3f> vertices_list;
		std::vector<nanovdb::Vec3f> normal_list;
		std::vector<float2> uv_list;

		BSDF_loader bsdf_loader;

		unsigned int triangles_num = 0;

		bool load_obj(std::string path, nanovdb::Vec3f position, float scale, bool useGPU = true, unsigned int BSDF_id = BASE_BSDF);
		bool load_mtl(std::string path, bool useGPU = true);
		
		void set_texture_manager(texture_manager* src) { texture_m_ptr = src; }

		BVH* getBVH(bool is_gpu_available);

		void clean();

	private:
		texture_manager* texture_m_ptr;
	};

	bool obj_loader::load_mtl(std::string path, bool useGPU) {
#ifndef WINDOWS_VERSION
		std::ifstream file("assets/models/" + path);
#else
		std::ifstream file(path);
#endif

		if (file.is_open()) {
			std::string line;

			bool apply_definition = false;
			while (std::getline(file, line)) {
				std::vector<std::string> tokens = file_util::get_line_tokens(line, { ' ' });
				
				if (!tokens.empty()) {
					if (tokens.at(0) == "newmtl") {
						int i;
						if (!file_util::is_word_in_list(tokens.at(1), bsdf_loader.BSDF_name, i)) {
							bsdf_loader.create_new_BSDF(tokens.at(1), nanovdb::Vec3f(0.f), 0.f);
							apply_definition = true;
						}
						else {
							apply_definition = false;
						}
					}
					else if (tokens.at(0) == "Kd") {
						if (apply_definition) {
							nanovdb::Vec3f& a = bsdf_loader.BSDF_list.back().albedo;
							sscanf((tokens.at(1) + " " + tokens.at(2) + " " + tokens.at(3)).c_str(), "%f %f %f", &a[0], &a[1], &a[2]);
						}
					}
					else if (tokens.at(0) == "Ke") {
						if (apply_definition) {
							nanovdb::Vec3f& a = bsdf_loader.BSDF_list.back().emission;
							sscanf((tokens.at(1) + " " + tokens.at(2) + " " + tokens.at(3)).c_str(), "%f %f %f", &a[0], &a[1], &a[2]);
						}
					}
					else if (tokens.at(0) == "Ni") {
						if (apply_definition) {
							float& a = bsdf_loader.BSDF_list.back().ior;
							sscanf(tokens.at(1).c_str(), "%f", &a);
						}
					}
					else if (tokens.at(0) == "Pr") {
						if (apply_definition) {
							float& a = bsdf_loader.BSDF_list.back().roughness;
							sscanf(tokens.at(1).c_str(), "%f", &a);
						}
					}
					else if (tokens.at(0) == "Pm") {
						if (apply_definition) {
							float& a = bsdf_loader.BSDF_list.back().metalness;
							sscanf(tokens.at(1).c_str(), "%f", &a);
						}
					}
					else if (tokens.at(0) == "d") {
						if (apply_definition) {
							float& a = bsdf_loader.BSDF_list.back().transparency;
							sscanf(tokens.at(1).c_str(), "%f", &a);
							a = 1.f - a;
						}
					}
					else if (tokens.at(0) == "map_Kd") {
						if (apply_definition) {
							
							if (useGPU) {
								cudaTextureObject_t& a = bsdf_loader.BSDF_list.back().albedo_tex_CUDA;

#ifndef WINDOWS_VERSION
								if(texture_m_ptr->load_texture_GPU_float4("assets/models/" + tokens.at(1), a)) 
#else
								if (texture_m_ptr->load_texture_GPU_float4(file_util::removeFileName(path) + tokens.at(1), a))
#endif
									bsdf_loader.BSDF_list.back().use_albedo_tex = true;
							}
							else {
								CPU_float4_texture& a = bsdf_loader.BSDF_list.back().albedo_tex_HOST;

#ifndef WINDOWS_VERSION
								if(texture_m_ptr->load_texture_CPU_float4("assets/models/" + tokens.at(1), a)) 
#else
								if (texture_m_ptr->load_texture_CPU_float4(file_util::removeFileName(path) + tokens.at(1), a))
#endif
									bsdf_loader.BSDF_list.back().use_albedo_tex = true;
							}
						}
					}
					else if (tokens.at(0) == "map_Pr") {
						if (apply_definition) {
							if (useGPU) {
								cudaTextureObject_t& a = bsdf_loader.BSDF_list.back().roughness_tex_CUDA;

#ifndef WINDOWS_VERSION
								if(texture_m_ptr->load_texture_GPU_float("assets/models/" + tokens.at(1), a))
#else 
								if (texture_m_ptr->load_texture_GPU_float(file_util::removeFileName(path) + tokens.at(1), a))
#endif
									bsdf_loader.BSDF_list.back().use_roughness_tex = true;
							}
							else {
								CPU_float_texture& a = bsdf_loader.BSDF_list.back().roughness_tex_HOST;

#ifndef WINDOWS_VERSION
								if(texture_m_ptr->load_texture_CPU_float("assets/models/" + tokens.at(1), a))
#else
								if (texture_m_ptr->load_texture_CPU_float(file_util::removeFileName(path) + tokens.at(1), a))
#endif
									bsdf_loader.BSDF_list.back().use_roughness_tex = true;
							}
						}
					}
					else if (tokens.at(0) == "map_Bump") {
						if (apply_definition) {
							if (useGPU) {
								cudaTextureObject_t& a = bsdf_loader.BSDF_list.back().normal_tex_CUDA;

#ifndef WINDOWS_VERSION
								if(texture_m_ptr->load_texture_GPU_float4("assets/models/" + tokens.at(3), a))
#else
								if (texture_m_ptr->load_texture_GPU_float4(file_util::removeFileName(path) + tokens.at(3), a))
#endif
									bsdf_loader.BSDF_list.back().use_normal_tex = true;
							}
							else {
								CPU_float4_texture& a = bsdf_loader.BSDF_list.back().normal_tex_HOST;

#ifndef WINDOWS_VERSION
								if(texture_m_ptr->load_texture_CPU_float4("assets/models/" + tokens.at(3), a))
#else
								if (texture_m_ptr->load_texture_CPU_float4(file_util::removeFileName(path) + tokens.at(3), a))
#endif
									bsdf_loader.BSDF_list.back().use_normal_tex = true;
							}
						}
					}
				}
				
			}
		}
		else return false;

		return true;
	}

	bool obj_loader::load_obj(std::string path, nanovdb::Vec3f position, float scale, bool useGPU, unsigned int BSDF_id) {

		bool normals = false;
		bool uvs = false;

		unsigned int vertex_start = vertices_list.size();
		unsigned int normal_start = normal_list.size();
		unsigned int uv_start = uv_list.size();

		bool mtl_file = false;
#ifndef WINDOWS_VERSION
		std::ifstream file("assets/models/" + path);
#else
		std::ifstream file(path);
#endif

		if (file.is_open()) {
			std::string line;
			while (std::getline(file, line)) {
				std::vector<std::string> tokens = file_util::get_line_tokens(line, {' ', '/'});
				
				if (!tokens.empty()) {
					if (tokens.at(0) == "v") {
						//std::cout << "Vertice\n";
						vertices_list.push_back({ std::stof(tokens.at(1)) * scale + position[0], std::stof(tokens.at(2)) * scale + position[1], std::stof(tokens.at(3)) * scale + position[2] });
						//std::cout << std::stof(tokens.at(1)) << " " << std::stof(tokens.at(2)) << " " << std::stof(tokens.at(3)) << "\n";
					}
					else if (tokens.at(0) == "vn") {
						//std::cout << "Normal\n";
						normal_list.push_back({ std::stof(tokens.at(1)), std::stof(tokens.at(2)), std::stof(tokens.at(3)) });
						normals = true;
					}
					else if (tokens.at(0) == "vt") {
						//std::cout << "UV\n";
						uv_list.push_back(make_float2(std::stof(tokens.at(1)), std::stof(tokens.at(2))));
						uvs = true;
					}
					else if (tokens.at(0) == "mtllib") {
#ifndef WINDOWS_VERSION
						mtl_file = load_mtl(tokens.at(1), useGPU);
#else
						mtl_file = load_mtl(file_util::removeFileName(path) + tokens.at(1), useGPU);
#endif
					}
					else if (tokens.at(0) == "usemtl" && mtl_file) {
						// choose material
						for (int i = 0; i < bsdf_loader.BSDF_list.size(); i++) {
							if (tokens.at(1) == bsdf_loader.BSDF_name.at(i)) BSDF_id = i;
						}
					}
					else if (tokens.at(0) == "f") {
						//std::cout << "Triangle";

						/*for (auto& str : tokens) std::cout << str << " ";
						std::cout << "\n";
						std::cout << tokens.size() << "\n";*/
						int verticeA, verticeB, verticeC;
						int normalA, normalB, normalC;
						int uvA, uvB, uvC;

						if (normals && uvs) { // everything is here !
							//std::cout << "everything is here !\n";

							verticeA = std::stoi(tokens.at(1)) - 1 + vertex_start;
							verticeB = std::stoi(tokens.at(4)) - 1 + vertex_start;
							verticeC = std::stoi(tokens.at(7)) - 1 + vertex_start;

							normalA = std::stoi(tokens.at(3)) - 1 + normal_start;
							normalB = std::stoi(tokens.at(6)) - 1 + normal_start;
							normalC = std::stoi(tokens.at(9)) - 1 + normal_start;

							uvA = std::stoi(tokens.at(2)) - 1 + uv_start;
							uvB = std::stoi(tokens.at(5)) - 1 + uv_start;
							uvC = std::stoi(tokens.at(8)) - 1 + uv_start;

							Triangle tr(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC));
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

							tr.BSDF_index = BSDF_id;
#endif
							tr_dat.BSDF_index = BSDF_id;

							triangle_list.push_back(tr);
							tr_data_list.push_back(tr_dat);
						}
						else if (normals) { // normals but not uvs
							//std::cout << "normals but not uvs\n";

							verticeA = std::stoi(tokens.at(1)) - 1 + vertex_start;
							verticeB = std::stoi(tokens.at(3)) - 1 + vertex_start;
							verticeC = std::stoi(tokens.at(5)) - 1 + vertex_start;

							normalA = std::stoi(tokens.at(2)) - 1 + normal_start;
							normalB = std::stoi(tokens.at(4)) - 1 + normal_start;
							normalC = std::stoi(tokens.at(6)) - 1 + normal_start;

							Triangle tr(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC));
							Triangle_data tr_dat;
							tr_dat.nA = normal_list.at(normalA);
							tr_dat.nB = normal_list.at(normalB);
							tr_dat.nC = normal_list.at(normalC);
#if MODE_TRIANGLE == 0
							tr.nA = normal_list.at(normalA);
							tr.nB = normal_list.at(normalB);
							tr.nC = normal_list.at(normalC);

							tr.BSDF_index = BSDF_id;
#endif
							tr_dat.BSDF_index = BSDF_id;

							triangle_list.push_back(tr);
							tr_data_list.push_back(tr_dat);
						}
						else if (uvs) { // uvs but not normals
							//std::cout << "uvs but not normals\n";

							verticeA = std::stoi(tokens.at(1)) - 1 + vertex_start;
							verticeB = std::stoi(tokens.at(3)) - 1 + vertex_start;
							verticeC = std::stoi(tokens.at(5)) - 1 + vertex_start;

							uvA = std::stoi(tokens.at(2)) - 1 + uv_start;
							uvB = std::stoi(tokens.at(4)) - 1 + uv_start;
							uvC = std::stoi(tokens.at(6)) - 1 + uv_start;

							Triangle tr(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC));
							Triangle_data tr_dat;

							tr_dat.uvA = uv_list.at(uvA);
							tr_dat.uvB = uv_list.at(uvB);
							tr_dat.uvC = uv_list.at(uvC);

							tr_dat.BSDF_index = BSDF_id;
#if MODE_TRIANGLE == 0
							tr.BSDF_index = BSDF_id;
#endif

							triangle_list.push_back(tr);
							tr_data_list.push_back(tr_dat);
						}
						else { // only points, do nothing special
							//std::cout << "only points\n";

							verticeA = std::stoi(tokens.at(1)) - 1 + vertex_start;
							verticeB = std::stoi(tokens.at(2)) - 1 + vertex_start;
							verticeC = std::stoi(tokens.at(3)) - 1 + vertex_start;

							Triangle tr(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC));
							Triangle_data tr_dat;

							tr_dat.BSDF_index = BSDF_id;
#if MODE_TRIANGLE == 0
							tr.BSDF_index = BSDF_id;
#endif

							triangle_list.push_back(tr);
							tr_data_list.push_back(tr_dat);
						}

						triangles_num++;
					}
				}
			}
			
			file.close();
			return true;
		}
		else {
			std::cout << "Unable to load object " << path << "\n";
			return false;
		}
		return true;
	}

	BVH* obj_loader::getBVH(bool is_gpu_available) {
		std::cout << "Triangle data :\n";
		std::cout << " - Number of triangles : " << triangles_num << "\n";
		BVH* ptr;
		
		if (is_gpu_available) cudaMallocManaged((void**)&ptr, sizeof(BVH));
		else ptr = (BVH*)malloc(sizeof(BVH));

		ptr->zeroValues();
		ptr->fillBVH(triangle_list, tr_data_list, is_gpu_available);
		ptr->buildBVH();
		return ptr;
		//bsdf_loader.send_to_scene(scene, is_gpu_available);

	}
	void obj_loader::clean() {
		bsdf_loader.clean();

		triangle_list.clear();
		tr_data_list.clear();
		vertices_list.clear();
		normal_list.clear();
		uv_list.clear();
		triangles_num = 0;
	}

	

	void BSDF_loader::create_new_BSDF(std::string name, 
		float roughness,
		nanovdb::Vec3f albedo,
		nanovdb::Vec3f emission,
		nanovdb::Vec3f absorption,
		float metalness,
		float ior,
		float transparency)
	{
		principled_BSDF current;
		current.roughness = roughness;
		current.albedo = albedo;
		current.emission = emission;
		current.absorption = absorption;
		current.metalness = metalness;
		current.ior = ior;
		current.transparency = transparency;

		BSDF_list.push_back(current);
		BSDF_name.push_back(name);
	}
	void BSDF_loader::create_new_BSDF(std::string name, 
		nanovdb::Vec3f albedo, float roughness) {
		principled_BSDF current;
		current.albedo = albedo;
		current.roughness = roughness;

		BSDF_list.push_back(current);
		BSDF_name.push_back(name);
	}
	void BSDF_loader::clean() {
		BSDF_list.clear();
		BSDF_name.clear();
		principled_BSDF zero; zero.alpha = 0.f; BSDF_list.push_back(zero); BSDF_name.push_back("zero");
		principled_BSDF base_BSDF; BSDF_list.push_back(base_BSDF); BSDF_name.push_back("base_BSDF");
	}

	int BSDF_loader::gifn(std::string name) {
		for (int i = 0; i < BSDF_list.size(); i++) {
			if (name == BSDF_name.at(i)) return i;
		}
		return BASE_BSDF;
	}
}