#pragma once

namespace penguinPT::loader {
	class obj_loader {
	public:
		obj_loader() {}
		~obj_loader() {}

		std::vector<Triangle> triangle_list;
		std::vector<Triangle_data> tr_data_list;
		std::vector<nanovdb::Vec3f> vertices_list;
		std::vector<nanovdb::Vec3f> normal_list;
		std::vector<float2> uv_list;

		unsigned int triangles_num = 0;

		bool load_obj(std::string path, nanovdb::Vec3f position, float scale, unsigned int BSDF_id = BASE_BSDF);
		void send_to_scene(Scene_data& scene);
	};

	bool obj_loader::load_obj(std::string path, nanovdb::Vec3f position, float scale, unsigned int BSDF_id) {

		bool normals = false;
		bool uvs = false;

		unsigned int vertex_start = vertices_list.size();
		unsigned int normal_start = normal_list.size();
		unsigned int uv_start = uv_list.size();

		std::ifstream file("assets/models/" + path);

		if (file.is_open()) {
			std::string line;
			while (std::getline(file, line)) {
				std::vector<std::string> tokens = file_util::get_line_tokens(line, {' ', '/'});
				
				if (tokens.at(0) == "v") {
					//std::cout << "Vertice\n";
					vertices_list.push_back({ std::stof(tokens.at(1)) * scale + position[0], std::stof(tokens.at(2)) * scale + position[1], std::stof(tokens.at(3)) * scale + position[2] });
					//std::cout << std::stof(tokens.at(1)) << " " << std::stof(tokens.at(2)) << " " << std::stof(tokens.at(3)) << "\n";
				} else if (tokens.at(0) == "vn") {
					//std::cout << "Normal\n";
					normal_list.push_back({ std::stof(tokens.at(1)), std::stof(tokens.at(2)), std::stof(tokens.at(3)) });
					normals = true;
				}
				else if (tokens.at(0) == "vt") {
					//std::cout << "UV\n";
					uv_list.push_back(make_float2(std::stof(tokens.at(1)), std::stof(tokens.at(2))));
					uvs = true;
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
			
			file.close();
			return true;
		}
		else {
			std::cout << "Unable to load object " << path << "\n";
			return false;
		}
		return true;
	}

	void obj_loader::send_to_scene(Scene_data& scene) {
		std::cout << triangles_num << " " << triangle_list.size() << "\n";
		scene = Scene_data(triangle_list.size());
		for (int i = 0; i < triangle_list.size(); i++) {
			scene.triangles[i] = triangle_list.at(i);
			scene.tr_data[i] = tr_data_list.at(i);
			scene.triangle_indicies[i] = i;
		}

		triangle_list.clear();
		tr_data_list.clear();
		vertices_list.clear();
		normal_list.clear();
		uv_list.clear();
	}

	class BSDF_loader {
	public:
		
		BSDF_loader() {
			principled_BSDF zero; zero.is_through = true; BSDF_list.push_back(zero); BSDF_name.push_back("zero");

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
		void send_to_gpu(Scene_data& scene, bool use_gpu);
		void send_to_scene(Scene_data& scene);

		int gifn(std::string name);
	};

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
	void BSDF_loader::send_to_scene(Scene_data& scene) {
		scene.bsdf_list = (principled_BSDF*)malloc(BSDF_list.size() * sizeof(principled_BSDF));
		scene.num_of_bsdf = BSDF_list.size();
		std::cout << "BSDF list : \n";
		for (int i = 0; i < BSDF_list.size(); i++) {
			scene.bsdf_list[i] = BSDF_list.at(i);
			std::cout << " - " << BSDF_name.at(i) << "\n";
		}
	}
	void BSDF_loader::send_to_gpu(Scene_data& scene, bool use_gpu) {
		principled_BSDF* host_list = new principled_BSDF[BSDF_list.size()];
		for (int i = 0; i < BSDF_list.size(); i++) {
			host_list[i] = BSDF_list.at(i);
		}
		
		if (use_gpu) {
			// transfer
			cudaError_t err = cudaMalloc((void**)&scene.bsdf_list, BSDF_list.size() * sizeof(principled_BSDF));
			if (err != CUDA_SUCCESS) {
				std::cout << "Failed CUDA memory allocation for BSDF.\n";
			}
			err = cudaMemcpy(scene.bsdf_list, host_list, BSDF_list.size() * sizeof(principled_BSDF), cudaMemcpyHostToDevice);
			if (err != CUDA_SUCCESS) {
				std::cout << "Failed CUDA memory transfert for BSDF.\n";
			}

			delete[] host_list;
		}
		else {
			scene.bsdf_list = host_list;
		}
	}
	int BSDF_loader::gifn(std::string name) {
		for (int i = 0; i < BSDF_list.size(); i++) {
			if (name == BSDF_name.at(i)) return i;
		}
		return BASE_BSDF;
	}
}