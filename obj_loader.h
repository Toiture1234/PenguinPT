#pragma once

namespace penguinPT::loader {
	// utility for reading lines
	// assume the current character is not an escape
	std::string read_to_escape(std::string line, unsigned int& i) {
		std::string result = "";

		for (i; i < line.length(); i++) {
			char c = line[i];
			if (c == ' ' || c == '\n' || c == '/') return result;
			else result += c;
		}
		return result;
	}
	std::vector<std::string> get_line_tokens(std::string line) {
		unsigned int i = 0;
		std::vector<std::string> tokens;

		while (i < line.length()) {
			char c = line[i];

			if (c == ' ' || c == '/') i++;
			else if (c == '\n') break;
			else {
				tokens.push_back(read_to_escape(line, i));
			}
		}
		return tokens;
	}

	class obj_loader {
	public:
		obj_loader() {}
		~obj_loader() {}

		std::vector<Triangle> triangle_list;
		std::vector<nanovdb::Vec3f> vertices_list;
		std::vector<nanovdb::Vec3f> normal_list;
		std::vector<float2> uv_list;

		unsigned int triangles_num = 0;

		bool load_obj(std::string path);
		void send_to_scene(Scene_data& scene);
	};

	bool obj_loader::load_obj(std::string path) {

		bool normals = false;
		bool uvs = false;

		std::ifstream file(path);

		if (file.is_open()) {
			std::string line;
			while (std::getline(file, line)) {
				std::vector<std::string> tokens = get_line_tokens(line);
				
				if (tokens.at(0) == "v") {
					//std::cout << "Vertice\n";
					vertices_list.push_back({ std::stof(tokens.at(1)), std::stof(tokens.at(2)), std::stof(tokens.at(3)) });
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

						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(4)) - 1;
						verticeC = std::stoi(tokens.at(7)) - 1;

						normalA = std::stoi(tokens.at(3)) - 1;
						normalB = std::stoi(tokens.at(6)) - 1;
						normalC = std::stoi(tokens.at(9)) - 1;

						uvA = std::stoi(tokens.at(2)) - 1;
						uvB = std::stoi(tokens.at(5)) - 1;
						uvC = std::stoi(tokens.at(8)) - 1;

						Triangle tr(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC));
						tr.nA = normal_list.at(normalA);
						tr.nB = normal_list.at(normalB);
						tr.nC = normal_list.at(normalC);

						triangle_list.push_back(tr);
					}
					else if (normals) { // normals but not uvs
						//std::cout << "normals but not uvs\n";

						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(3)) - 1;
						verticeC = std::stoi(tokens.at(5)) - 1;

						normalA = std::stoi(tokens.at(2)) - 1;
						normalB = std::stoi(tokens.at(4)) - 1;
						normalC = std::stoi(tokens.at(6)) - 1;

						Triangle tr(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC));
						tr.nA = normal_list.at(normalA);
						tr.nB = normal_list.at(normalB);
						tr.nC = normal_list.at(normalC);

						triangle_list.push_back(tr);
					}
					else if (uvs) { // uvs but not normals
						//std::cout << "uvs but not normals\n";

						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(3)) - 1;
						verticeC = std::stoi(tokens.at(5)) - 1;

						uvA = std::stoi(tokens.at(2)) - 1;
						uvB = std::stoi(tokens.at(4)) - 1;
						uvC = std::stoi(tokens.at(6)) - 1;

						triangle_list.push_back(Triangle(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC)));
					}
					else { // only points, do nothing special
						//std::cout << "only points\n";

						verticeA = std::stoi(tokens.at(1)) - 1;
						verticeB = std::stoi(tokens.at(2)) - 1;
						verticeC = std::stoi(tokens.at(3)) - 1;

						triangle_list.push_back(Triangle(vertices_list.at(verticeA), vertices_list.at(verticeB), vertices_list.at(verticeC)));
					}
					
					triangles_num++;
				}
			}
			
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
			scene.triangle_indicies[i] = i;
		}

		triangle_list.clear();
		vertices_list.clear();
		normal_list.clear();
		uv_list.clear();
	}
}