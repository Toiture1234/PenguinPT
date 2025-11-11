#pragma once

namespace penguinPT::loader {
	class usd_scene_loader {
	public:
		obj_loader typeObj;
		std::vector<Sphere> primitiveSphere;

		bool load_usd_scene(std::string path);

		usd_scene_loader() {}
		~usd_scene_loader() {}
	};
}
// utility for reading lines
// assume the current character is not an escape

bool penguinPT::loader::usd_scene_loader::load_usd_scene(std::string path) {
	// type of file : .usda

	std::ifstream file("assets/scenes/" + path);
	std::vector<std::string> tokens;

	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line))
		{
			std::vector<std::string> line_tokens = file_util::get_line_tokens(line, {' ', ','});
			for (std::string& l : line_tokens) tokens.push_back(l);
			tokens.push_back("endl");
		}

		for (std::string& l : tokens) {
			std::cout << l << "\n";
			printf("HHE");
		}
	}
	else {
		std::cout << "ERROR : FAILED TO LOAD assets/scenes/" << path << "\n";
		return false;
	}

	return true;
}