#pragma once

namespace penguinPT::loader {
	class Mesh_USD {
	public:
		std::string name;
	};
	class Volume_USD {
	public:
		std::string name;
	};
	class Light_USD {
	public:
		std::string name;
	};
	class DomeLight_USD {
	public:
		std::string name;
	};

	class Xform {
	public :
		std::vector<Xform> childs;
		std::string name;

		// possibilities of data carried
		Mesh_USD mesh;
		Volume_USD volume;
		Light_USD light;
		DomeLight_USD dome_light;

		// every properties
		// ...
	};
	class usd_scene_loader {
	public:
		obj_loader typeObj;
		
		bool load_usd_scene(std::string path);

		usd_scene_loader() {}
		~usd_scene_loader() {}

		// utility functions
	private:
		void skip_line(unsigned int& i, std::vector<std::string>& tokens);

		// this version does not modify i, please call it on the index of the entrance bracket or after at least
		int get_next_bracket(unsigned int i, std::vector<std::string>& tokens, std::string entrance, std::string closure);

		void goto_next_symbol(unsigned int& i, std::vector<std::string>& tokens, std::string symbol);
	};
}
void penguinPT::loader::usd_scene_loader::goto_next_symbol(unsigned int& i, std::vector<std::string>& tokens, std::string symbol) {
	while (tokens.at(i) != symbol) i++;
}
int penguinPT::loader::usd_scene_loader::get_next_bracket(unsigned int i, std::vector<std::string>& tokens, std::string entrance, std::string closure) {
	unsigned int level = 0;

	if (tokens.at(i) == entrance) i++; // avoid doing shit while called exactly on the entrance (AS IT SHOULD BE)

	while (tokens.at(i) != closure || level != 0)
	{
		if (tokens.at(i) == entrance) level++;
		else if (tokens.at(i) == closure) level--;

		i++;
	}
	return i;
}
void penguinPT::loader::usd_scene_loader::skip_line(unsigned int& i, std::vector<std::string>& tokens) {
	if (tokens.at(i) == "ENDLINE") {
		i++;
		return;
	}
	while (tokens.at(i) != "ENDLINE") i++;
	i++;
	return;
}
bool penguinPT::loader::usd_scene_loader::load_usd_scene(std::string path) {
	// type of file : .usda

	std::ifstream file("assets/scenes/" + path);
	std::vector<std::string> tokens;

	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line))
		{
			std::vector<std::string> line_tokens = file_util::get_line_tokens(line, { ' ' });
			for (std::string l : line_tokens) tokens.push_back(l);
			tokens.push_back("ENDLINE");
			line_tokens.clear();
		}
		// def is made this way : def [type] [name] ( [args] ) { [body] }
		// types : custom, float3, double3, uniform

		unsigned int i = 0;
		goto_next_symbol(i, tokens, "{");
		
		std::cout << tokens.at(get_next_bracket(i, tokens, "{", "}"));
	}
	else {
		std::cout << "ERROR : FAILED TO LOAD assets/scenes/" << path << "\n";
		return false;
	}

	return true;
}