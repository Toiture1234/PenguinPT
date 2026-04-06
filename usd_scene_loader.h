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
		// goes on next line (may be issues with last line idk)
		void skip_line(unsigned int& i, std::vector<std::string>& tokens);

		// delete useless ENDLINE to make a more clear definition
		void preprocess_endline(std::vector<std::string>& tokens);

		// this version does not modify i, please call it on the index of the entrance bracket or after at least
		int get_next_bracket(unsigned int i, std::vector<std::string>& tokens, std::string entrance, std::string closure);

		void goto_next_symbol(unsigned int& i, std::vector<std::string>& tokens, std::string symbol);

		// returns if the declaration has some parameter-like properties
		// returns the index of the end bracket
		bool takes_params(unsigned int i, std::vector<std::string>& tokens, unsigned int& i_out);

		// CALLED ON A def <-- THIS IS VERY IMPORTANT !!!!
		// Needs refinement especially on the Xform part !
		void track_file_tree(unsigned int& i, std::vector<std::string>& tokens, Xform& root);
		void print_tree(Xform root);
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
void penguinPT::loader::usd_scene_loader::preprocess_endline(std::vector<std::string>& tokens) {
	unsigned int i = 0;
	while (i < tokens.size() - 1)
	{
		if (tokens.at(i) == "ENDLINE" && tokens.at(i + 1) == "{") tokens.erase(tokens.begin() + i);
		i++;
	}
}
void penguinPT::loader::usd_scene_loader::skip_line(unsigned int& i, std::vector<std::string>& tokens) {
	// goto next token if we're currently on and ENDLINE
	if (tokens.at(i) == "ENDLINE") {
		i++;
		return;
	}
	// loop until we find and ENDLINE
	while (tokens.at(i) != "ENDLINE") i++;
	i++;
	return;
}
void penguinPT::loader::usd_scene_loader::track_file_tree(unsigned int& i, std::vector<std::string>& tokens, Xform& root) {
	i++; 

	// get data type
	if (tokens.at(i) == "Xform") { // recursive shit
		i++;
		Xform child;
		child.name = tokens.at(i);

		std::cout << root.name << " has a new child : " << child.name << "\n";

		if (tokens.at(i + 1) == "(") { // arguments are so fucking haaaaard 
			i += 1;
			i = get_next_bracket(i, tokens, "(", ")");
		}
		i++;

		if (tokens.at(i) == "{") {
			int i_max = get_next_bracket(i, tokens, "{", "}");
			
			while (i < i_max) {
				if (tokens.at(i) == "def") {
					track_file_tree(i, tokens, child);
				}
				else if (tokens.at(i)[0] == '#') skip_line(i, tokens);
				// we do not count additionnal params now
				i++;
			}
		}
		else {
			std::cout << "BRO IDK WTF IS HAPPENING :(\n";
			return;
		}

		root.childs.push_back(child);
	}
	else if (tokens.at(i) == "Mesh") { // almost ez
		i++;
		std::cout << "New mesh : " << tokens.at(i) << "\n";

		// avoid params now
		i++;
		goto_next_symbol(i, tokens, "{");
		i = get_next_bracket(i, tokens, "{", "}");
	}
	else if (tokens.at(i) == "Volume") { // almost ez
		i++;
		std::cout << "New Volume : " << tokens.at(i) << "\n";

		// avoid params now
		i++;
		goto_next_symbol(i, tokens, "{");
		i = get_next_bracket(i, tokens, "{", "}");
	}
	else if (tokens.at(i) == "SphereLight") { // almost ez
		i++;
		std::cout << "New SphereLight : " << tokens.at(i) << "\n";

		// avoid params now
		i++;
		goto_next_symbol(i, tokens, "{");
		i = get_next_bracket(i, tokens, "{", "}");
	}
	else if (tokens.at(i) == "DomeLight") { // almost ez
		i++;
		std::cout << "New DomeLight : " << tokens.at(i) << "\n";

		// avoid params now
		i++;
		goto_next_symbol(i, tokens, "{");
		i = get_next_bracket(i, tokens, "{", "}");
	}
	else if (tokens.at(i) == "Camera") { // not implemented yet 
		i++;
		std::cout << "New Camera : " << tokens.at(i) << "\n";

		// avoid params now
		i++;
		goto_next_symbol(i, tokens, "{");
		i = get_next_bracket(i, tokens, "{", "}");
	}
}
void penguinPT::loader::usd_scene_loader::print_tree(Xform root) {
	std::cout << "Xform : " << root.name << "\n";
	for (Xform& child : root.childs) print_tree(child);
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
		// preprocess to delete useless ENDLINE
		preprocess_endline(tokens);
		//for (std::string& l : tokens) if (l == "ENDLINE") std::cout << "\n"; else std::cout << l << " ";

		// def is made this way : def [type] [name] ( [args] ) { [body] }
		// types : custom, float3, double3, uniform

		unsigned int i = 0;
		Xform file_root; // aka root0 : the root of the root
		goto_next_symbol(i, tokens, "def");

		track_file_tree(i, tokens, file_root);
		print_tree(file_root);
	}
	else {
		std::cout << "ERROR : FAILED TO LOAD assets/scenes/" << path << "\n";
		return false;
	}

	return true;
}