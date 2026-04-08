#pragma once

namespace penguinPT::loader {
	class Xform {
	public:
		Xform() {}
		~Xform() {}

		std::vector<std::string> parameters;

		std::vector<Xform> childs;
		std::string name;

		// other data

	};

	class usd_scene_loader {
	public:
		obj_loader typeObj;

		bool load_usd_scene(std::string path);

		usd_scene_loader() {}
		~usd_scene_loader() {}

	private:
		// actually load file
		bool load_tokens();

		std::vector<std::string> get_line(unsigned int& i);
		unsigned int get_closure(unsigned int i, std::string entrance, std::string closure);
		void goto_next(unsigned int& i, std::string s);

		// this function is clearly not cool
		// called on the Xform name : def Xform [name] <-- here
		Xform build_Xform_tree(unsigned int& i);

		std::vector<std::string> tokens;
		std::string scene_path;

		Xform default_prim;
		float meters_per_unit = 1.f;
		std::string up_axis = "Y";
	};


}

std::vector<std::string> penguinPT::loader::usd_scene_loader::get_line(unsigned int& i) {
	std::vector<std::string> temp;
	while (tokens.at(i) != "endline")
	{
		temp.push_back(tokens.at(i));
		i++;
	}
	// avoid ending on an endline
	i++;
	return temp;
}

unsigned int penguinPT::loader::usd_scene_loader::get_closure(unsigned int i, std::string entrance, std::string closure) {
	int level = 0;

	// check if called on the right character
	if (tokens.at(i) != entrance) {
		std::cout << "WARNING : 'get_closure' should be called on the entrance bracket.\n";
	}
	else i++;

	while (tokens.at(i) != closure || level != 0)
	{
		std::string this_token = tokens.at(i);
		if (this_token == entrance) level++;
		if (this_token == closure) level--;

		i++;
	}
	return i;
}
void penguinPT::loader::usd_scene_loader::goto_next(unsigned int& i, std::string s) {
	while (tokens.at(i) != s) i++;
}
bool penguinPT::loader::usd_scene_loader::load_tokens() {
	std::ifstream file("assets/scenes/" + scene_path);
	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line))
		{
			std::vector<std::string> line_tokens = file_util::get_line_tokens(line, { ' ' });
			if (line_tokens.size() > 0) {
				if (line_tokens.at(0)[0] != '#') { // ignore comments
					for (std::string& l : line_tokens) tokens.push_back(l);
					tokens.push_back("endline");
				}
				line_tokens.clear();
			}

		}
		return true;
	}
	return false;
}

penguinPT::loader::Xform penguinPT::loader::usd_scene_loader::build_Xform_tree(unsigned int& i) {
	Xform current;
	current.name = file_util::remove_quote(tokens.at(i++));
	std::cout << "New Xform called " << current.name << " initialized.\n";

	bool hasParameters = false;

	// we have 4 cases : 
	//  - Xform has parameters and parameters starts directly : "("
	//  - Xform has parameters and parameters starts on next line : "endline" -> "("
	//  - Xform has no parameters and body starts directly : "{"
	//  - Xform has no parameters and body starts on next line : "endline" -> "{"

	if (tokens.at(i) == "endline") i++;
	if (tokens.at(i) == "(") {
		std::cout << "Xform " << current.name << " has parameters.\n";
		hasParameters = true;
	}
	else if (tokens.at(i) == "{") std::cout << "Xform " << current.name << " has no parameters.\n";
	else {
		std::cout << "ERROR : unknown form of Xform.\n";
		return Xform();
	}

	// read Xform parameters
	if (hasParameters) {
		unsigned int parameters_closure_i = get_closure(i, "(", ")");

		// ignore now but will be done someday
		i = parameters_closure_i;

		goto_next(i, "{");
	}
	unsigned int data_closure_i = get_closure(i, "{", "}");

	// get into next line
	get_line(i);

	// loop through Xform data
	while (i < data_closure_i)
	{
		// multiple cases
		//  * definition
		//    - next Xform
		//    - Mesh, Volume, Camera...
		//  * random property
		if (tokens.at(i) == "def") {
			i++;

			if (tokens.at(i) == "Xform") {
				i++; // goto Xform name
				current.childs.push_back(build_Xform_tree(i));
			}
			else if (tokens.at(i) == "Volume") {
				std::string name = file_util::remove_quote(tokens.at(++i));
				std::cout << "New Volume : " << name << "\n";

				i++;

				// parameters check
				if (tokens.at(i) == "endline") i++;
				if (tokens.at(i) == "(") std::cout << "Volume " << name << " has parameters.\n";
				else if (tokens.at(i) == "{") std::cout << "Volume " << name << " has no parameters.\n";
				else { std::cout << "ERROR : unknown Volume form.\n"; return Xform(); }

				goto_next(i, "{");
				// loop here
				i = get_closure(i, "{", "}");

				get_line(i);
			}
			else if (tokens.at(i) == "Mesh") {
				std::string name = file_util::remove_quote(tokens.at(++i));
				std::cout << "New Mesh : " << name << "\n";

				i++;

				// parameters check
				if (tokens.at(i) == "endline") i++;
				if (tokens.at(i) == "(") std::cout << "Mesh " << name << " has parameters.\n";
				else if (tokens.at(i) == "{") std::cout << "Mesh " << name << " has no parameters.\n";
				else { std::cout << "ERROR : unknown Mesh form.\n"; return Xform(); }

				goto_next(i, "{");
				// loop here
				i = get_closure(i, "{", "}");

				get_line(i);
			}
			else if (tokens.at(i) == "Camera") {
				std::string name = file_util::remove_quote(tokens.at(++i));
				std::cout << "New Camera : " << name << "\n";

				i++;

				// parameters check
				if (tokens.at(i) == "endline") i++;
				if (tokens.at(i) == "(") std::cout << "Camera " << name << " has parameters.\n";
				else if (tokens.at(i) == "{") std::cout << "Camera " << name << " has no parameters.\n";
				else { std::cout << "ERROR : unknown Camera form.\n"; return Xform(); }

				goto_next(i, "{");
				// loop here
				i = get_closure(i, "{", "}");

				get_line(i);
			}
			else if (tokens.at(i) == "SphereLight") {
				std::string name = file_util::remove_quote(tokens.at(++i));
				std::cout << "New SphereLight : " << name << "\n";

				i++;

				// parameters check
				if (tokens.at(i) == "endline") i++;
				if (tokens.at(i) == "(") std::cout << "SphereLight " << name << " has parameters.\n";
				else if (tokens.at(i) == "{") std::cout << "SphereLight " << name << " has no parameters.\n";
				else { std::cout << "ERROR : unknown SphereLight form.\n"; return Xform(); }

				goto_next(i, "{");
				// loop here
				i = get_closure(i, "{", "}");

				get_line(i);
			}
			else if (tokens.at(i) == "DomeLight") {
				std::string name = file_util::remove_quote(tokens.at(++i));
				std::cout << "New DomeLight : " << name << "\n";

				i++;

				// parameters check
				if (tokens.at(i) == "endline") i++;
				if (tokens.at(i) == "(") std::cout << "DomeLight " << name << " has parameters.\n";
				else if (tokens.at(i) == "{") std::cout << "DomeLight " << name << " has no parameters.\n";
				else { std::cout << "ERROR : unknown DomeLight form.\n"; return Xform(); }

				goto_next(i, "{");
				// loop here
				i = get_closure(i, "{", "}");

				get_line(i);
			}
		}
		else {
			if (tokens.at(i) == "float3[]") {
				std::cout << "New float3[]\n";
			} 
			else if (tokens.at(i) == "int[]") {
				std::cout << "New int[]\n";
			}
			else if (tokens.at(i) == "normal3f[]") {
				std::cout << "New normal3f[]\n";
			}
			else if (tokens.at(i) == "point3f[]") {
				std::cout << "New point3f[]\n";
			}
			else if (tokens.at(i) == "bool[]") {
				std::cout << "New bool[]\n";
			}
			else if (tokens.at(i) == "texCoord2f[]") {
				std::cout << "New texCoord2f[]\n";
			}
			else if (tokens.at(i) == "float3") {
				std::cout << "New float3\n";
			}
			else if (tokens.at(i) == "float") {
				std::cout << "New float\n";
			}
			else if (tokens.at(i) == "int") {
				std::cout << "New int\n";
			}
			else if (tokens.at(i) == "bool") {
				std::cout << "New bool\n";
			}
			else if (tokens.at(i) == "custom") {
				std::cout << "New custom\n";
			}
			get_line(i);
		}
	}

	if (tokens.at(i + 1) == "endline") {
		std::cout << "Fine, end before an 'endline'.\n";
		get_line(i);
	}

	return current;
}

bool penguinPT::loader::usd_scene_loader::load_usd_scene(std::string path) {
	scene_path = path;
	if (!load_tokens()) return false;

	// technically, '(' should be the first token
	if (tokens.at(0) != "(") { std::cout << "ERROR : USD is not as expected.\n"; return false; }

	// read global file params
	// file start is "(", "endline"
	unsigned int i = 0;
	std::string defaultPrim_name;
	while (i < get_closure(0, "(", ")")) {
		std::vector<std::string> line = get_line(i);
		
		// choose which parameter change
		if (line.at(0) == "defaultPrim") defaultPrim_name = file_util::remove_quote(line.back());
		else if (line.at(0) == "metersPerUnit") meters_per_unit = std::stof(line.back());
		else if (line.at(0) == "upAxis") up_axis = file_util::remove_quote(line.back());
	}
	std::cout << "Default Prim name : " << defaultPrim_name << ", Meters per unit : " << meters_per_unit << ", Up axis : " << up_axis << "\n";
	
	// end on ")", go to next line
	get_line(i);

	// check if first prim is matching
	if (tokens.at(i++) == "def" && tokens.at(i++) == "Xform") {
		std::cout << "First prim found.\n";
		if (file_util::remove_quote(tokens.at(i)) == defaultPrim_name) {
			// build tree
			default_prim = build_Xform_tree(i);
		}
		else {
			std::cout << "First Prim name (" << tokens.at(i) << ") found is not matching the one mentionned earlier : " << defaultPrim_name << ".\n";
			return false;
		}
	}
}