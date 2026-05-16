// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/loaders/obj_loader.h>

#define SYMBOL_CHECK(x, y, c) if(x != y) { c }
#define INVALID_USD std::cout << "ERROR -- Invalid USD file.\n";

#define USD_TRACK_PRINT 0

namespace penguinPT::loader {
	struct Property {
		std::string type;
		std::string name;
		std::string value;

		std::vector<Property> sub_property_list;
	};
	// This class isn't really and Xform as intented in the USD file format 
	// but more a node in the tree representation of the scene.
	class Xform {
	public:
		Xform() {}
		~Xform() {}

		std::string name;
		std::string type;

		std::vector<Property> property_list;
		std::vector<Xform> childs;
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

		Property parseFloat(unsigned int i);
		Property parseFloatList(unsigned int i);

		Property parseFloat3(unsigned int i);
		Property parseFloat3List(unsigned int i);
		
		Property parseInt(unsigned int i);
		Property parseIntList(unsigned int i);
		Property parseInt3List(unsigned int i);
		Property parseBool(unsigned int i);
		Property parseBoolList(unsigned int i, int& line_count);
		Property parseNormal3fList(unsigned int i, int& line_count);
		Property parseTexcoord2fList(unsigned int i, int& line_count);
		Property parseRel(unsigned int i);
		Property parseString(unsigned int i);
		
		Property parseToken(unsigned int i);
		Property parseTokenList(unsigned int i);

		Property parseProperty(unsigned int& i);

		void printProperty(Property& p, unsigned int level = 0U);
		void printXformTree(Xform& root, unsigned int level = 0U);

		std::vector<std::string> tokens;
		std::string scene_path;

		Xform default_prim;
		float meters_per_unit = 1.f;
		std::string up_axis = "Y";
	};


}
void penguinPT::loader::usd_scene_loader::printProperty(Property& p, unsigned int level) {
	file_util::alinea(level);
	std::cout << "Property : " << p.type << " : " << p.name << " : " << p.value << "\n";
	for (Property& pp : p.sub_property_list) printProperty(pp, level + 1);
}
void penguinPT::loader::usd_scene_loader::printXformTree(Xform& root, unsigned int level) {
	file_util::alinea(level);
	std::cout << "* " << root.type << " " << root.name << "\n";
	for (Property& p : root.property_list) printProperty(p, level + 2);
	for (Xform& c : root.childs) printXformTree(c, level + 2);
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseFloat(unsigned int i) {
	Property current_property;
	current_property.type = "float";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	current_property.value = tokens.at(i + 3);
	SYMBOL_CHECK(tokens.at(i + 4), "endline", INVALID_USD);
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseFloat3(unsigned int i) {
	Property current_property;
	current_property.type = tokens.at(i);
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	current_property.value = tokens.at(i + 3) + tokens.at(i + 4) + tokens.at(i + 5);
	SYMBOL_CHECK(tokens.at(i + 6), "endline", INVALID_USD)
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseFloatList(unsigned int i) {
	Property current_property;
	current_property.type = "float[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline") {
		current_property.value += tokens.at(y++);
	}
	SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseFloat3List(unsigned int i) {
	Property current_property;
	current_property.type = "float3[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline") {
		current_property.value += tokens.at(y++);
	}
	SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseInt(unsigned int i) {
	Property current_property;
	current_property.type = "int";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	current_property.value = tokens.at(i + 3);
	SYMBOL_CHECK(tokens.at(i + 4), "endline", INVALID_USD)
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseInt3List(unsigned int i) {
	Property current_property;
	current_property.type = "int3[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline") {
		current_property.value += tokens.at(y++);
	}
	SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseBool(unsigned int i) {
	Property current_property;
	current_property.type = "bool";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	current_property.value = tokens.at(i + 3);
	SYMBOL_CHECK(tokens.at(i + 4), "endline", INVALID_USD)
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseIntList(unsigned int i) {
	Property current_property;
	current_property.type = "int[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline") {
		current_property.value += tokens.at(y++);
	}
	SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseBoolList(unsigned int i, int& line_count) {
	Property current_property;
	current_property.type = "bool[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline" && tokens.at(y) != "(") {
		current_property.value += tokens.at(y++);
	}
	if (tokens.at(y) == "(") {
		y += 2; // land on first sub property name
		while (tokens.at(y) != ")")
		{
			Property sub_current;
			sub_current.type = "string";
			sub_current.name = tokens.at(y);
			SYMBOL_CHECK(tokens.at(y + 1), "=", INVALID_USD);
			sub_current.value = tokens.at(y + 2);
			y += 4;
			line_count++;

			current_property.sub_property_list.push_back(sub_current);
		}
		line_count++;
	}
	else SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);

	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseNormal3fList(unsigned int i, int& line_count) {
	Property current_property;
	current_property.type = "normal3f[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline" && tokens.at(y) != "(") {
		current_property.value += tokens.at(y++);
	}
	if (tokens.at(y) == "(") {
		y += 2; // land on first sub property name
		while (tokens.at(y) != ")")
		{
			Property sub_current;
			sub_current.type = "string";
			sub_current.name = tokens.at(y);
			SYMBOL_CHECK(tokens.at(y + 1), "=", INVALID_USD);
			sub_current.value = tokens.at(y + 2);
			y += 4;
			line_count++;

			current_property.sub_property_list.push_back(sub_current);
		}
		line_count++;
	}
	else SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);

	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseTexcoord2fList(unsigned int i, int& line_count) {
	Property current_property;
	current_property.type = "texcoord2f[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline" && tokens.at(y) != "(") {
		current_property.value += tokens.at(y++);
	}
	if (tokens.at(y) == "(") {
		y += 2; // land on first sub property name
		while (tokens.at(y) != ")")
		{
			Property sub_current;
			sub_current.type = "string";
			sub_current.name = tokens.at(y);
			SYMBOL_CHECK(tokens.at(y + 1), "=", INVALID_USD);
			sub_current.value = tokens.at(y + 2);
			y += 4;
			line_count++;

			current_property.sub_property_list.push_back(sub_current);
		}
		line_count++;
	}
	else SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);

	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseRel(unsigned int i) {
	Property current_property;
	current_property.type = "rel";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	current_property.value = tokens.at(i + 3);
	SYMBOL_CHECK(tokens.at(i + 4), "endline", INVALID_USD);
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseString(unsigned int i) {
	Property current_property;
	current_property.type = "string";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	current_property.value += tokens.at(y++);
	while (tokens.at(y) != "endline") {
		current_property.value += " " + tokens.at(y++);
	}
	current_property.value = file_util::remove_quote(current_property.value);
	SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);
	return current_property;
}

penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseToken(unsigned int i) {
	Property current_property;
	current_property.type = "token";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	current_property.value = file_util::remove_quote(tokens.at(i + 3));
	SYMBOL_CHECK(tokens.at(i + 4), "endline", INVALID_USD)
	return current_property;
}
penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseTokenList(unsigned int i) {
	Property current_property;
	current_property.type = "token[]";
	current_property.name = tokens.at(i + 1);
	SYMBOL_CHECK(tokens.at(i + 2), "=", INVALID_USD);
	unsigned int y = i + 3;
	while (tokens.at(y) != "endline") {
		current_property.value += file_util::remove_quote(tokens.at(y++));
	}
	SYMBOL_CHECK(tokens.at(y), "endline", INVALID_USD);
	return current_property;
}

penguinPT::loader::Property penguinPT::loader::usd_scene_loader::parseProperty(unsigned int& i) {
	Property current_property;
	if (tokens.at(i) == "float3[]") {
		current_property = parseFloat3List(i);
	}
	else if (tokens.at(i) == "int[]") {
		current_property = parseIntList(i);
	}
	else if (tokens.at(i) == "bool[]") {
		int lc = 0;
		current_property = parseBoolList(i, lc);
		for (int k = 0; k < lc; k++) get_line(i);
	}
	else if (tokens.at(i) == "normal3f[]") {
		int lc = 0;
		current_property = parseNormal3fList(i, lc);
		for (int k = 0; k < lc; k++) get_line(i);
	}
	else if (tokens.at(i) == "point3f[]") {
		current_property = parseInt3List(i);
	}
	else if (tokens.at(i) == "texCoord2f[]") {
		int lc = 0;
		current_property = parseTexcoord2fList(i, lc);
		for (int k = 0; k < lc; k++) get_line(i);
	}
	// we don't like doubles here
	else if (tokens.at(i) == "float3" || tokens.at(i) == "double3") {
		current_property = parseFloat3(i);
	}
	else if (tokens.at(i) == "float") {
		current_property = parseFloat(i);
	}
	else if (tokens.at(i) == "int") {
		current_property = parseInt(i);
	}
	else if (tokens.at(i) == "bool") {
		current_property = parseBool(i);
	}
	else if (tokens.at(i) == "custom") {
		i++;
		current_property = parseProperty(i);
		current_property.type = "custom " + current_property.type;
	}
	else if (tokens.at(i) == "uniform") {
		i++;
		current_property = parseProperty(i);
		current_property.type = "uniform " + current_property.type;
	}
	else if (tokens.at(i) == "token") {
		current_property = parseToken(i);
	}
	else if (tokens.at(i) == "token[]") {
		current_property = parseTokenList(i);
	}
	else if (tokens.at(i) == "string") {
		current_property = parseString(i);
	}
	else if (tokens.at(i) == "rel") {
		current_property = parseRel(i);
	}
	return current_property;
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
	current.type = tokens.at(i++);
	current.name = file_util::remove_quote(tokens.at(i++));
#if USD_TRACK_PRINT
	std::cout << "New Xform called " << current.name << " initialized.\n";
#endif

	bool hasParameters = false;

	// we have 4 cases : 
	//  - Xform has parameters and parameters starts directly : "("
	//  - Xform has parameters and parameters starts on next line : "endline" -> "("
	//  - Xform has no parameters and body starts directly : "{"
	//  - Xform has no parameters and body starts on next line : "endline" -> "{"

	if (tokens.at(i) == "endline") i++;
	if (tokens.at(i) == "(") {
#if USD_TRACK_PRINT
		std::cout << "Xform " << current.name << " has parameters.\n";
#endif
		hasParameters = true;
	}
	else if (tokens.at(i) == "{") {
#if USD_TRACK_PRINT
		std::cout << "Xform " << current.name << " has no parameters.\n";
#endif
	}
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

			if (
				tokens.at(i) == "Xform" || 
				tokens.at(i) == "Volume" || 
				tokens.at(i) == "Mesh" || 
				tokens.at(i) == "Camera" || 
				tokens.at(i) == "SphereLight" || 
				tokens.at(i) == "DomeLight" ||
				tokens.at(i) == "OpenVDBAsset" ||
				tokens.at(i) == "Scope" ||
				tokens.at(i) == "Material")
			{
				current.childs.push_back(build_Xform_tree(i));
			}
		}
		else {
			Property current_property = parseProperty(i);
#if USD_TRACK_PRINT
			printProperty(current_property);
#endif
			current.property_list.push_back(current_property);

			get_line(i);
		}
	}

	if (tokens.at(i + 1) == "endline") {
		get_line(i);
	}
	else {
		std::cout << "ERROR -- Uncorresponding token.\n";
		return Xform();
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
	if (tokens.at(i++) == "def" && tokens.at(i) == "Xform") {
		std::cout << "First prim found.\n";
		if (file_util::remove_quote(tokens.at(i + 1)) == defaultPrim_name) {
			// build tree
			default_prim = build_Xform_tree(i);

			// print tree
			printXformTree(default_prim);
		}
		else {
			std::cout << "First Prim name (" << tokens.at(i) << ") found is not matching the one mentionned earlier : " << defaultPrim_name << ".\n";
			return false;
		}
	}
}