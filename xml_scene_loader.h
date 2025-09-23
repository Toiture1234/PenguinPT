#pragma once

namespace penguinPT::loader {
	class xml_scene_loader {
	public:
		obj_loader typeObj;
		std::vector<Sphere> primitiveSphere;

		bool load_xml_scene(std::string path);

		xml_scene_loader() {}
		~xml_scene_loader() {}
	};
}