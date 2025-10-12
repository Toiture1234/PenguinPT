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

// maybe recursive ?

//void read_until_end()
bool penguinPT::loader::usd_scene_loader::load_usd_scene(std::string path) {

}