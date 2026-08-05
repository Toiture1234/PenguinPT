// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/core/renderer_services.h>
#include <ppt/user/gui/options_interface.h>

#include <ppt/core/envmap.h>
#include <ppt/loaders/obj_loader.h>
#include <ppt/loaders/nanovdb_loader.h>
#include <ppt/loaders/BVH_loader.h>
#include <ppt/loaders/SDPT_loader.h>

#include <ppt/loaders/BVH_loader.h>

#define INTRO_DURATION 0

namespace penguinPT {
	struct AppOptions {
		OptionsInterface::LanguageTypes language = OptionsInterface::LanguageTypes::EN_US;

		enum class AppColor {
			DARK,
			BRIGHT,
			YELLOW,
			LIGHT_BLUE
		} color = AppColor::DARK;

		OptionsInterface::WindowResolution wnd_res = OptionsInterface::WindowResolution::R_1280x720;

		unsigned int intro_duration = 2;
	};
	struct LoaderModule {
		loader::obj_loader loader_obj;
		loader::envmap_loader loader_envmap;
		loader::nanovdb_loader loader_nvdb;
		loader::SDPT_Loader loader_sdpt;
		loader::BVHLoader loader_bvh;
		texture_manager manager_textures;
	};
	class Application {
	public:
		Application() {}
		~Application() {}

		void initApplication();
		void initLoaders();

		void clean();

		sf::Vector2u getViewportResolution();

		renderer_services rs;
		OptionsInterface user_interface;
		LoaderModule scene_ressource_loader;

		AppOptions options;

		BVH* m_bvh_ptr;
		bool edit_mode = false;
	private:
		void retrieveOptions();
	};
}

void penguinPT::Application::initApplication() {
	retrieveOptions();
	this->rs.mainCam.speed = 500.f;
	this->rs.mainCam.zoom = 1.f;
	this->rs.mainCam.position = { 0.f, 0.f, 0.f };
	this->rs.is_rendering = false;
	this->rs.focus_needed = false;
	sf::Vector2u wnd_size = getViewportResolution();
	this->rs.width = wnd_size.x;
	this->rs.height = wnd_size.y;

	this->user_interface.setResolution(options.wnd_res);
	this->user_interface.setLanguage(options.language);
	this->user_interface.oiOnInit(&rs);

	OptionsInterface::introWindow(options.intro_duration);

	this->user_interface.engineSelectorProcedure(&rs);

	this->rs.fill_host_pixel_buffer();
}
void penguinPT::Application::initLoaders() {
	this->scene_ressource_loader.loader_obj.set_texture_manager(&this->scene_ressource_loader.manager_textures);
	this->scene_ressource_loader.loader_sdpt.set_GPU_Compat(this->rs.CUDA_CAPABLE_GPU);
}

void penguinPT::Application::clean() {
	user_interface.oiClear();

	rs.clean_all();

	if (rs.CUDA_CAPABLE_GPU) cudaFree(m_bvh_ptr);
	else free(m_bvh_ptr);

	scene_ressource_loader.loader_obj.clean();
	scene_ressource_loader.loader_nvdb.clean();
	scene_ressource_loader.loader_envmap.clean();

	scene_ressource_loader.manager_textures.clean();
}
sf::Vector2u penguinPT::Application::getViewportResolution() {
	switch (options.wnd_res)
	{
	case OptionsInterface::WindowResolution::R_640x480:
		return sf::Vector2u(640U, 480U);
		break;
	case OptionsInterface::WindowResolution::R_1280x720:
		return sf::Vector2u(1280U, 720U);
		break;
	case OptionsInterface::WindowResolution::R_1920x1080:
		return sf::Vector2u(1920U, 1080U);
		break;
	case OptionsInterface::WindowResolution::R_2560x1440:
		return sf::Vector2u(2560U, 1440U);
		break;
	}
}
void penguinPT::Application::retrieveOptions() {

	std::ifstream file("assets/options.txt");

	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line))
		{
			std::vector<std::string> tokens = file_util::get_line_tokens(line, { ' ' });
			
			if (!tokens.empty()) {
				if (tokens.at(0) == "LANGUAGE") {
					std::string s = tokens.at(1);
					if (s == "FR_FR") options.language = OptionsInterface::LanguageTypes::FR_FR;
					else if (s == "EN_US") options.language = OptionsInterface::LanguageTypes::EN_US;
				}
				if (tokens.at(0) == "RESOLUTION") {
					std::string s = tokens.at(1);
					if (s == "R_640x480") options.wnd_res = OptionsInterface::WindowResolution::R_640x480;
					else if (s == "R_1280x720") options.wnd_res = OptionsInterface::WindowResolution::R_1280x720;
					else if (s == "R_1920x1080") options.wnd_res = OptionsInterface::WindowResolution::R_1920x1080;
					else if (s == "R_2560x1440") options.wnd_res = OptionsInterface::WindowResolution::R_2560x1440;
				}
				if (tokens.at(0) == "INTRO_DURATION") {
					std::string s = tokens.at(1);
					options.intro_duration = std::stoi(s);
				}
			}
		}
		file.close();
	}
	else {
		error_log("Failed to open options.txt file");
	}
}