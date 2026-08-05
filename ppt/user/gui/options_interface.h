// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/utility.h>
#include <SFML/Graphics.hpp>

#include <ppt/user/gui/slider.h>
#include <ppt/user/gui/button.h>
#include <ppt/user/gui/text_box.h>

#include <ppt/core/renderer_services.h>
#include <ppt/loaders/SDPT_loader.h>
#include <ppt/loaders/obj_loader.h>
#include <ppt/loaders/nanovdb_loader.h>
#include <ppt/core/Mesh.h>
#include <ppt/core/envmap.h>

namespace penguinPT {
	class OptionsInterface {
	public:
		OptionsInterface() {}
		~OptionsInterface() {}

		void oiOnInit(renderer_services* rs);
		void oiClear();

		void oiUpdate(
			sf::RenderWindow* target,
			sf::RenderWindow* window_render,
			renderer_services* rs, 
			sf::Texture* display_texture,
			loader::obj_loader* loader_obj,
			loader::nanovdb_loader* loader_nvdb,
			loader::envmap_loader* loader_envmap,
			loader::SDPT_Loader* loader_sdpt,
			BVH* geom_ptr,
			texture_manager* manager_textures);
		void oiDraw(sf::RenderWindow* target);

		// this function is not static because it needs the font stored in OptionsInterface
		void engineSelectorProcedure(renderer_services* rs);

		static void introWindow(int seconds);
		static void sceneLoadProcedure(
			renderer_services* rs, 
			loader::obj_loader* loader_obj, 
			loader::nanovdb_loader* loader_nvdb, 
			loader::envmap_loader* loader_envmap, 
			loader::SDPT_Loader* loader_sdpt,
			BVH* geom_ptr,
			texture_manager* manager_textures);

		sf::Texture getTextureFromIndex(unsigned int index);

		enum class LanguageTypes {
			FR_FR,
			EN_US
		};
		enum class WindowResolution {
			R_640x480,
			R_1280x720,
			R_1920x1080,
			R_2560x1440
		};

		void setLanguage(LanguageTypes l);
		void setResolution(WindowResolution r);
	private:
		sf::Texture m_symbol_sheet;
		std::string m_symbol_sheet_path = "assets/GUI/symbol_sheet.png";

		sf::Font m_font;
		
		bool m_is_render_engine_selected = false;

		std::vector<sf::RectangleShape> m_rectangles;
		std::vector<sf::Texture> m_textures;
		std::vector<GUI::ButtonTextured> m_button_textured_list_global;
		std::vector<GUI::ButtonTextual> m_button_textual_list_global;
		std::vector<GUI::CheckBox> m_checkbox_list_global;

		enum m_table {
			camera = 0,
			scene,
			options,
			post_processing
		};
		m_table m_current_table = m_table::camera;

		// specific to tables
		void* m_toggled_slider_ptr = nullptr;
		std::vector<std::vector<GUI::sliderBase<float>>> m_float_slider_list_tables;
		std::vector<std::vector<GUI::ButtonTextual>> m_button_textual_list_tables;
		std::vector<std::vector<GUI::ButtonTextured>> m_button_textured_list_tables;
		std::vector<std::vector<GUI::CheckBox>> m_checkbox_list_tables;
		std::vector<std::vector<GUI::TextBox>> m_textbox_list_tables;
		std::vector<std::vector<GUI::SelectorBox>> m_selectorbox_list_tables;

		// accessors
		GUI::ButtonTextured& getTexturedButtonGlobal(std::string name);
		GUI::ButtonTextual& getTextualButtonGlobal(std::string name);

		GUI::ButtonTextured& getTexturedButtonTable(std::string name, m_table table);
		GUI::ButtonTextual& getTextualButtonTable(std::string name, m_table table);

		GUI::SelectorBox& getSelectorBoxTable(std::string name, m_table table);

		LanguageTypes language = LanguageTypes::EN_US;
		std::string getSentence(std::string english_base);

		WindowResolution wnd_res = WindowResolution::R_1280x720;

		void saveOptions();
	};

	static void save_screenshot(sf::Texture& tex) {
		std::time_t result = std::time(nullptr);

		char buffer[26];

		time(&result);
		ctime_s(buffer, sizeof(buffer), &result);

		std::string name;
		for (int i = 0; i < strlen(buffer); i++) { // avoid line return character
			if (buffer[i] == ' ' || buffer[i] == ':') name += '_';
			else if (buffer[i] == '\n') break;
			else name += buffer[i];
		}

		sf::Image saver = tex.copyToImage();
		saver.saveToFile("saves/screenshots/" + name + ".png");
		std::cout << "Image saved as" << name << ".png\n";
	}

}

void penguinPT::OptionsInterface::oiOnInit(renderer_services* rs) {
	m_font.loadFromFile("assets/GUI/Montserrat-Light.ttf");

	// logo
	m_textures.push_back(sf::Texture());
	m_textures.back().loadFromFile("assets/GUI/logo_v0.png"); // INDEX : 0

	m_textures.push_back(sf::Texture());
	m_textures.back().loadFromFile("assets/GUI/options_bkg.png"); // INDEX : 1

	// symbols
	if (!m_symbol_sheet.loadFromFile(m_symbol_sheet_path)) {
		error_log("Failed to load symbol sheet.");
	}

	// background -> first drawed
	m_rectangles.push_back(sf::RectangleShape({ 64,512 }));
	m_rectangles.back().setTexture(&m_textures.at(1));

	// toolbar
	m_rectangles.push_back(sf::RectangleShape({ 32, 512 }));
	m_rectangles.back().setPosition({ 64,0 });
	m_rectangles.back().setFillColor({ 31,31,31 });
	m_rectangles.back().setOutlineThickness(-1);
	m_rectangles.back().setOutlineColor({ 61,61,61 });

	// table selector
	m_rectangles.push_back(sf::RectangleShape({ 416,48 }));
	m_rectangles.back().setPosition({ 96,0 });
	m_rectangles.back().setFillColor({ 31,31,31 });
	m_rectangles.back().setOutlineThickness(-1);
	m_rectangles.back().setOutlineColor({ 61,61,61 });

	m_button_textured_list_global.push_back(GUI::ButtonTextured("render:launch", { 64,0 }, { 32,32 }, &m_symbol_sheet, { 0,0, 32, 32 }));
	m_button_textured_list_global.push_back(GUI::ButtonTextured("render:pause", { 64,32 }, { 32,32 }, &m_symbol_sheet, { 96,32, 32, 32 }));
	m_button_textured_list_global.push_back(GUI::ButtonTextured("render:stop", { 64,64 }, { 32,32 }, &m_symbol_sheet, { 32,0, 32, 32 }));
	//m_button_textured_list_global.push_back(GUI::ButtonTextured("parameters:open", { 64,64 }, { 32,32 }, &m_symbol_sheet, { 96,0, 32, 32 }));
	m_button_textured_list_global.push_back(GUI::ButtonTextured("screenshot:take", { 64,96 }, { 32,32 }, &m_symbol_sheet, { 64,32, 32, 32 }));
	m_button_textured_list_global.push_back(GUI::ButtonTextured("app:exit", { 64,480 }, { 32,32 }, &m_symbol_sheet, { 64,0, 32, 32 }));

	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:camera", getSentence("Camera"), { 96, 0 }, { 208,24 }, &m_font));
	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:options", getSentence("App properties"), { 96, 24 }, { 208,24 }, &m_font));
	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:scene", getSentence("Scene"), { 304, 0 }, { 208,24 }, &m_font));
	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:postprocess", getSentence("Post processing"), { 304, 24 }, { 208,24 }, &m_font));


	// init tables vector
	m_button_textual_list_tables.push_back(std::vector<GUI::ButtonTextual>());
	m_button_textual_list_tables.push_back(std::vector<GUI::ButtonTextual>());
	m_button_textual_list_tables.push_back(std::vector<GUI::ButtonTextual>());
	m_button_textual_list_tables.push_back(std::vector<GUI::ButtonTextual>());

	m_button_textured_list_tables.push_back(std::vector<GUI::ButtonTextured>());
	m_button_textured_list_tables.push_back(std::vector<GUI::ButtonTextured>());
	m_button_textured_list_tables.push_back(std::vector<GUI::ButtonTextured>());
	m_button_textured_list_tables.push_back(std::vector<GUI::ButtonTextured>());

	m_checkbox_list_tables.push_back(std::vector<GUI::CheckBox>());
	m_checkbox_list_tables.push_back(std::vector<GUI::CheckBox>());
	m_checkbox_list_tables.push_back(std::vector<GUI::CheckBox>());
	m_checkbox_list_tables.push_back(std::vector<GUI::CheckBox>());

	m_textbox_list_tables.push_back(std::vector<GUI::TextBox>());
	m_textbox_list_tables.push_back(std::vector<GUI::TextBox>());
	m_textbox_list_tables.push_back(std::vector<GUI::TextBox>());
	m_textbox_list_tables.push_back(std::vector<GUI::TextBox>());

	m_float_slider_list_tables.push_back(std::vector<GUI::sliderBase<float>>());
	m_float_slider_list_tables.push_back(std::vector<GUI::sliderBase<float>>());
	m_float_slider_list_tables.push_back(std::vector<GUI::sliderBase<float>>());
	m_float_slider_list_tables.push_back(std::vector<GUI::sliderBase<float>>());

	m_selectorbox_list_tables.push_back(std::vector<GUI::SelectorBox>());
	m_selectorbox_list_tables.push_back(std::vector<GUI::SelectorBox>());
	m_selectorbox_list_tables.push_back(std::vector<GUI::SelectorBox>());
	m_selectorbox_list_tables.push_back(std::vector<GUI::SelectorBox>());

	// camera
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:zoom", getSentence("Camera zoom"), &m_font, &rs->mainCam.zoom, {100, 64}, {200, 24}, 0.1f, 10.f));
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:DOFstrength", getSentence("DOF strength"), &m_font, &rs->mainCam.DOF_strength, { 100, 96 }, { 200, 24 }, 0.f, 5.f));
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:focaldst", getSentence("Focal distance"), &m_font, &rs->mainCam.focal_distance, { 308, 96 }, { 200, 24 }, 0.1f, 1000.f));
	m_textbox_list_tables.at(m_table::camera).push_back(GUI::TextBox({ 96, 128 }, { 208, 24 }, getSentence("Lock Camera"), &m_font));
	m_checkbox_list_tables.at(m_table::camera).push_back(GUI::CheckBox("camera:lock", { 304, 128 }, { 24, 24 }, &m_symbol_sheet, { 36,36,24,24 }, &(rs->lock_camera)));
	m_textbox_list_tables.at(m_table::camera).push_back(GUI::TextBox({ 96, 160 }, { 208, 24 }, getSentence("Focus needed to move"), &m_font));
	m_checkbox_list_tables.at(m_table::camera).push_back(GUI::CheckBox("camera:focusneed", { 304, 160 }, { 24, 24 }, &m_symbol_sheet, { 36,36,24,24 }, &(rs->focus_needed)));
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:speed", getSentence("Camera moving speed"), &m_font, &rs->mainCam.speed, { 308, 64 }, { 200, 24 }, 1.f, 2000.f));
	m_textbox_list_tables.at(m_table::camera).push_back(GUI::TextBox({ 96, 192 }, { 208, 24 }, getSentence("Preview mode"), &m_font));
	m_checkbox_list_tables.at(m_table::camera).push_back(GUI::CheckBox("camera:previewmode", { 304, 192 }, { 24, 24 }, &m_symbol_sheet, { 36,36,24,24 }, &(rs->first_frame_mode)));
	
	// parameters
	m_textbox_list_tables.at(m_table::options).push_back(GUI::TextBox({ 96, 64 }, { 208, 24 }, getSentence("Language"), &m_font));
	m_textbox_list_tables.at(m_table::options).push_back(GUI::TextBox({ 96, 96 }, { 208, 24 }, getSentence("Viewport resolution"), &m_font));
	m_selectorbox_list_tables.at(m_table::options).push_back(GUI::SelectorBox("options:wnd_res", { "640x480", "1280x720", "1920x1080", "2560x1440" }, {308, 96}, {200, 24}, & m_font));
	switch (wnd_res)
	{
	case penguinPT::OptionsInterface::WindowResolution::R_640x480:
		m_selectorbox_list_tables.at(m_table::options).back().setState("640x480");
		break;
	case penguinPT::OptionsInterface::WindowResolution::R_1280x720:
		m_selectorbox_list_tables.at(m_table::options).back().setState("1280x720");
		break;
	case penguinPT::OptionsInterface::WindowResolution::R_1920x1080:
		m_selectorbox_list_tables.at(m_table::options).back().setState("1920x1080");
		break;
	case penguinPT::OptionsInterface::WindowResolution::R_2560x1440:
		m_selectorbox_list_tables.at(m_table::options).back().setState("2560x1440");
		break;
	default:
		break;
	}
	m_selectorbox_list_tables.at(m_table::options).push_back(GUI::SelectorBox("options:lang", { "English (US)", "Français (FR)" }, {308, 64}, {200, 24}, & m_font));
	switch (language)
	{
	case penguinPT::OptionsInterface::LanguageTypes::FR_FR:
		m_selectorbox_list_tables.at(m_table::options).back().setState("Français (FR)");
		break;
	case penguinPT::OptionsInterface::LanguageTypes::EN_US:
		m_selectorbox_list_tables.at(m_table::options).back().setState("English (US)");
		break;
	default:
		break;
	}
	m_button_textual_list_tables.at(m_table::options).push_back(GUI::ButtonTextual("options:save", getSentence("Save profile"), { 100, 128 }, { 408, 24 }, &m_font));

	// scene
	m_button_textual_list_tables.at(m_table::scene).push_back(GUI::ButtonTextual("scene:load", getSentence("Load new scene from .sdpt"), { 100, 64 }, { 408, 24 }, &m_font));
	m_float_slider_list_tables.at(m_table::scene).push_back(GUI::sliderBase<float>("scene:envmap:angle", getSentence("Environnement map angle"), &m_font, &rs->scene.environnement_map.angle, { 100, 96 }, { 408, 24 }, -1.f, 1.f));
	m_float_slider_list_tables.at(m_table::scene).push_back(GUI::sliderBase<float>("scene:envmap:strength", getSentence("Environnement map strength"), &m_font, &rs->scene.environnement_map.strength, { 100, 128 }, { 408, 24 }, 0.f, 1.f));

	// post process
	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:exposure", getSentence("Camera exposure"), &m_font, &rs->mainCam.exposure, { 100, 64 }, { 200, 24 }, 0.f, 2.f));
	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:contrast", getSentence("Camera constrast"), &m_font, &rs->mainCam.contrast, { 308, 64 }, { 200, 24 }, 0.f, 2.f));
	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:saturation", getSentence("Camera saturation"), &m_font, &rs->mainCam.saturation, { 100, 96 }, { 200, 24 }, 0.f, 2.f));
	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:vignette", getSentence("Camera vignette"), &m_font, &rs->mainCam.vignette, { 308, 96 }, { 200, 24 }, 0.f, 1.f));

	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:multiplier:red", getSentence("Red multiplier"), &m_font, &rs->mainCam.multiplier[0], {100, 128}, {408, 24}, 0.f, 2.f));
	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:multiplier:green", getSentence("Green multiplier"), &m_font, &rs->mainCam.multiplier[1], { 100, 160 }, { 408, 24 }, 0.f, 2.f));
	m_float_slider_list_tables.at(m_table::post_processing).push_back(GUI::sliderBase<float>("postprocess:multiplier:blue", getSentence("Blue multiplier"), &m_font, &rs->mainCam.multiplier[2], { 100, 192 }, { 408, 24 }, 0.f, 2.f));

}

void penguinPT::OptionsInterface::oiClear() {
}

void penguinPT::OptionsInterface::oiUpdate(sf::RenderWindow* target, sf::RenderWindow* window_render, renderer_services* rs, sf::Texture* display_texture, loader::obj_loader* loader_obj, loader::nanovdb_loader* loader_nvdb, loader::envmap_loader* loader_envmap, loader::SDPT_Loader* loader_sdpt, BVH* geom_ptr, texture_manager* manager_textures) {
	sf::Vector2f mouse_position = target->mapPixelToCoords(sf::Mouse::getPosition(*target));

	if (m_toggled_slider_ptr == nullptr) {
		if (getTexturedButtonGlobal("render:launch").isPressed(mouse_position)) {
			rs->first_frame_mode = false;
			if (!rs->is_paused && !rs->is_rendering) rs->frame_index = 0;
			rs->is_paused = false;
			rs->is_rendering = true;
		}
		if (getTexturedButtonGlobal("render:pause").isPressed(mouse_position) && !rs->first_frame_mode) {
			rs->is_paused = true;
		}

		if (getTexturedButtonGlobal("render:stop").isPressed(mouse_position)) {
			rs->is_rendering = false;
			rs->frame_index = 0;
		}
		if (getTexturedButtonGlobal("app:exit").isPressed(mouse_position)) {
			target->close();
		}
		if (getTexturedButtonGlobal("screenshot:take").isPressed(mouse_position)) {
			save_screenshot(*display_texture);
		}

		if (getTextualButtonGlobal("table:camera").isPressed(mouse_position)) {
			m_current_table = penguinPT::OptionsInterface::m_table::camera;
		}
		if (getTextualButtonGlobal("table:options").isPressed(mouse_position)) {
			m_current_table = penguinPT::OptionsInterface::m_table::options;
		}
		if (getTextualButtonGlobal("table:scene").isPressed(mouse_position)) {
			m_current_table = penguinPT::OptionsInterface::m_table::scene;
		}
		if (getTextualButtonGlobal("table:postprocess").isPressed(mouse_position)) {
			m_current_table = penguinPT::OptionsInterface::m_table::post_processing;
		}
	}
	// title shit
	if (rs->first_frame_mode || !rs->is_rendering) target->setTitle("PenguinPT -- V1.0");
	else if(rs->is_paused) target->setTitle("PenguinPT -- V1.0 -- Paused");
	else target->setTitle("PenguinPT -- V1.0 -- Rendering");
	
	if (m_toggled_slider_ptr == nullptr) for (GUI::CheckBox& b : m_checkbox_list_global) b.update(mouse_position);

	switch (m_current_table)
	{
	case penguinPT::OptionsInterface::m_table::camera:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::camera)) {
			s.update(mouse_position, &m_toggled_slider_ptr);
		}
		if (m_toggled_slider_ptr == nullptr) {
			for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) c.update(mouse_position);
			for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::camera)) s.update(mouse_position);
		}
		break;
	case penguinPT::OptionsInterface::m_table::scene:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::scene)) s.update(mouse_position, &m_toggled_slider_ptr);
		if (m_toggled_slider_ptr == nullptr) {
			for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) c.update(mouse_position);
			for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::scene)) s.update(mouse_position);


			if (getTextualButtonTable("scene:load", m_table::scene).isPressed(mouse_position)) {
				sceneLoadProcedure(rs, loader_obj, loader_nvdb, loader_envmap, loader_sdpt, geom_ptr, manager_textures);

				if (!window_render->isOpen()) window_render->create(sf::VideoMode(WINDOW_RES_X, WINDOW_RES_Y), "PenguinPT -- Render");
			}
		}

		break;
	case penguinPT::OptionsInterface::m_table::options:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::options)) s.update(mouse_position, &m_toggled_slider_ptr);
		if (m_toggled_slider_ptr == nullptr) {
			for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::options)) c.update(mouse_position);
			for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::options)) s.update(mouse_position);

			if (getTextualButtonTable("options:save", m_table::options).isPressed(mouse_position)) {
				saveOptions();
			}
		}
		break;
	case penguinPT::OptionsInterface::m_table::post_processing:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::post_processing)) s.update(mouse_position, &m_toggled_slider_ptr);
		if (m_toggled_slider_ptr == nullptr) {
			for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) c.update(mouse_position);
			for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::post_processing)) s.update(mouse_position);
		}
		break;
	default:
		break;
	}
}
void penguinPT::OptionsInterface::oiDraw(sf::RenderWindow* target) {
	bool can_activate = m_toggled_slider_ptr == nullptr;
	sf::Vector2f mouse_position = target->mapPixelToCoords(sf::Mouse::getPosition(*target));

	for (sf::RectangleShape& r : m_rectangles) target->draw(r);

	for (GUI::ButtonTextured& b : m_button_textured_list_global) b.draw(target, can_activate);
	for (GUI::ButtonTextual& b : m_button_textual_list_global) b.draw(target, can_activate);

	for (GUI::CheckBox& b : m_checkbox_list_global) b.draw(target, can_activate);

	switch (m_current_table)
	{
	case penguinPT::OptionsInterface::m_table::camera:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) b.draw(target, can_activate);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) b.draw(target, can_activate);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::camera)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) c.draw(target, can_activate);
		for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::camera)) s.draw(target, can_activate);
		break;
	case penguinPT::OptionsInterface::m_table::scene:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) b.draw(target, can_activate);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) b.draw(target, can_activate);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::scene)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) c.draw(target, can_activate);
		for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::scene)) s.draw(target, can_activate);
		break;
	case penguinPT::OptionsInterface::m_table::options:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::options)) b.draw(target, can_activate);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::options)) b.draw(target, can_activate);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::options)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::options)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::options)) c.draw(target, can_activate);
		for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::options)) s.draw(target, can_activate);
		break;
	case penguinPT::OptionsInterface::m_table::post_processing:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) b.draw(target, can_activate);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) b.draw(target, can_activate);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::post_processing)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) c.draw(target, can_activate);
		for (GUI::SelectorBox& s : m_selectorbox_list_tables.at(m_table::post_processing)) s.draw(target, can_activate);
		break;
	default:
		break;
	}
}
sf::Texture penguinPT::OptionsInterface::getTextureFromIndex(unsigned int index) {
	return m_textures.at(index);
}

penguinPT::GUI::ButtonTextured& penguinPT::OptionsInterface::getTexturedButtonGlobal(std::string name) {
	for (GUI::ButtonTextured& b : m_button_textured_list_global) if (b.getName() == name) return b;
	error_log("Failed to find textured button called " << name);
}

penguinPT::GUI::ButtonTextual& penguinPT::OptionsInterface::getTextualButtonGlobal(std::string name) {
	for (GUI::ButtonTextual& b : m_button_textual_list_global) if (b.getName() == name) return b;
	error_log("Failed to find textual button called " << name);
}

void penguinPT::OptionsInterface::introWindow(int seconds) {
	sf::RenderWindow temp_loader_window(sf::VideoMode(1024, 512), "", sf::Style::None);
	sf::RectangleShape temp_loader_rect({ 1024,512 });
	sf::Texture temp_loader;
	temp_loader.loadFromFile("assets/GUI/launch_img_v1.0.0.png");
	temp_loader_rect.setTexture(&temp_loader);

	temp_loader_window.clear();
	temp_loader_window.draw(temp_loader_rect);
	temp_loader_window.display();

	std::this_thread::sleep_for(std::chrono::seconds(seconds));

	temp_loader_window.close();
}

void penguinPT::OptionsInterface::engineSelectorProcedure(renderer_services* rs) {
	int GPU_count;
	cudaError_t device_check = cudaGetDeviceCount(&GPU_count);
	if (device_check != CUDA_SUCCESS) {
		rs->CUDA_CAPABLE_GPU = false;
	}
	else {
		sf::RenderWindow engine_selector_window(sf::VideoMode(512, 24 * (GPU_count + 1)), getSentence("Please select rendering engine"), sf::Style::Titlebar);
		std::vector<GUI::ButtonTextual> engines;

		for (int i = 0; i < GPU_count; i++) {
			cudaDeviceProp current_properties;
			CUDA_CHECK(cudaGetDeviceProperties(&current_properties, i));
			engines.push_back(GUI::ButtonTextual(current_properties.name, current_properties.name, { 0, i * 24.f }, { 512, 24 }, &m_font));
		}
		engines.push_back(GUI::ButtonTextual("cpu", "CPU rendering", {0, GPU_count * 24.f}, {512, 24}, &m_font));

		// loop
		while (engine_selector_window.isOpen()) {
			sf::Event event;
			while (engine_selector_window.pollEvent(event))
			{
				if (event.type == sf::Event::Closed) {
					engine_selector_window.close();
				}
			}

			sf::Vector2f mouse_position = engine_selector_window.mapPixelToCoords(sf::Mouse::getPosition(engine_selector_window));
			for (int i = 0; i < GPU_count + 1; i++) {
				if (engines.at(i).isPressed(mouse_position)) {
					if (i >= GPU_count) {
						rs->CUDA_CAPABLE_GPU = false;
					}
					else {
						rs->CUDA_CAPABLE_GPU = true;
						CUDA_CHECK(cudaSetDevice(i));
					}
					engine_selector_window.close();
				}
			}

			engine_selector_window.clear();
			for (GUI::ButtonTextual& b : engines) b.draw(&engine_selector_window);
			engine_selector_window.display();
		}
	}
}

penguinPT::GUI::ButtonTextured& penguinPT::OptionsInterface::getTexturedButtonTable(std::string name, m_table table) {
	for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(table)) if (b.getName() == name) return b;
	error_log("Failed to find textured button called " << name);
}
penguinPT::GUI::ButtonTextual& penguinPT::OptionsInterface::getTextualButtonTable(std::string name, m_table table) {
	for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(table)) if (b.getName() == name) return b;
	error_log("Failed to find textured button called " << name);
}

void penguinPT::OptionsInterface::sceneLoadProcedure(
	renderer_services* rs,
	loader::obj_loader* loader_obj,
	loader::nanovdb_loader* loader_nvdb,
	loader::envmap_loader* loader_envmap,
	loader::SDPT_Loader* loader_sdpt,
	BVH* geom_ptr,
	texture_manager* manager_textures)
{
	if (rs->CUDA_CAPABLE_GPU) cudaFree(geom_ptr);
	else free(geom_ptr);

	rs->clean_envmap();
	rs->clean_solid();
	rs->clean_volumes();

	loader_envmap->clean();
	loader_nvdb->clean();
	loader_obj->clean();

	manager_textures->clean();

	loader_sdpt->forceLoad(loader_obj, loader_nvdb, loader_envmap);

	BVH* bvh_ptr = loader_obj->getBVH(rs->CUDA_CAPABLE_GPU);
	std::vector<Mesh> try_mesh;

	try_mesh.push_back(Mesh(bvh_ptr));
	try_mesh.back().setTransforms(math::Mat4f());

	rs->scene.setBSDFList(loader_obj->bsdf_loader.BSDF_list, rs->CUDA_CAPABLE_GPU);
	rs->scene.setTLAS(try_mesh, rs->CUDA_CAPABLE_GPU);
	rs->scene.setVolumeList(loader_nvdb->vol_list, rs->CUDA_CAPABLE_GPU);

	loader_envmap->send_to_gpu(rs->scene.environnement_map, cudaFilterModeLinear);
}

void penguinPT::OptionsInterface::setLanguage(LanguageTypes l) {
	language = l;
}
std::string penguinPT::OptionsInterface::getSentence(std::string english_base) {
	switch (language)
	{
	case penguinPT::OptionsInterface::LanguageTypes::FR_FR:
		if (english_base == "Please select rendering engine") return "Veuillez selectionner le moteur de rendu";
		else if (english_base == "Camera zoom") return "Zoom de la caméra";
		else if (english_base == "Camera") return "Caméra";
		else if (english_base == "App properties") return "Propriétés d'app";
		else if (english_base == "Scene") return "Scene";
		else if (english_base == "Post processing") return "Edition d'image";
		else if (english_base == "DOF strength") return "Profondeur de champ";
		else if (english_base == "Focal distance") return "Distance focale";
		else if (english_base == "Lock Camera") return "Bloquer la caméra";
		else if (english_base == "Focus needed to move") return "Focus nécessaire";
		else if (english_base == "Camera moving speed") return "Vitesse de la caméra";
		else if (english_base == "Preview mode") return "Pré-rendu";
		else if (english_base == "Load new scene from .sdpt") return "Charger une scène depuis un .sdpt";
		else if (english_base == "Environnement map angle") return "Angle de l'environnement";
		else if (english_base == "Environnement map strength") return "Intensité de l'environnement";
		else if (english_base == "Camera exposure") return "Exposition";
		else if (english_base == "Camera constrast") return "Contraste";
		else if (english_base == "Camera saturation") return "Saturation";
		else if (english_base == "Camera vignette") return "Vignette";
		else if (english_base == "Red multiplier") return "Multiplicateur de rouge";
		else if (english_base == "Green multiplier") return "Multiplicateur de vert";
		else if (english_base == "Blue multiplier") return "Multiplicateur de bleu";
		else if (english_base == "Language") return "Selectionnez la langue";
		else if (english_base == "Viewport resolution") return "Résolution de la vue";
		else if (english_base == "Save profile") return "Enregistrer les modifications";
		break;
	case penguinPT::OptionsInterface::LanguageTypes::EN_US:
		return english_base;
		break;
	}
	return english_base;
}
void penguinPT::OptionsInterface::setResolution(WindowResolution r) {
	wnd_res = r;
}
void penguinPT::OptionsInterface::saveOptions() {
	std::ofstream file("assets/options.txt", std::ios::trunc);

	if (file.is_open()) {
		{
			std::string l = getSelectorBoxTable("options:wnd_res", m_table::options).getState();
			if (l == "640x480") file << "RESOLUTION R_640x480";
			else if (l == "1280x720") file << "RESOLUTION R_1280x720";
			else if (l == "1920x1080") file << "RESOLUTION R_1920x1080";
			else if (l == "2560x1440") file << "RESOLUTION R_2560x1440";
		}

		file << std::endl;

		{
			std::string l = getSelectorBoxTable("options:lang", m_table::options).getState();
			if (l == "English (US)") file << "LANGUAGE EN_US";
			else if (l == "Français (FR)") file << "LANGUAGE FR_FR";
		}

		file.close();
	}
	else {
		error_log("Failed to open options.txt");
	}
}
penguinPT::GUI::SelectorBox& penguinPT::OptionsInterface::getSelectorBoxTable(std::string name, m_table table) {
	for (GUI::SelectorBox& b : m_selectorbox_list_tables.at(table)) if (b.getName() == name) return b;
	error_log("Failed to find selector box called " << name);
}