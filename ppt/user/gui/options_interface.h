#pragma once

#include <ppt/util/utility.h>
#include <SFML/Graphics.hpp>

#include <ppt/user/gui/slider.h>
#include <ppt/user/gui/button.h>
#include <ppt/user/gui/text_box.h>

#include <ppt/core/renderer_services.h>

namespace penguinPT {
	class OptionsInterface {
	public:
		OptionsInterface() {}
		~OptionsInterface() {}

		void oiOnInit(renderer_services* rs);
		void oiClear();

		void oiUpdate(sf::RenderWindow* target, renderer_services* rs);
		void oiDraw(sf::RenderWindow* target);

		// this function is not static because it needs the font stored in OptionsInterface
		void engineSelectorProcedure(renderer_services* rs);

		static void introWindow(int seconds);

		sf::Texture getTextureFromIndex(unsigned int index);
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
			envmap,
			post_processing
		};
		m_table m_current_table = m_table::camera;

		// specific to tables
		std::vector<std::vector<GUI::sliderBase<float>>> m_float_slider_list_tables;
		std::vector<std::vector<GUI::ButtonTextual>> m_button_textual_list_tables;
		std::vector<std::vector<GUI::ButtonTextured>> m_button_textured_list_tables;
		std::vector<std::vector<GUI::CheckBox>> m_checkbox_list_tables;
		std::vector<std::vector<GUI::TextBox>> m_textbox_list_tables;

		// accessors
		GUI::ButtonTextured& getTexturedButtonGlobal(std::string name);
		GUI::ButtonTextual& getTextualButtonGlobal(std::string name);
	};

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
	m_button_textured_list_global.push_back(GUI::ButtonTextured("render:stop", { 64,32 }, { 32,32 }, &m_symbol_sheet, { 32,0, 32, 32 }));
	m_button_textured_list_global.push_back(GUI::ButtonTextured("parameters:open", { 64,64 }, { 32,32 }, &m_symbol_sheet, { 96,0, 32, 32 }));
	m_button_textured_list_global.push_back(GUI::ButtonTextured("app:exit", { 64,480 }, { 32,32 }, &m_symbol_sheet, { 64,0, 32, 32 }));

	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:camera", "Camera", { 96, 0 }, { 208,24 }, &m_font));
	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:envmap", "Environnement map", { 96, 24 }, { 208,24 }, &m_font));
	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:scene", "Scene", { 304, 0 }, { 208,24 }, &m_font));
	m_button_textual_list_global.push_back(GUI::ButtonTextual("table:postprocess", "Post processing", { 304, 24 }, { 208,24 }, &m_font));


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

	// camera
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:zoom", "Camera zoom", &m_font, &rs->mainCam.zoom, {100, 64}, {200, 24}, 0.1f, 10.f));
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:DOFstrength", "DOF strength", &m_font, &rs->mainCam.DOF_strength, { 100, 96 }, { 200, 24 }, 0.f, 5.f));
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:focaldst", "Focal distance", &m_font, &rs->mainCam.focal_distance, { 308, 96 }, { 200, 24 }, 0.1f, 1000.f));
	m_textbox_list_tables.at(m_table::camera).push_back(GUI::TextBox({ 96, 128 }, { 208, 24 }, "Lock Camera", &m_font));
	m_checkbox_list_tables.at(m_table::camera).push_back(GUI::CheckBox("camera:lock", { 304, 128 }, { 24, 24 }, &m_symbol_sheet, { 36,36,24,24 }, &(rs->lock_camera)));
	m_textbox_list_tables.at(m_table::camera).push_back(GUI::TextBox({ 96, 160 }, { 208, 24 }, "Focus needed to move", &m_font));
	m_checkbox_list_tables.at(m_table::camera).push_back(GUI::CheckBox("camera:focusneed", { 304, 160 }, { 24, 24 }, &m_symbol_sheet, { 36,36,24,24 }, &(rs->focus_needed)));
	m_float_slider_list_tables.at(m_table::camera).push_back(GUI::sliderBase<float>("camera:speed", "Camera moving speed", &m_font, &rs->mainCam.speed, { 308, 64 }, { 200, 24 }, 1.f, 5000.f));

	// envmap

	// scene

	// post process
}

void penguinPT::OptionsInterface::oiClear() {
}

void penguinPT::OptionsInterface::oiUpdate(sf::RenderWindow* target, renderer_services* rs) {
	sf::Vector2f mouse_position = target->mapPixelToCoords(sf::Mouse::getPosition(*target));

	if (getTexturedButtonGlobal("render:launch").isPressed(mouse_position)) {
		rs->is_rendering = true;
		rs->frame_index = 0;
	}
	if (getTexturedButtonGlobal("render:stop").isPressed(mouse_position)) {
		rs->is_rendering = false;
		rs->frame_index = 0;
	}
	if (getTexturedButtonGlobal("app:exit").isPressed(mouse_position)) {
		target->close();
	}

	if (getTextualButtonGlobal("table:camera").isPressed(mouse_position)) {
		m_current_table = penguinPT::OptionsInterface::m_table::camera;
	}
	if (getTextualButtonGlobal("table:envmap").isPressed(mouse_position)) {
		m_current_table = penguinPT::OptionsInterface::m_table::envmap;
	}
	if (getTextualButtonGlobal("table:scene").isPressed(mouse_position)) {
		m_current_table = penguinPT::OptionsInterface::m_table::scene;
	}
	if (getTextualButtonGlobal("table:postprocess").isPressed(mouse_position)) {
		m_current_table = penguinPT::OptionsInterface::m_table::post_processing;
	}

	for (GUI::CheckBox& b : m_checkbox_list_global) b.update(mouse_position);

	switch (m_current_table)
	{
	case penguinPT::OptionsInterface::m_table::camera:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::camera)) s.update(mouse_position);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) c.update(mouse_position);
		break;
	case penguinPT::OptionsInterface::m_table::scene:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::scene)) s.update(mouse_position);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) c.update(mouse_position);
		break;
	case penguinPT::OptionsInterface::m_table::envmap:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::envmap)) s.update(mouse_position);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::envmap)) c.update(mouse_position);
		break;
	case penguinPT::OptionsInterface::m_table::post_processing:
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::post_processing)) s.update(mouse_position);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) c.update(mouse_position);
		break;
	default:
		break;
	}
}
void penguinPT::OptionsInterface::oiDraw(sf::RenderWindow* target) {
	sf::Vector2f mouse_position = target->mapPixelToCoords(sf::Mouse::getPosition(*target));

	for (sf::RectangleShape& r : m_rectangles) target->draw(r);

	for (GUI::ButtonTextured& b : m_button_textured_list_global) b.draw(target);
	for (GUI::ButtonTextual& b : m_button_textual_list_global) b.draw(target);

	for (GUI::CheckBox& b : m_checkbox_list_global) b.draw(target);

	
	switch (m_current_table)
	{
	case penguinPT::OptionsInterface::m_table::camera:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) b.draw(target);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) b.draw(target);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::camera)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::camera)) c.draw(target);
		break;
	case penguinPT::OptionsInterface::m_table::scene:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) b.draw(target);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) b.draw(target);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::scene)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::scene)) c.draw(target);
		break;
	case penguinPT::OptionsInterface::m_table::envmap:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::envmap)) b.draw(target);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::envmap)) b.draw(target);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::envmap)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::envmap)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::envmap)) c.draw(target);
		break;
	case penguinPT::OptionsInterface::m_table::post_processing:
		for (GUI::ButtonTextured& b : m_button_textured_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) b.draw(target);
		for (GUI::ButtonTextual& b : m_button_textual_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) b.draw(target);
		for (GUI::sliderBase<float>& s : m_float_slider_list_tables.at(m_table::post_processing)) s.draw(target, mouse_position);
		for (GUI::TextBox& t : m_textbox_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) t.draw(target);
		for (GUI::CheckBox& c : m_checkbox_list_tables.at(penguinPT::OptionsInterface::m_table::post_processing)) c.draw(target);
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
		sf::RenderWindow engine_selector_window(sf::VideoMode(512, 24 * (GPU_count + 1)), "Please select rendering engine", sf::Style::Titlebar);
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