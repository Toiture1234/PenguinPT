#pragma once

namespace penguinPT::GUI {
	enum character_type
	{
		title = 30,
		body = 15,
		comment = 10
	};
	enum slider_type {
		horizontal = 0,
		vertical
	};
	namespace GUI_utility {
		void draw_GUI_rect(sf::RenderWindow* target, sf::Vector2f position, sf::Vector2f size) {
			sf::RectangleShape first_rectangle;
			first_rectangle.setPosition(position), first_rectangle.setSize(size);
			first_rectangle.setFillColor(sf::Color(61, 61, 61));

			target->draw(first_rectangle);

			sf::RectangleShape second_rectangle;
			second_rectangle.setPosition(position + sf::Vector2f(2, 2)), second_rectangle.setSize(size - sf::Vector2f(4, 4));
			second_rectangle.setFillColor(sf::Color(31, 31, 31));

			target->draw(second_rectangle);	
		}
		void draw_GUI_rect_bright(sf::RenderWindow* target, sf::Vector2f position, sf::Vector2f size) {
			sf::RectangleShape first_rectangle;
			first_rectangle.setPosition(position), first_rectangle.setSize(size);
			first_rectangle.setFillColor(sf::Color(91, 91, 91));

			target->draw(first_rectangle);

			sf::RectangleShape second_rectangle;
			second_rectangle.setPosition(position + sf::Vector2f(2, 2)), second_rectangle.setSize(size - sf::Vector2f(4, 4));
			second_rectangle.setFillColor(sf::Color(61, 61, 61));

			target->draw(second_rectangle);
		}
		void draw_GUI_rect_noBorders(sf::RenderWindow* target, sf::Vector2f position, sf::Vector2f size, int value = 31) {
			sf::RectangleShape first_rectangle;
			first_rectangle.setPosition(position), first_rectangle.setSize(size);
			first_rectangle.setFillColor(sf::Color(value, value, value));

			target->draw(first_rectangle);
		}

		sf::Vector2f get_mouse_inWindow(sf::RenderWindow* window) {
			return window->mapPixelToCoords(sf::Mouse::getPosition(*window));
		}
	}
	class text_manager {
	public:
		text_manager(std::string path) {
			font.loadFromFile(path);
			text.setFont(font);
		}
		~text_manager() {}

	public:
		sf::Font font;
		sf::Text text;

		void draw_text(sf::RenderWindow* target, std::string dsp_str, sf::Vector2f position, int ch_size);
	};

	class button {
	public:
		sf::Vector2f position;

		std::string name;
		character_type char_type = body;
		
		text_manager* txt_m = nullptr;
	public:
		button(std::string n) : name(n) {}
		~button() {}
		button(sf::Vector2f p, std::string n) : position(p), name(n) {}

		bool is_button_pressed(sf::Vector2f mouse_pos) const;
		void draw(sf::RenderWindow* target);

		void set_text_manager(text_manager* src);
		void set_string(std::string str);

	private:
		bool is_mouse_on_button(sf::Vector2f mouse_pos) const;
		sf::Vector2f size;
		std::string text;
	};

	class slider {
	public:
		sf::Vector2f position;
		sf::Vector2f size;

		std::string name;

		float interpolation_value = 0.5f;
		float min_bounds = 0.f;
		float max_bounds = 1.f;

		float slider_percentage = 0.5f;
		slider_type type = horizontal;
	public:
		slider() {}
		~slider() {}
		slider(sf::Vector2f p, sf::Vector2f s) : position(p), size(s) {}

		void draw(sf::RenderWindow* target);
		float update_slider(sf::Vector2f mouse_pos);

	private:
		bool is_mouse_on_slider(sf::Vector2f mouse_pos) const;
		bool toggled = false;

		float last_value_onScreen;
		float last_value_true;
	};

	class text_zone {
	public:
		sf::Vector2f position;
		
		std::string name;
		character_type char_type = body;
		text_manager* txt_m = nullptr;
	public:
		text_zone(std::string n) : name(n) {}
		~text_zone() {}
		text_zone(sf::Vector2f p, std::string n) : position(p), name(n) {}

		void draw(sf::RenderWindow* target);

		void set_string(std::string str);
	private:
		sf::Vector2f size;
		std::string text;
	};

	class GUI_manager {
	public:
		GUI_manager() {}
		~GUI_manager() {}

		std::vector<button> button_list;
		std::vector<slider> slider_list;
		std::vector<text_zone> text_zone_list;

		void draw_GUI(sf::RenderWindow* target);

		button& find_button(std::string name);
		slider& find_slider(std::string name);
		text_zone& find_text_zone(std::string name);
	};

	/////////////////////////////////////// text functions ///////////////////////////////////////
	void text_manager::draw_text(sf::RenderWindow* target, std::string dsp_str, sf::Vector2f position, int ch_size) {
		text.setString(dsp_str);
		text.setPosition(position);
		text.setCharacterSize(ch_size);

		target->draw(text);
	}
	/////////////////////////////////////// button functions ///////////////////////////////////////
	void button::set_text_manager(text_manager* src) {
		txt_m = src;
	}
	void button::set_string(std::string str) {
		text = str;

		if (txt_m == nullptr) {
			std::cout << "No text manager, please define it before.\n";
			return;
		}

		txt_m->text.setString(text);
		txt_m->text.setPosition(position);
		txt_m->text.setCharacterSize(char_type);

		sf::FloatRect r = txt_m->text.getLocalBounds();
		size = sf::Vector2f(r.getSize().x * 1.1f, r.getSize().y * 1.5f);
	}
	bool button::is_mouse_on_button(sf::Vector2f mouse_pos) const {
		return mouse_pos.x > position.x && mouse_pos.x < position.x + size.x && mouse_pos.y > position.y && mouse_pos.y < position.y + size.y;
	}
	bool button::is_button_pressed(sf::Vector2f mouse_pos) const {
		return is_mouse_on_button(mouse_pos) && sf::Mouse::isButtonPressed(sf::Mouse::Left);
	}
	void button::draw(sf::RenderWindow* target) {
		sf::Vector2f mouse_pos = GUI_utility::get_mouse_inWindow(target);
		if (is_mouse_on_button(mouse_pos)) {
			GUI_utility::draw_GUI_rect_bright(target, position, size);
		}
		else {
			GUI_utility::draw_GUI_rect(target, position, size);
		}

		if (txt_m != nullptr) txt_m->draw_text(target, text, position + size * 0.05f, char_type);
	}

	/////////////////////////////////////// test zone functions ///////////////////////////////////////
	void text_zone::set_string(std::string str) {
		text = str;

		if (txt_m == nullptr) {
			std::cout << "No text manager, please define it before.\n";
			return;
		}

		txt_m->text.setString(text);
		txt_m->text.setPosition(position);
		txt_m->text.setCharacterSize(char_type);

		sf::FloatRect r = txt_m->text.getLocalBounds();
		size = sf::Vector2f(r.getSize().x * 1.1f, r.getSize().y * 1.5f);
	}
	void text_zone::draw(sf::RenderWindow* target) {
		GUI_utility::draw_GUI_rect(target, position, size);

		if (txt_m != nullptr) txt_m->draw_text(target, text, position + size * 0.05f, char_type);
	}

	/////////////////////////////////////// slider functions ///////////////////////////////////////
	void slider::draw(sf::RenderWindow* target) {
		GUI_utility::draw_GUI_rect_noBorders(target, position, size);

		// calculate inner rect size
		float slider_size = (size.x * 0.92f) * slider_percentage;
		float pos_adderX = (size.x * 0.92f) * (1.f - slider_percentage) * interpolation_value;
		
		sf::Vector2f mouse_pos = GUI_utility::get_mouse_inWindow(target);
		GUI_utility::draw_GUI_rect_noBorders(target, position + sf::Vector2f(0.02f * size.x + pos_adderX, size.y * 0.02f), sf::Vector2f(slider_size, size.y * 0.96f), is_mouse_on_slider(mouse_pos) || toggled ? 91 : 61);
	}
	bool slider::is_mouse_on_slider(sf::Vector2f mouse_pos) const {
		return mouse_pos.x > position.x && mouse_pos.x < position.x + size.x && mouse_pos.y > position.y && mouse_pos.y < position.y + size.y;
	}
	float slider::update_slider(sf::Vector2f mouse_pos) {
		float middle = position.x + size.x * 0.5f;
		float slider_size = (size.x * 0.92f) * slider_percentage;

		// diff goes from 
		float diff = mouse_pos.x - middle;
		float range = size.x * 0.92f - slider_size;

		float value_relative = diff / range - last_value_onScreen;
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && is_mouse_on_slider(mouse_pos)) toggled = true;
		if (toggled) interpolation_value = last_value_true + value_relative;
		else last_value_onScreen = diff / range, last_value_true = interpolation_value;

		interpolation_value = util::clamp(interpolation_value, 0.f, 1.f);
		if (!sf::Mouse::isButtonPressed(sf::Mouse::Left)) toggled = false;
		return util::mix(min_bounds, max_bounds, interpolation_value);
	}
}