#pragma once

namespace penguinPT::GUI {
	namespace Typography {
		enum character_type
		{
			title = 30,
			body = 15,
			comment = 10
		};
	}
	enum slider_type {
		horizontal = 0,
		vertical
	};
	namespace Color {
		enum brightness_level {
			dark = 1,
			medium = 31,
			light = 61,
			extra_light = 91
		};
	}
	namespace GUI_utility {
		void draw_GUI_rect(sf::RenderWindow* target, sf::Vector2f position, sf::Vector2f size, int value = Color::medium) {
			sf::RectangleShape first_rectangle;
			first_rectangle.setPosition(position), first_rectangle.setSize(size);
			first_rectangle.setFillColor(sf::Color(value + 30.f, value + 30.f, value + 30.f));

			target->draw(first_rectangle);

			sf::RectangleShape second_rectangle;
			second_rectangle.setPosition(position + sf::Vector2f(2, 2)), second_rectangle.setSize(size - sf::Vector2f(4, 4));
			second_rectangle.setFillColor(sf::Color(value, value, value));

			target->draw(second_rectangle);	
		}
		void draw_GUI_rect_noBorders(sf::RenderWindow* target, sf::Vector2f position, sf::Vector2f size, int value = Color::medium) {
			sf::RectangleShape first_rectangle;
			first_rectangle.setPosition(position), first_rectangle.setSize(size);
			first_rectangle.setFillColor(sf::Color(value, value, value));

			target->draw(first_rectangle);
		}

		sf::Vector2f get_mouse_inWindow(sf::RenderWindow* window) {
			return window->mapPixelToCoords(sf::Mouse::getPosition(*window));
		}
	}
	class rect_shape {
	public:
		rect_shape(sf::Vector2f pos, sf::Vector2f s, Color::brightness_level b, bool hb) : position(pos), size(s), b_lv(b), has_borders(hb) {}
		~rect_shape() {}

		void draw(sf::RenderWindow* target);
		sf::Vector2f get_size() const { return size; };
	private:
		sf::Vector2f position;
		sf::Vector2f size;

		Color::brightness_level b_lv;
		bool has_borders;
	};

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

		int get_y(Typography::character_type ch_type) {
			switch (ch_type)
			{
			case Typography::comment:
				return 11;
				break;
			case Typography::body:
				return 14;
				break;
			case Typography::title:
				return 29;
				break;
			}
		}
	};

	class button {
	public:
		sf::Vector2f position;

		std::string name;
		Typography::character_type char_type = Typography::body;
		
		Color::brightness_level b_lv = Color::medium;
	public:
		button(std::string n) : name(n) {}
		~button() {}
		button(sf::Vector2f p, std::string n) : position(p), name(n) {}

		bool is_button_pressed(sf::Vector2f mouse_pos) const;
		void draw(sf::RenderWindow* target);

		void set_text_manager(text_manager* src);
		void set_string(std::string str);
		void define_size(sf::Vector2f s);

		sf::Vector2f get_size() const { return size; };
	private:
		bool is_mouse_on_button(sf::Vector2f mouse_pos) const;
		sf::Vector2f size;
		std::string text;

		text_manager* txt_m = nullptr;
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
		slider(std::string n) : name(n) {}
		~slider() {}
		slider(sf::Vector2f p, sf::Vector2f s, std::string n) : position(p), size(s), name(n) {}

		void draw(sf::RenderWindow* target);
		float update_slider(sf::Vector2f mouse_pos);

		sf::Vector2f get_size() const { return size; };
	private:
		bool is_mouse_on_slider(sf::Vector2f mouse_pos) const;
		bool is_mouse_on_sliderslide(sf::Vector2f mouse_pos) const;
		bool toggled = false;

		float last_value_onScreen;
		float last_value_true;
	};

	class text_zone {
	public:
		sf::Vector2f position;
		
		std::string name;
		Typography::character_type char_type = Typography::body;
		text_manager* txt_m = nullptr;

		bool has_b = false;
		Color::brightness_level b_lv = Color::medium;
	public:
		text_zone(std::string n) : name(n) {}
		~text_zone() {}
		text_zone(sf::Vector2f p, std::string n) : position(p), name(n) {}

		void draw(sf::RenderWindow* target);

		void set_string(std::string str);
		void define_size(sf::Vector2f s);

		sf::Vector2f get_size() const { return size; };
	private:
		sf::Vector2f size;
		std::string text;
	};

	class GUI_manager {
	public:
		GUI_manager() {}
		~GUI_manager() {}

		void add_button(button r);
		void add_slider(slider r);
		void add_text_zone(text_zone r);
		void add_rect_shape(rect_shape r);

		void apply_tex_manager(text_manager* src);

		void draw_GUI(sf::RenderWindow* target);

		button& find_button(std::string name);
		slider& find_slider(std::string name);
		text_zone& find_text_zone(std::string name);

		sf::Vector2f on_follow_button(std::string name, char axis = 'X');
		sf::Vector2f on_follow_slider(std::string name, char axis = 'X');
		sf::Vector2f on_follow_text_zone(std::string name, char axis = 'X');
		
	private:
		std::vector<button> button_list;
		std::vector<slider> slider_list;
		std::vector<text_zone> text_zone_list;
		std::vector<rect_shape> rect_shape_list;

		text_manager* txt_ptr;
	};

	/////////////////////////////////////// text functions ///////////////////////////////////////
	void text_manager::draw_text(sf::RenderWindow* target, std::string dsp_str, sf::Vector2f position, int ch_size) {
		text.setString(dsp_str);
		text.setPosition(position);
		text.setCharacterSize(ch_size);

		target->draw(text);
	}
	/////////////////////////////////////// rect_shape functions ///////////////////////////////////////
	void rect_shape::draw(sf::RenderWindow* target) {
		if (has_borders) GUI_utility::draw_GUI_rect(target, position, size, b_lv);
		else GUI_utility::draw_GUI_rect_noBorders(target, position, size, b_lv);
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
		size = sf::Vector2f(r.getSize().x * 1.1f, txt_m->get_y(char_type) * 1.7f);
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
			GUI_utility::draw_GUI_rect(target, position, size, b_lv + 30);
		}
		else {
			GUI_utility::draw_GUI_rect(target, position, size, b_lv);
		}

		if (txt_m != nullptr) txt_m->draw_text(target, text, position + size * 0.05f, char_type);
	}
	// NB : the defined size can't be smaller than the original size given by string
	void button::define_size(sf::Vector2f s) {
		size.x = fmaxf(size.x, s.x), size.y = fmaxf(size.y, s.y);
	}

	/////////////////////////////////////// text zone functions ///////////////////////////////////////
	
	// sets the string which will be displayed on screen
	// this need to be called while every parameter we want is defined if we want the size of the box to be correct
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
		size = sf::Vector2f(r.getSize().x * 1.1f, txt_m->get_y(char_type) * 1.7f);
		
	}
	void text_zone::draw(sf::RenderWindow* target) {
		if(has_b)
			GUI_utility::draw_GUI_rect(target, position, size, b_lv);
		else GUI_utility::draw_GUI_rect_noBorders(target, position, size, b_lv);

		if (txt_m != nullptr) txt_m->draw_text(target, text, position + size * 0.05f, char_type);
	}
	// NB : the defined size can't be smaller than the original size given by string
	void text_zone::define_size(sf::Vector2f s) {
		size.x = fmaxf(size.x, s.x), size.y = fmaxf(size.y, s.y);
	}

	/////////////////////////////////////// slider functions ///////////////////////////////////////
	void slider::draw(sf::RenderWindow* target) {
		GUI_utility::draw_GUI_rect_noBorders(target, position, size);

		// calculate inner rect size
		float slider_size = (size.x * 0.96f) * slider_percentage;
		float pos_adderX = (size.x * 0.96f) * (1.f - slider_percentage) * interpolation_value;
		
		sf::Vector2f mouse_pos = GUI_utility::get_mouse_inWindow(target);
		GUI_utility::draw_GUI_rect_noBorders(target, position + sf::Vector2f(0.02f * size.x + pos_adderX, size.y * 0.25f), sf::Vector2f(slider_size, size.y * 0.5f), is_mouse_on_sliderslide(mouse_pos) || toggled ? 91 : 61);
	}
	bool slider::is_mouse_on_slider(sf::Vector2f mouse_pos) const {
		return mouse_pos.x > position.x && mouse_pos.x < position.x + size.x && mouse_pos.y > position.y && mouse_pos.y < position.y + size.y;
	}
	bool slider::is_mouse_on_sliderslide(sf::Vector2f mouse_pos) const {
		float slider_size = (size.x * 0.96f) * slider_percentage;
		float pos_adderX = (size.x * 0.96f) * (1.f - slider_percentage) * interpolation_value;

		sf::Vector2f position_s = position + sf::Vector2f(0.02f * size.x + pos_adderX, size.y * 0.25f);
		sf::Vector2f size_s = sf::Vector2f(slider_size, size.y * 0.5f);
		return mouse_pos.x > position_s.x && mouse_pos.x < position_s.x + size_s.x && mouse_pos.y > position_s.y && mouse_pos.y < position_s.y + size_s.y;
	}
	float slider::update_slider(sf::Vector2f mouse_pos) {
		float middle = position.x + size.x * 0.5f;
		float slider_size = (size.x * 0.92f) * slider_percentage;

		float diff = mouse_pos.x - middle;
		float range = size.x * 0.92f - slider_size;

		float value_relative = diff / range - last_value_onScreen;
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && is_mouse_on_sliderslide(mouse_pos)) {
			toggled = true;
		}
		else if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && is_mouse_on_slider(mouse_pos)) {
			interpolation_value = diff / range + 0.5f;
		}

		if (toggled) interpolation_value = last_value_true + value_relative;
		else last_value_onScreen = diff / range, last_value_true = interpolation_value;

		interpolation_value = util::clamp(interpolation_value, 0.f, 1.f);
		if (!sf::Mouse::isButtonPressed(sf::Mouse::Left)) toggled = false;
		return util::mix(min_bounds, max_bounds, interpolation_value);
	}

	/////////////////////////////////////// GUI functions ///////////////////////////////////////
	void GUI_manager::apply_tex_manager(text_manager* src) {
		txt_ptr = src;
	}
	void GUI_manager::draw_GUI(sf::RenderWindow* target) {
		for (rect_shape& r : rect_shape_list) r.draw(target);
		for (text_zone& r : text_zone_list) r.draw(target);
		for (button& r : button_list) r.draw(target);
		for (slider& r : slider_list) r.draw(target);
	}

	button& GUI_manager::find_button(std::string name) {
		for (button& r : button_list) if (r.name == name) return r;
	}
	slider& GUI_manager::find_slider(std::string name) {
		for (slider& r : slider_list) if (r.name == name) return r;
	}
	text_zone& GUI_manager::find_text_zone(std::string name) {
		for (text_zone& r : text_zone_list) if (r.name == name) return r;
	}
	void GUI_manager::add_button(button r) {
		button_list.push_back(r);
		button_list.back().set_text_manager(txt_ptr);
	}
	void GUI_manager::add_slider(slider r) {
		slider_list.push_back(r);
	}
	void GUI_manager::add_text_zone(text_zone r) {
		text_zone_list.push_back(r);
		text_zone_list.back().txt_m = txt_ptr;
	}
	void GUI_manager::add_rect_shape(rect_shape r) {
		rect_shape_list.push_back(r);
	}

	sf::Vector2f GUI_manager::on_follow_button(std::string name, char axis) {
		button& r = find_button(name);
		if (axis == 'X') {
			return sf::Vector2f(r.position.x + r.get_size().x, r.position.y);
		}
		else {
			return sf::Vector2f(r.position.x, r.position.y + r.get_size().y);
		}
	}
	sf::Vector2f GUI_manager::on_follow_slider(std::string name, char axis) {
		slider& r = find_slider(name);
		if (axis == 'X') {
			return sf::Vector2f(r.position.x + r.get_size().x, r.position.y);
		}
		else {
			return sf::Vector2f(r.position.x, r.position.y + r.get_size().y);
		}
	}
	sf::Vector2f GUI_manager::on_follow_text_zone(std::string name, char axis) {
		text_zone& r = find_text_zone(name);
		if (axis == 'X') {
			return sf::Vector2f(r.position.x + r.get_size().x, r.position.y);
		}
		else {
			return sf::Vector2f(r.position.x, r.position.y + r.get_size().y);
		}
	}
}