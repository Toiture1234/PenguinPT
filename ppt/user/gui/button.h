// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <SFML/Graphics.hpp>

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/user/gui/gui_utility.h>

#define BUTTON_BASE_COLOR {31, 31, 31}
#define BUTTON_OUTLINE_COLOR {61, 61, 61}
#define BUTTON_TOGGLE_COLOR {91, 91, 91}
#define BUTTON_CHARACTER_SIZE 15U

#define CHECKBOX_COOLDOWN 250 // in milliseconds
#define BUTTON_COOLDOWN 250

namespace penguinPT::GUI {
	class ButtonTextual {
	public:
		ButtonTextual(
			std::string n,
			std::string txt,
			sf::Vector2f pos,
			sf::Vector2f size,
			sf::Font* font_ref)
		{
			m_name = n;
			m_body.setPosition(pos);
			m_body.setSize(size);

			m_text.setCharacterSize(BUTTON_CHARACTER_SIZE);
			m_text.setFont(*font_ref);
			m_text.setString(txt);
			centerText();
		}

		~ButtonTextual() {}

		bool isPressed(sf::Vector2f mouse_pos);
		void draw(sf::RenderWindow* target, bool can_activate = true);

		std::string getName() const { return m_name; };
	private:
		sf::Text m_text;
		sf::RectangleShape m_body;

		std::chrono::steady_clock::time_point m_last_press;

		std::string m_name;

		void centerText();
	};
	class ButtonTextured {
	public:
		ButtonTextured(
			std::string n,
			sf::Vector2f pos,
			sf::Vector2f size,
			sf::Texture* tex)
		{
			m_name = n;
			m_body.setPosition(pos);
			m_body.setSize(size);
			m_body.setTexture(tex);
		}
		ButtonTextured(
			std::string n,
			sf::Vector2f pos,
			sf::Vector2f size,
			sf::Texture* tex,
			sf::IntRect tex_rect)
		{
			m_name = n;
			m_body.setPosition(pos);
			m_body.setSize(size);
			m_body.setTexture(tex);
			m_body.setTextureRect(tex_rect);
		}
		~ButtonTextured() {}

		bool isPressed(sf::Vector2f mouse_pos);
		void draw(sf::RenderWindow* target, bool can_activate = true);

		std::string getName() const { return m_name; };
	private:

		std::chrono::steady_clock::time_point m_last_press;

		sf::RectangleShape m_body;
		std::string m_name;
	};

	class CheckBox {
	public:
		CheckBox(std::string n, sf::Vector2f pos, sf::Vector2f size, sf::Texture* tex, bool* link) : m_link(link)
		{
			m_name = n;
			m_body.setPosition(pos), m_body.setSize(size);
			m_body.setFillColor(BUTTON_BASE_COLOR), m_body.setOutlineColor(BUTTON_OUTLINE_COLOR), m_body.setOutlineThickness(-1);
			m_checked.setPosition(pos), m_checked.setSize(size);
			m_checked.setTexture(tex);

			m_last_press = std::chrono::steady_clock::now();
		}
		CheckBox(std::string n, sf::Vector2f pos, sf::Vector2f size, sf::Texture* tex, sf::IntRect tex_rect, bool* link) : m_link(link)
		{
			m_name = n;
			m_body.setPosition(pos), m_body.setSize(size);
			m_body.setFillColor(BUTTON_BASE_COLOR), m_body.setOutlineColor(BUTTON_OUTLINE_COLOR), m_body.setOutlineThickness(-1);
			m_checked.setPosition(pos), m_checked.setSize(size);
			m_checked.setTexture(tex);
			m_checked.setTextureRect(tex_rect);

			m_last_press = std::chrono::steady_clock::now();
		}
		~CheckBox() {}

		void draw(sf::RenderWindow* target, bool can_activate = true);
		void update(sf::Vector2f mouse_pos);
	private:
		sf::RectangleShape m_body;
		sf::RectangleShape m_checked;

		bool* m_link;

		std::chrono::steady_clock::time_point m_last_press;
		
		std::string m_name;
	};

	class SelectorBox {
	public:
		SelectorBox(std::string n, std::vector<std::string> states, sf::Vector2f pos, sf::Vector2f size, sf::Font* font_ref) : m_index(0U), m_name(n)
		{
			m_states = states;
			m_body.setPosition(pos), m_body.setSize(size);
			m_body.setFillColor(BUTTON_BASE_COLOR), m_body.setOutlineColor(BUTTON_OUTLINE_COLOR), m_body.setOutlineThickness(-1);

			m_text.setCharacterSize(BUTTON_CHARACTER_SIZE);
			m_text.setFont(*font_ref);
			m_text.setString(states.at(m_index));
			centerText();
		}
		~SelectorBox() {}

		void draw(sf::RenderWindow* target, bool can_activate = true);
		void update(sf::Vector2f mouse_pos);

		std::string getState();

		void setState(std::string state);
		std::string getName() const { return m_name; };
	private:
		std::string m_name;
		sf::Text m_text;
		sf::RectangleShape m_body;

		std::chrono::steady_clock::time_point m_last_press;

		std::vector<std::string> m_states;
		unsigned int m_index;

		void centerText();
	};
}

bool penguinPT::GUI::ButtonTextual::isPressed(sf::Vector2f mouse_pos) {
	std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();

	auto elapsed = this_time - m_last_press;
	if (penguinPT::GUI::util::isMouseOnRect(mouse_pos, m_body) && sf::Mouse::isButtonPressed(sf::Mouse::Left) && elapsed > std::chrono::milliseconds(BUTTON_COOLDOWN)) {
		m_last_press = this_time;
		return true;
	}
	return false;
}
void penguinPT::GUI::ButtonTextual::centerText() {
	m_text.setOrigin(m_text.getGlobalBounds().getSize() * 0.5f);
	m_text.setPosition({ m_body.getPosition().x + m_body.getSize().x * 0.5f, m_body.getPosition().y + m_body.getSize().y * 0.33f });
}
void penguinPT::GUI::ButtonTextual::draw(sf::RenderWindow* target, bool can_activate) {
	sf::Vector2f mouse_pos = target->mapPixelToCoords(sf::Mouse::getPosition(*target));
	if (penguinPT::GUI::util::isMouseOnRect(mouse_pos, m_body) && can_activate) {
		sf::RectangleShape br(m_body.getSize());
		br.setPosition(m_body.getPosition());
		br.setFillColor(BUTTON_OUTLINE_COLOR);
		br.setOutlineThickness(-1);
		br.setOutlineColor(BUTTON_TOGGLE_COLOR);
		target->draw(br);
	}
	target->draw(m_text);
}

bool penguinPT::GUI::ButtonTextured::isPressed(sf::Vector2f mouse_pos) {
	std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();

	auto elapsed = this_time - m_last_press;
	if (penguinPT::GUI::util::isMouseOnRect(mouse_pos, m_body) && sf::Mouse::isButtonPressed(sf::Mouse::Left) && elapsed > std::chrono::milliseconds(BUTTON_COOLDOWN)) {
		m_last_press = this_time;
		return true;
	}
	return false;
}
void penguinPT::GUI::ButtonTextured::draw(sf::RenderWindow* target, bool can_activate) {
	sf::Vector2f mouse_pos = target->mapPixelToCoords(sf::Mouse::getPosition(*target));
	if (penguinPT::GUI::util::isMouseOnRect(mouse_pos, m_body) && can_activate) {
		sf::RectangleShape br(m_body.getSize());
		br.setPosition(m_body.getPosition());
		br.setFillColor(BUTTON_OUTLINE_COLOR);
		br.setOutlineThickness(-1);
		br.setOutlineColor(BUTTON_TOGGLE_COLOR);
		target->draw(br);
	}
	target->draw(m_body);
}

void penguinPT::GUI::CheckBox::draw(sf::RenderWindow* target, bool can_activate) {
	sf::Vector2f mouse_pos = target->mapPixelToCoords(sf::Mouse::getPosition(*target));
	if (penguinPT::GUI::util::isMouseOnRect(mouse_pos, m_body) && can_activate) {
		m_body.setFillColor(BUTTON_OUTLINE_COLOR), m_body.setOutlineColor(BUTTON_TOGGLE_COLOR);
	}
	else {
		m_body.setFillColor(BUTTON_BASE_COLOR), m_body.setOutlineColor(BUTTON_OUTLINE_COLOR);
	}
	target->draw(m_body);
	if (*m_link) target->draw(m_checked);
}
void penguinPT::GUI::CheckBox::update(sf::Vector2f mouse_pos) {
	std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();

	auto elapsed = this_time - m_last_press;
	if (util::isMouseOnRect(mouse_pos, m_body) && sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
		if (elapsed > std::chrono::milliseconds(CHECKBOX_COOLDOWN)) {
			(*m_link) = !(*m_link);
			m_last_press = this_time;
		}
	}
}

void penguinPT::GUI::SelectorBox::centerText() {
	m_text.setOrigin(m_text.getGlobalBounds().getSize() * 0.5f);
	m_text.setPosition({ m_body.getPosition().x + m_body.getSize().x * 0.5f, m_body.getPosition().y + m_body.getSize().y * 0.33f });
}

void penguinPT::GUI::SelectorBox::draw(sf::RenderWindow* target, bool can_activate) {
	sf::Vector2f mouse_pos = target->mapPixelToCoords(sf::Mouse::getPosition(*target));
	if (penguinPT::GUI::util::isMouseOnRect(mouse_pos, m_body) && can_activate) {
		m_body.setFillColor(BUTTON_OUTLINE_COLOR), m_body.setOutlineColor(BUTTON_TOGGLE_COLOR);
	}
	else {
		m_body.setFillColor(BUTTON_BASE_COLOR), m_body.setOutlineColor(BUTTON_OUTLINE_COLOR);
	}
	target->draw(m_body);
	target->draw(m_text);
}
void penguinPT::GUI::SelectorBox::update(sf::Vector2f mouse_pos) {
	std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();

	auto elapsed = this_time - m_last_press;
	if (util::isMouseOnRect(mouse_pos, m_body) && sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
		if (elapsed > std::chrono::milliseconds(CHECKBOX_COOLDOWN)) {
			m_index = (m_index + 1) % m_states.size();
			m_text.setString(m_states.at(m_index));
			centerText();
			m_last_press = this_time;
		}
	}
}
std::string penguinPT::GUI::SelectorBox::getState() {
	return m_states.at(m_index);
}

void penguinPT::GUI::SelectorBox::setState(std::string state) {
	for (unsigned int i = 0; i < m_states.size(); i++) if (m_states.at(i) == state) m_index = i;
	m_text.setString(m_states.at(m_index));
	centerText();
}