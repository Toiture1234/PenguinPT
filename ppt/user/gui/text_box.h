// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <SFML/Graphics.hpp>

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/user/gui/gui_utility.h>

#define TEXTBOX_BASE_COLOR {31, 31, 31}
#define TEXTBOX_OUTLINE_COLOR {61, 61, 61}
#define TEXTBOX_CHARACTER_SIZE 15U

namespace penguinPT::GUI {
	class TextBox {
	public:
		TextBox(sf::Vector2f pos, sf::Vector2f size, std::string txt, sf::Font* font_ref, bool has_body = false) : m_has_body(has_body)
		{
			m_body.setPosition(pos), m_body.setSize(size);
			if (has_body) {
				m_body.setFillColor(TEXTBOX_BASE_COLOR), m_body.setOutlineThickness(-1), m_body.setOutlineColor(TEXTBOX_OUTLINE_COLOR);
			}
			m_text.setFont(*font_ref);
			m_text.setCharacterSize(TEXTBOX_CHARACTER_SIZE);
			m_text.setString(txt);
			centerText();
		}

		void draw(sf::RenderWindow* target);
	private:
		sf::RectangleShape m_body;
		bool m_has_body;

		sf::Text m_text;

		void centerText();
	};
}
void penguinPT::GUI::TextBox::draw(sf::RenderWindow* target) {
	if (m_has_body) target->draw(m_body);
	target->draw(m_text);
}
void penguinPT::GUI::TextBox::centerText() {
	m_text.setOrigin(m_text.getGlobalBounds().getSize() * 0.5f);
	m_text.setPosition({ m_body.getPosition().x + m_body.getSize().x * 0.5f, m_body.getPosition().y + m_body.getSize().y * 0.33f });
}