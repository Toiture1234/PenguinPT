// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <SFML/Graphics.hpp>

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/user/gui/gui_utility.h>

#define SLIDER_BASE_COLOR {31, 31, 31}
#define SLIDER_OUTLINE_COLOR {61, 61, 61}
#define SLIDER_TOGGLE_COLOR {91, 91, 91}
#define SLIDER_CHARACTER_SIZE 15U
#define SLIDER_PERCENTAGE 0.1f

namespace penguinPT::GUI {
	template<typename T>
	class sliderBase {
	public:
		sliderBase(
			std::string n, T* ref,
			sf::Vector2f p, 
			sf::Vector2f s,
			T mi,
			T ma
		) : m_name(n), m_ref(ref), m_min(mi), m_max(ma)
		{
			m_rect.setPosition(p);
			m_rect.setSize(s);

			m_rect.setFillColor(SLIDER_BASE_COLOR);
			m_rect.setOutlineThickness(-1);
			m_rect.setOutlineColor(SLIDER_OUTLINE_COLOR);

			m_slider_rect.setSize({ s.x * SLIDER_PERCENTAGE, s.y });
			m_slider_rect.setOrigin(m_slider_rect.getSize().x * 0.5f, 0.f);
			m_slider_rect.setFillColor(SLIDER_OUTLINE_COLOR);

			m_slider_rect.setPosition({ m_rect.getPosition().x + m_rect.getSize().x * getInterpolation(), m_rect.getPosition().y });
		}
		sliderBase(
			std::string n, 
			std::string txt, 
			sf::Font* font_ref, 
			T* ref, 
			sf::Vector2f p, 
			sf::Vector2f s,
			T mi,
			T ma
		) : m_name(n), m_ref(ref), m_min(mi), m_max(ma)
		{
			m_rect.setPosition(p);
			m_rect.setSize(s);

			m_rect.setFillColor(SLIDER_BASE_COLOR);
			m_rect.setOutlineThickness(1);
			m_rect.setOutlineColor(SLIDER_OUTLINE_COLOR);

			m_slider_rect.setSize({ s.x * SLIDER_PERCENTAGE, s.y });
			m_slider_rect.setOrigin(m_slider_rect.getSize().x * 0.5f, 0.f);
			m_slider_rect.setFillColor(SLIDER_OUTLINE_COLOR);

			m_text.setFont(*font_ref);
			m_text.setString(txt);
			m_text.setCharacterSize(SLIDER_CHARACTER_SIZE);
			centerText();

			m_slider_rect.setPosition({ m_rect.getPosition().x + m_rect.getSize().x * getInterpolation(), m_rect.getPosition().y });
		}
		~sliderBase() {}

		void update(sf::Vector2f mouse_pos);
		void draw(sf::RenderTexture* target, sf::Vector2f mouse_pos);
		void draw(sf::RenderWindow* target, sf::Vector2f mouse_pos);
		
		bool isDragged() const { return m_dragging; };
	private:
		T* m_ref;
		T m_min, m_max;
		sf::RectangleShape m_rect; 
		sf::RectangleShape m_slider_rect;
		sf::Text m_text;
		std::string m_name;

		bool m_dragging = false;

		void centerText();

		// assume m_ref is between m_min and m_max (m_min < m_max)
		// Scales the interval given the percentage of slider
		float getInterpolation();
	};
}

template<typename T> void penguinPT::GUI::sliderBase<T>::draw(sf::RenderTexture* target, sf::Vector2f mouse_pos) {
	target->draw(m_rect);
	target->draw(m_slider_rect);
	target->draw(m_text);
}
template<typename T> void penguinPT::GUI::sliderBase<T>::draw(sf::RenderWindow* target, sf::Vector2f mouse_pos) {
	target->draw(m_rect);
	target->draw(m_slider_rect);
	target->draw(m_text);
}

template<typename T> void penguinPT::GUI::sliderBase<T>::centerText() {
	m_text.setOrigin(m_text.getLocalBounds().getSize() * 0.5f);
	m_text.setPosition({ m_rect.getPosition().x + m_rect.getSize().x * 0.5f, m_rect.getPosition().y + m_rect.getSize().y * 0.33f });
}

template<typename T> float penguinPT::GUI::sliderBase<T>::getInterpolation() {
	return (*m_ref - m_min) / (m_max - m_min);
}

template<typename T> void penguinPT::GUI::sliderBase<T>::update(sf::Vector2f mouse_pos) {
	float mouse_interpolation = (mouse_pos.x - (m_rect.getPosition().x + SLIDER_PERCENTAGE * 0.5f * m_rect.getSize().x)) / (m_rect.getSize().x * (1.f - SLIDER_PERCENTAGE));
	mouse_interpolation = CLAMP(mouse_interpolation, 0.f, 1.f);
	if (util::isMouseOnRect(mouse_pos, m_rect))
	{
		if (util::isMouseOnRect(mouse_pos + sf::Vector2f(m_slider_rect.getSize().x * 0.5f, 0.f), m_slider_rect)) {
			m_slider_rect.setFillColor(SLIDER_TOGGLE_COLOR);
			if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) m_dragging = true;
		}
		else {
			m_slider_rect.setFillColor(SLIDER_OUTLINE_COLOR);
			if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) *m_ref = m_min + (m_max - m_min) * mouse_interpolation;
		}
	} else if(!m_dragging){
		m_slider_rect.setFillColor(SLIDER_OUTLINE_COLOR);
	}
	
	if (m_dragging) {
		*m_ref = m_min + (m_max - m_min) * mouse_interpolation;

		if (!sf::Mouse::isButtonPressed(sf::Mouse::Left)) m_dragging = false;
	}
	
	*m_ref = CLAMP(*m_ref, m_min, m_max);
	m_slider_rect.setPosition({ m_rect.getPosition().x + m_rect.getSize().x * (1.f - SLIDER_PERCENTAGE) * getInterpolation() + m_rect.getSize().x * SLIDER_PERCENTAGE * 0.5f, m_rect.getPosition().y});
}