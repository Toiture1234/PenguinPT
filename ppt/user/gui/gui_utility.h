// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <SFML/Graphics.hpp>

namespace penguinPT::GUI::util {
	inline bool isMouseOnRect(sf::Vector2f mouse_pos, sf::RectangleShape rect) {
		return
			mouse_pos.x > rect.getPosition().x && mouse_pos.x < rect.getPosition().x + rect.getSize().x &&
			mouse_pos.y > rect.getPosition().y && mouse_pos.y < rect.getPosition().y + rect.getSize().y;
	}
}