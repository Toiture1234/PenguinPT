#pragma once

namespace penguinPT {
	class camera {
	public:
		nanovdb::Vec3f position = nanovdb::Vec3f(0.f);
		nanovdb::Vec3f angles = nanovdb::Vec3f(0.f);

		float zoom = 1.0f; // TODO : replace with proper fov
        float speed = 1.0f;

		__hostdev__ camera() {}
        __hostdev__ ~camera() {}

		void move(sf::RenderWindow* window, float delta_t, unsigned int* frame_index);
	};
}


void penguinPT::camera::move(sf::RenderWindow* window, float delta_t, unsigned int* frame_index) {
    // camera movement
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z)) {
        *frame_index = 0;
        position[2] += -cosf(angles[0]) * delta_t * speed;
        position[0] += sinf(angles[0]) * delta_t * speed;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
        *frame_index = 0;
        position[2] -= -cosf(angles[0]) * delta_t * speed;
        position[0] -= sinf(angles[0]) * delta_t * speed;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
        *frame_index = 0;
        position[2] -= sinf(angles[0]) * delta_t * speed;
        position[0] -= cosf(angles[0]) * delta_t * speed;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
        *frame_index = 0;
        position[2] += sinf(angles[0]) * delta_t * speed;
        position[0] += cosf(angles[0]) * delta_t  * speed;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) {
        position[1] -= 1. * delta_t * speed;
        *frame_index = 0;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        position[1] += 1. * delta_t * speed;
        *frame_index = 0;
    }

    // view changing
    sf::Vector2f mousePosition = window->mapPixelToCoords(sf::Mouse::getPosition(*window));
    float angleX = (mousePosition.x - window->getSize().x * 0.5) / 180.;
    float angleY = (window->getSize().y * 0.5 - mousePosition.y) / 180.;
    if (sf::Mouse::isButtonPressed(sf::Mouse::Middle)) {
        angles = { angleX, angleY, 0. };
        *frame_index = 0;
       
    } 
}