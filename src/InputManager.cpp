/*
 * Copyright (c) 2015, 2016 See AUTHORS file.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <unordered_map>

#include "InputManager.hpp"
#include "Emulator.hpp"

namespace y3e {

InputManager::InputManager(Emulator * emulator_) :
		        emulator { emulator_ } {
	initButtonsMap();
}

uint8_t InputManager::getRail4Byte() {
	// BIT3 - START pressed
	// BIT2 - SELECT pressed
	// BIT1 - B pressed
	// BIT0 - A pressed

	uint8_t value = 0b11011111;

	value -= (isPressed(GBButton::START) << 3);
	value -= (isPressed(GBButton::SELECT) << 2);
	value -= (isPressed(GBButton::B) << 1);
	value -= (isPressed(GBButton::A) << 0);

	return value;
}

uint8_t InputManager::getRail5Byte() {
	// BIT3 - DOWN pressed
	// BIT2 - UP pressed
	// BIT1 - LEFT pressed
	// BIT0 - RIGHT pressed

	uint8_t value = 0b11101111;

	value -= (isPressed(GBButton::DOWN) << 3);
	value -= (isPressed(GBButton::UP) << 2);
	value -= (isPressed(GBButton::LEFT) << 1);
	value -= (isPressed(GBButton::RIGHT) << 0);

	return value;
}

bool InputManager::anyPressed() {
	return (buttons.find(true) != buttons.end());
}

bool InputManager::isPressed(GBButton button) {
	return buttons[static_cast<std::underlying_type<GBButton>::type>(button)];
}

void InputManager::setPressed(GBButton button) {
	buttons[static_cast<std::underlying_type<GBButton>::type>(button)] = true;
}

void InputManager::setNotPressed(GBButton button) {
	buttons[static_cast<std::underlying_type<GBButton>::type>(button)] = false;
}


void InputManager::initButtonsMap() {
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::A)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::B)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::UP)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::DOWN)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::LEFT)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::RIGHT)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::START)] = false;
	buttons[static_cast<std::underlying_type<GBButton>::type>(GBButton::SELECT)] = false;
}

}
