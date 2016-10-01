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

#include <iostream>

#include "SDLInputManager.hpp"
#include "Emulator.hpp"

namespace y3e {

SDLInputManager::SDLInputManager(Emulator * emulator_, bool useControllers_) :
		        InputManager(emulator_),
		        useControllers { useControllers_ } {
	initSDLButtonMap();

	if (useControllers) {
		pollControllers();
	}
}

SDLInputManager::~SDLInputManager() {
	if (ownedController != nullptr) {
//		SDL_GameControllerClose(ownedController);
		// This probably should be called but seems to be bugged under linux at least...
	}
}

void SDLInputManager::handleSDLEvent(const SDL_Event &e) {
	if (e.type == SDL_CONTROLLERBUTTONDOWN || e.type == SDL_CONTROLLERBUTTONUP) {
		handleControllerButtonEvent(e);
	} else if (e.type == SDL_CONTROLLERDEVICEADDED || e.type == SDL_CONTROLLERDEVICEREMOVED) {
		handleControllerDeviceEvent(e);
	} else {
		handleKeyEvent(e);
	}
}

void SDLInputManager::handleKeyEvent(const SDL_Event &e) {
	const auto &scancode = e.key.keysym.scancode;

	if (e.key.keysym.scancode == SDL_SCANCODE_F1) {
		emulator->setFrameLimitRate(1);
	} else if (e.key.keysym.scancode == SDL_SCANCODE_F2) {
		emulator->setFrameLimitRate(2);
	} else if (e.key.keysym.scancode == SDL_SCANCODE_F3) {
		emulator->setFrameLimitRate(3);
	} else if (e.key.keysym.scancode == SDL_SCANCODE_F4) {
		emulator->setFrameLimitRate(4);
	}

	if (scancode == SDL_SCANCODE_P && e.key.keysym.mod & KMOD_LCTRL && e.type == SDL_KEYDOWN) {
		emulator->batteryHandler.update(true);
	}

	if (isStateKey(scancode)) {
		if (e.type == SDL_KEYDOWN) {
			if (e.key.keysym.mod & KMOD_LCTRL) {
				// save state request
				shouldSaveState_ = stateFromKey(scancode);
				shouldLoadState_ = -1;
				return;
			} else if (e.key.keysym.mod & KMOD_LALT) {
				// load state request
				shouldLoadState_ = stateFromKey(scancode);
				shouldSaveState_ = -1;
				return;
			}
		} else {
			shouldLoadState_ = -1;
			shouldSaveState_ = -1;
		}
	}

	if (e.type == SDL_KEYDOWN) {
		if (scancode == SDL_SCANCODE_F12) {
			emulator->enableDebug();
			return;
		} /*else if (scancode == SDL_SCANCODE_F11) {
		 emulator->setDebugPrintingForAllInstructions(true);
		 return;
		 }*/else if (scancode == SDL_SCANCODE_EQUALS) {
			emulator->audio->volumeUp();
		} else if (scancode == SDL_SCANCODE_MINUS) {
			emulator->audio->volumeDown();
		}
	}

	if (scancode == SDL_SCANCODE_ESCAPE) {
		shouldQuit_ = (e.type == SDL_KEYDOWN);

		if (shouldQuit_) {
			emulator->signalQuitRequested();
		}
	}

	if (sdlButtonMap.find(scancode) != sdlButtonMap.end()) {
		setBasedOnEvent(sdlButtonMap[scancode], e);
	}
}

void SDLInputManager::handleControllerButtonEvent(const SDL_Event &e) {
	const auto button = e.cbutton.button;

	if (sdlControllerButtonMap.find(button) != sdlControllerButtonMap.end()) {
		setBasedOnEvent(sdlControllerButtonMap[button], e);
	}
}

void SDLInputManager::handleControllerDeviceEvent(const SDL_Event &e) {
	pollControllers();
}

void SDLInputManager::setBasedOnEvent(GBButton button, const SDL_Event &e) {
	if (e.type == SDL_KEYDOWN || e.type == SDL_CONTROLLERBUTTONDOWN) {
		setPressed(button);
	} else if (e.type == SDL_KEYUP || e.type == SDL_CONTROLLERBUTTONUP) {
		setNotPressed(button);
	} else {
		std::cout << "WARNING: Invalid event got to setBasedOnEvent for SDLInputManager\n";
		setNotPressed(button);
	}

	emulator->interruptHandler.triggerInterrupt(InterruptType::PIN_TRANSITION);
}

void SDLInputManager::initSDLButtonMap() {
	sdlButtonMap[SDL_SCANCODE_Z] = GBButton::A;
	sdlButtonMap[SDL_SCANCODE_X] = GBButton::B;

	sdlButtonMap[SDL_SCANCODE_W] = GBButton::UP;
	sdlButtonMap[SDL_SCANCODE_S] = GBButton::DOWN;
	sdlButtonMap[SDL_SCANCODE_A] = GBButton::LEFT;
	sdlButtonMap[SDL_SCANCODE_D] = GBButton::RIGHT;

	sdlButtonMap[SDL_SCANCODE_UP] = GBButton::UP;
	sdlButtonMap[SDL_SCANCODE_DOWN] = GBButton::DOWN;
	sdlButtonMap[SDL_SCANCODE_LEFT] = GBButton::LEFT;
	sdlButtonMap[SDL_SCANCODE_RIGHT] = GBButton::RIGHT;

	sdlButtonMap[SDL_SCANCODE_RETURN] = GBButton::START;
	sdlButtonMap[SDL_SCANCODE_BACKSPACE] = GBButton::SELECT;

	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_A] = GBButton::A;
	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_B] = GBButton::B;

	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_DPAD_UP] = GBButton::UP;
	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_DPAD_DOWN] = GBButton::DOWN;
	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_DPAD_LEFT] = GBButton::LEFT;
	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = GBButton::RIGHT;

	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_START] = GBButton::START;
	sdlControllerButtonMap[SDL_CONTROLLER_BUTTON_BACK] = GBButton::SELECT;
}

int SDLInputManager::shouldSaveState() {
	return shouldSaveState_;
}

int SDLInputManager::shouldLoadState() {
	return shouldLoadState_;
}

bool SDLInputManager::isStateKey(const SDL_Scancode &scancode) {
	return (scancode >= SDL_SCANCODE_1 && scancode <= SDL_SCANCODE_0);
}

int SDLInputManager::stateFromKey(const SDL_Scancode &scancode) {
	return scancode - SDL_SCANCODE_1;
}

void SDLInputManager::pollControllers() {
	std::cout << "Polling " << SDL_NumJoysticks() << " possible controllers.\n";
	for (int i = 0; i < SDL_NumJoysticks(); ++i) {
		if (SDL_IsGameController(i)) {
			SDL_GameController * controller = SDL_GameControllerOpen(i);

			if (controller != nullptr) {
				std::cout << "Game controller found: " << SDL_GameControllerName(controller) << std::endl;
				ownedController = controller;
				break;
			}
		}
	}
}

controller_ptr make_controller_ptr(SDL_GameController *controller) {
	return controller_ptr(controller, SDL_GameControllerClose);
}

bool SDLInputManager::shouldQuit() {
	return shouldQuit_;
}

}
