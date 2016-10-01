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

#ifndef CODE_TRUNK_SDL2_SDLINPUTMANAGER_HPP_
#define CODE_TRUNK_SDL2_SDLINPUTMANAGER_HPP_

#include <unordered_map>
#include <memory>

#include <SDL2/SDL.h>

#include <APG/SXXDL.hpp>
#include "InputManager.hpp"

namespace y3e {
class Emulator;

using controller_ptr = std::unique_ptr<SDL_GameController, void(*)(SDL_GameController *)>;
controller_ptr make_controller_ptr(SDL_GameController *);

/**
 * Uses SDL2 to receive input from the system and relays this information back to the emulator.
 *
 * Supports controller input (tested with XBox 360 controller) and keyboard.
 */
class SDLInputManager final : public InputManager {
public:
	explicit SDLInputManager(Emulator * emulator_, bool useGamepads = true);
	virtual ~SDLInputManager();

	void handleSDLEvent(const SDL_Event &e);

	virtual int shouldSaveState() override final;
	virtual int shouldLoadState() override final;

	virtual bool shouldQuit() override final;

private:
	bool isStateKey(const SDL_Scancode &scancode);

	/**
	 * Doesn't guarantee a valid int if the scancode is not a valid number key.
	 */
	int stateFromKey(const SDL_Scancode &scancode);

	void setBasedOnEvent(GBButton button, const SDL_Event &e);
	void handleKeyEvent(const SDL_Event &e);
	void handleControllerButtonEvent(const SDL_Event &e);
	void handleControllerDeviceEvent(const SDL_Event &e);

	void pollControllers();

	void initSDLButtonMap();
	std::unordered_map<int, GBButton> sdlButtonMap;
	std::unordered_map<int, GBButton> sdlControllerButtonMap;

	int shouldSaveState_ = -1;
	int shouldLoadState_ = -1;
	bool useControllers = true;

	SDL_GameController * ownedController = nullptr;

	bool shouldQuit_ = false;

	/*
	 * These should be true on all platforms, but asserting it now allows us to assume
	 * more about the memory layout and turn what would otherwise be a map lookup into
	 * a simple subtraction which is a decent saving of cycles
	 */
	static_assert(SDL_SCANCODE_1 + 1 == SDL_SCANCODE_2, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 2 == SDL_SCANCODE_3, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 3 == SDL_SCANCODE_4, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 4 == SDL_SCANCODE_5, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 5 == SDL_SCANCODE_6, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 6 == SDL_SCANCODE_7, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 7 == SDL_SCANCODE_8, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 8 == SDL_SCANCODE_9, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
	static_assert(SDL_SCANCODE_1 + 9 == SDL_SCANCODE_0, "SDLInputManager assumes that SDL_SCANCODES are contiguous for values 1 through to 0.");
};

}

#endif /* CODE_TRUNK_SDL2_SDLINPUTMANAGER_HPP_ */
