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

#include <cstdlib>

#include <iostream>
#include <thread>

#include <SDL2/SDL.h>

#include "SDLEmulator.hpp"
#include "SDLInputManager.hpp"
#include "SDLRenderer.hpp"
#include "SDLAudioHandler.hpp"
#include "SDLPlatformTools.hpp"

namespace y3e {

SDLEmulator::SDLEmulator(bool audioEnabled, uint16_t windowScale_) :
		        Emulator(),
		        windowScale { windowScale_ } {
	windowWidth_ *= windowScale;
	windowHeight_ *= windowScale_;
	input = std::make_unique<SDLInputManager>(this);

	window = SXXDL::make_window_ptr(SDL_CreateWindow("Y3E - SDL2", //
	        SDL_WINDOWPOS_UNDEFINED, //
	        SDL_WINDOWPOS_UNDEFINED, //
	        windowWidth_, //
	        windowHeight_, //
	        SDL_WINDOW_SHOWN));

	if (window == nullptr) {
		std::cout << "Couldn't create SDL2 window: " << SDL_GetError() << std::endl;
		return;
	}

	renderer = std::make_unique<SDLRenderer>(this, window);
	serialHandler = std::make_unique<SerialHandler>(this);
	platformTools = std::make_unique<SDLPlatformTools>(this);

	if (audioEnabled) {
		audio = std::make_unique<SDLAudioHandler>(this);

		if (((SDLAudioHandler *) audio.get())->hasSDLAudioError()) {
			disableSDLAudio();
		}
	} else {
		disableSDLAudio();
	}
}

void SDLEmulator::frameBegin() {
}

void SDLEmulator::signalVBlank() {
	static SDL_Event e;

	while (SDL_PollEvent(&e)) {
		if (e.type == SDL_QUIT) {
			signalQuit();
		} else if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP || e.type == SDL_CONTROLLERBUTTONDOWN
		        || e.type == SDL_CONTROLLERBUTTONUP || e.type == SDL_CONTROLLERDEVICEADDED
		        || e.type == SDL_CONTROLLERDEVICEREMOVED) {
			static_cast<SDLInputManager*>(input.get())->handleSDLEvent(e);
		}
	}

	Emulator::signalVBlank();
}

void SDLEmulator::frameEnd() {
}

SDL_Renderer * SDLEmulator::getSDLRenderer() const {
	return ((SDLRenderer *) renderer.get())->renderer.get();
}

void SDLEmulator::disableSDLAudio() {
	audio = std::make_unique<AudioHandler>(this);
}

}

