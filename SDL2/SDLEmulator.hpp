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

#ifndef CODE_TRUNK_INCLUDE_SDLEMULATOR_HPP_
#define CODE_TRUNK_INCLUDE_SDLEMULATOR_HPP_

#include <SDL2/SDL.h>

#include "APG/SXXDL.hpp"

#include "Emulator.hpp"

namespace y3e {

/**
 * An SDL2 implementation of the emulator. Uses SDL2 versions of input, audio, rendering and
 * platform tools to emulate games. Also implements a "model reference implementation" of how
 * to extend Y3E to add your own backend.
 */
class SDLEmulator final : public Emulator {
public:
	explicit SDLEmulator(bool audioEnabled = true, uint16_t windowScale_ = 4u);
	virtual ~SDLEmulator() = default;

	virtual const char *getPlatformName() const override {
		return "SDL2";
	}

	virtual void frameBegin() override;
	virtual void frameEnd() override;

	virtual void signalVBlank() override;

	SDL_Renderer * getSDLRenderer() const;

	void disableSDLAudio();

private:
	const uint16_t windowScale;

	SXXDL::window_ptr window { SXXDL::make_window_ptr(nullptr) };

	uint16_t windowWidth_ = 160;
	uint16_t windowHeight_ = 144;
};

}

#endif /* CODE_TRUNK_INCLUDE_SDLEMULATOR_HPP_ */