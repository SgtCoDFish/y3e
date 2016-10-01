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

#ifndef CODE_TRUNK_SDL2_SDLRENDERER_HPP_
#define CODE_TRUNK_SDL2_SDLRENDERER_HPP_

#include <cstdint>

#include <SDL2/SDL.h>

#include <glm/glm.hpp>

#include "APG/SXXDL.hpp"

#include "Renderer.hpp"

namespace y3e {
class Emulator;

/**
 * Uses SDL2 to fill a texture with the LCD screen's contents.
 * The texture is uploaded to graphics memory during emulator vblank.
 */
class SDLRenderer final : public Renderer {
public:
	explicit SDLRenderer(Emulator * emulator_, const SXXDL::window_ptr &window);
	virtual ~SDLRenderer() = default;

	virtual void swap() override final;

	virtual void renderSprites() override final;
	virtual void renderBackgroundLine() override final;

	SXXDL::renderer_ptr renderer { SXXDL::make_renderer_ptr(nullptr) };

	static uint32_t glmToSDLColor(const glm::ivec4 &col);

private:
	SXXDL::sdl_texture_ptr texture { SXXDL::make_sdl_texture_ptr(nullptr) };

	void updateTexture();

	void rendererDebug();

	bool updateBG_ = false;
	bool updateTiles1_ = false;
	bool updateTiles2_ = false;

	constexpr static const int screenWidth = 160;
	constexpr static const int screenHeight = 144;

	constexpr static const int screenPitch = screenWidth * sizeof(uint32_t);

	std::array<uint8_t, screenHeight * screenPitch> screenData;
};

}

#endif /* CODE_TRUNK_SDL2_SDLRENDERER_HPP_ */
