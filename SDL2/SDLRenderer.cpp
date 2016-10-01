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

#include "SDLRenderer.hpp"
#include "Emulator.hpp"

namespace y3e {

SDLRenderer::SDLRenderer(Emulator * emulator, const SXXDL::window_ptr &window) :
		        Renderer(emulator) {
	renderer = SXXDL::make_renderer_ptr(SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED));

	if (renderer == nullptr) {
		std::cout << "Couldn't create renderer (hardware acceleration might not be available):\n" << SDL_GetError()
		        << std::endl;
		return;
	}

	rendererDebug();

	texture = SXXDL::make_sdl_texture_ptr(
	        SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, screenWidth,
	                screenHeight));
}

void SDLRenderer::renderSprites() {
	if (emulator_->gpu.isSpriteRenderingEnabled()) {
		const auto spriteMap = emulator_->gpu.getSpriteMap();

		for (const auto &pair : spriteMap) {
			const auto &sprite = pair.second;
			const auto &palette =
			        sprite.usesPalette1() ?
			                emulator_->paletteManager.getOBJ1Palette() : emulator_->paletteManager.getOBJ0Palette();

			const int16_t spriteX = sprite.xPos - 8;
			const int16_t spriteY = sprite.yPos - 16;

			const bool regularSprite = emulator_->gpu.getSpriteSizeMode() == SpriteSizeMode::SIZE_8_8;

			// 8x16 sprites ignore the LSB of the pattern number
			const auto patternNumber = regularSprite ? sprite.patternNumber : (sprite.patternNumber & ((~1) & 0xFF));

			const auto spriteCols = 7;
			const auto spriteRows = regularSprite ? 7 : 15;

			const auto tile = emulator_->gpu.getTileFromTable0(patternNumber);

			// tile2 used only for 8x16 mode.
			const auto tile2 = emulator_->gpu.getTileFromTable0(patternNumber + 1);

			for (int pixelY = 0; pixelY <= spriteRows; pixelY++) {
				for (int pixelX = 0; pixelX <= spriteCols; pixelX++) {
					const int16_t targetX = spriteX + pixelX;
					const int16_t targetY = spriteY + pixelY;

					if (targetX <= 0 || targetX >= 160) {
						continue;
					}

					const bool yFlip = sprite.yFlip();

					const int actualX = sprite.xFlip() ? spriteCols - pixelX : pixelX;
					const int actualY = yFlip ? spriteRows - pixelY : pixelY;

					const uint32_t memOffset = targetY * screenPitch + targetX * sizeof(uint32_t);

					if (memOffset + 3u >= screenData.size()) {
						continue;
					}

					uint8_t pixelIndex;

					if (yFlip && !regularSprite) {
						if (pixelY <= 7) {
							pixelIndex = tile2.getPixel(actualX, actualY - 8);
						} else {
							pixelIndex = tile.getPixel(actualX, actualY);
						}
					} else {
						if (pixelY <= 7) {
							pixelIndex = tile.getPixel(actualX, actualY);
						} else {
							pixelIndex = tile2.getPixel(actualX, actualY - 8);
						}
					}

					auto col = palette->getColor(pixelIndex);

					if (sprite.hasPriority()) {
						const auto bgCol = emulator_->paletteManager.getBGPalette()->getColor(0);

						if (screenData[memOffset + 0] != bgCol.b && //
						        screenData[memOffset + 1] != bgCol.g && //
						        screenData[memOffset + 2] != bgCol.r) {
							continue;
						}
					}

					if (!sprite.hasPriority() && pixelIndex == 0) {
						continue;
					}

					screenData[memOffset + 0] = col.b;
					screenData[memOffset + 1] = col.g;
					screenData[memOffset + 2] = col.r;
					screenData[memOffset + 3] = col.a;
				}

			}
		}
	}
}

void SDLRenderer::renderBackgroundLine() {
// get scanline from memory register
	const auto scanline = emulator_->memory.read(0xFF44);

	const auto scrollY = emulator_->memory.read(0xFF42);
	const auto scrollX = emulator_->memory.read(0xFF43);

	const int wx = emulator_->memory.read(0xFF4B);
	const int wy = emulator_->memory.read(0xFF4A);
	const auto actualWindowY = scanline - wy;

	bool window = emulator_->gpu.isWindowEnabled();

	if (wx > 166 || wy > 143 || wy > scanline) {
		window = false;
	}

	const auto screenYPos = (scanline + scrollY) % 256;

// Get the row that the tile is on
	const uint8_t tileRow = screenYPos / 8;
	const int windowTileRow = (actualWindowY / 8);

	const auto &palette = emulator_->paletteManager.getBGPalette();

// 160 horizontal pixels on GB screen
	for (int i = 0; i < 160; ++i) {
		const int actualWindowX = i - wx + 7;

		glm::ivec4 col;

		if (!window || actualWindowX < 0 || actualWindowX > 160) {
			const auto screenXPos = (scrollX + i) % 256;

			const uint8_t tileCol = (screenXPos / 8);

			// multiply by 32 tiles per row
			const uint16_t tileAddr = emulator_->gpu.getBGTileMapBaseLocation() + tileRow * 32 + tileCol;
			const uint8_t tileIndex = emulator_->memory.read(tileAddr);

			const auto tile = emulator_->gpu.getTileFromCurrentTable(tileIndex);
			const auto colIndex = tile.getPixel(screenXPos % 8, screenYPos % 8);

			col = palette->getColor(colIndex);
		} else if(emulator_->gpu.isBackgroundEnabled()) {
			const int windowTileCol = (actualWindowX / 8);

			const auto tileOffset = 32 * windowTileRow + windowTileCol;
			const int tileNumber = emulator_->memory.read(emulator_->gpu.getWindowTileMapBaseLocation() + tileOffset);

			const auto tile = emulator_->gpu.getTileFromCurrentTable(tileNumber);

			const auto colIndex = tile.getPixel(actualWindowX % 8, actualWindowY % 8);

			col = emulator_->paletteManager.getBGPalette()->getColor(colIndex);
		} else {
			col = palette->getColor(0);
		}

		const auto memOffset = scanline * screenPitch + (i) * sizeof(uint32_t);

		screenData[(memOffset + 0) % screenData.size()] = col.b;
		screenData[(memOffset + 1) % screenData.size()] = col.g;
		screenData[(memOffset + 2) % screenData.size()] = col.r;
		screenData[(memOffset + 3) % screenData.size()] = col.a;
	}
}

void SDLRenderer::swap() {
//	std::cout << "swap\n";
	const auto &palette = emulator_->paletteManager.getBGPalette();
	const glm::ivec4 &bg = (emulator_->gpu.isLCDOn() ? palette->getColor(0) : palette->getColor(2));

	updateTexture();

	SDL_SetRenderDrawColor(renderer.get(), bg.r, bg.g, bg.b, bg.a);
	SDL_RenderClear(renderer.get());

	if (emulator_->gpu.isLCDOn()) {
		SDL_RenderCopy(renderer.get(), texture.get(), nullptr, nullptr);
	}

	SDL_RenderPresent(renderer.get());
}

void SDLRenderer::updateTexture() {
	int pitch;
	uint8_t *pixels;
	SDL_LockTexture(texture.get(), nullptr, (void**) &pixels, &pitch);

	std::copy(screenData.begin(), screenData.end(), pixels);

	SDL_UnlockTexture(texture.get());
}

uint32_t SDLRenderer::glmToSDLColor(const glm::ivec4 &col) {
	uint32_t ret = 0;

	ret += (col.b & 0xFF) << 24;
	ret += (col.g & 0xFF) << 16;
	ret += (col.r & 0xFF) << 8;
	ret += (col.a & 0xFF) << 0;

	return ret;
}

void SDLRenderer::rendererDebug() {
	SDL_RendererInfo rinfo;
	SDL_GetRendererInfo(renderer.get(), &rinfo);
	std::cout << "Renderer info:\n";
	std::cout << "\tName: " << rinfo.name << std::endl;
	std::cout << "\tAvailable texture formats: " << rinfo.num_texture_formats << std::endl;

//	for (uint8_t i = 0; i < rinfo.num_texture_formats; i++) {
//		std::cout << "\t\t" << SDL_GetPixelFormatName(rinfo.texture_formats[i]) << std::endl;
//	}
}

}
