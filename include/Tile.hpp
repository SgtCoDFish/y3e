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

#ifndef CODE_TRUNK_INCLUDE_TILE_HPP_
#define CODE_TRUNK_INCLUDE_TILE_HPP_

#include <cstdint>

#include <array>

namespace y3e {
class MemoryManager;

/**
 * Stores a tile, which is located in memory in a Tile Data Table at either
 * 0x8000 to 0x8FFF or 0x8800 to 0x97FF.
 *
 * Each tile is 16 bytes.
 *
 * TDT 1 (0x8000 - 0x8FFF)
 * -----------------------
 * Numbered with uint8_t (pattern #0 is at 0x8000).
 *
 * Used for sprites, the background and the window display.
 *
 * TDT 2 (0x8800 - 0x97FF)
 * -----------------------
 * Numbered with int8_t (note signed) (pattern #0 is at 0x9000)
 *
 * Used for background and window display only.
 *
 * Layout:
 *
 */
struct Tile {
	static Tile loadFromMemory(MemoryManager * mm, const uint16_t &address);

	explicit Tile() = default;
	~Tile() = default;

	uint8_t getPixel(uint8_t x, uint8_t y) const;
	void setPixel(uint8_t x, uint8_t y, uint8_t value);

	constexpr static const uint8_t TILE_WIDTH = 8;
	constexpr static const uint8_t TILE_HEIGHT = 8;

	std::array<uint8_t, TILE_WIDTH * TILE_HEIGHT> pixels;
};

}

#endif /* CODE_TRUNK_INCLUDE_TILE_HPP_ */
