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

#include "Tile.hpp"
#include "MemoryManager.hpp"

namespace y3e {

Tile Tile::loadFromMemory(MemoryManager * mm, const uint16_t &address) {
	Tile tile;

	// each tile takes 16 bytes, read 2 at a time
	for(int i = 0; i < 16; i += 2) {
		const auto byte1 = mm->read(address + i + 0u);
		const auto byte2 = mm->read(address + i + 1u);

		for(int j = 7; j >= 0; j--) {
			const uint8_t lowBit = (byte1 >> j) & (1);
			const uint8_t highBit = (byte2 >> j) & (1);

			const uint8_t colorIndex = lowBit + (highBit << 1);

			tile.setPixel(7-j, i/2, colorIndex);
		}
	}

	return tile;
}

uint8_t Tile::getPixel(uint8_t x, uint8_t y) const {
	return pixels[y * TILE_WIDTH + x];
}

void Tile::setPixel(uint8_t x, uint8_t y, uint8_t value) {
	pixels[y * TILE_WIDTH + x] = value;
}

}
