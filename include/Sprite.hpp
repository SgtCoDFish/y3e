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

#ifndef CODE_TRUNK_INCLUDE_SPRITE_HPP_
#define CODE_TRUNK_INCLUDE_SPRITE_HPP_

#include <cstdint>

namespace y3e {

/**
 * These are stored in memory at 0xFE00 - 0xFE9F and represent the various properties a
 * sprite can have.
 *
 * Note that a Sprite has no guarantee to remain the same between two iterations of the main loop;
 * if you require an updated Sprite you must fetch each frame.
 */
struct Sprite {
	uint8_t yPos = 0u;
	uint8_t xPos = 0u;
	uint8_t patternNumber = 0u;
	uint8_t flags = 0u;

	/**
	 * Returns true if bit 7 of flags is set,
	 * that is, the sprite is hidden behind colours
	 * 1, 2, 3 of BG/Window.
	 */
	bool hasPriority() const;

	/**
	 * Returns true if the sprite should be flipped vertically.
	 * (flags bit 6)
	 */
	bool yFlip() const;

	/**
	 * Returns true if the sprite should be flipped horizontally.
	 * (flags bit 5)
	 */
	bool xFlip() const;

	/**
	 * If true, sprite uses OBJ 1PAL. If false, uses OBJ 0PAL.
	 * (flags bit 4)
	 */
	bool usesPalette1() const;
};

}

#endif /* CODE_TRUNK_INCLUDE_SPRITE_HPP_ */
