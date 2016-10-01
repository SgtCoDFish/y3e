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

#ifndef CODE_TRUNK_INCLUDE_PALETTEMANAGER_HPP_
#define CODE_TRUNK_INCLUDE_PALETTEMANAGER_HPP_

#include <cstdint>

#include <vector>
#include <array>
#include <memory>

#include <glm/glm.hpp>

namespace y3e {
class Emulator;

/**
 * An abstract class representing a palette for some system.
 * Subclasses should implement a palette for a given system (GB/GBC). Some subclasses are provided.
 */
struct Palette {
	explicit Palette() = default;
	virtual ~Palette() = default;

	/**
	 * Return a colour from the given index. If the index doesn't exist for the currently emulated system,
	 * the behaviour is undefined.
	 */
	virtual const glm::ivec4 &getColor(uint8_t index) const = 0;
};

/**
 * An unchanging palette for the original Game Boy. This will not work correctly for games which
 * manually change the palette to do special effects and the EditablePalette is recommended instead.
 */
struct GBPalette : public Palette {
	explicit GBPalette() = default;
	virtual ~GBPalette() = default;

	const std::array<glm::ivec4, 4> palette { { //
	        glm::ivec4 { 0xFC, 0xC8, 0x8C, 0xFF }, //
	                glm::ivec4 { 0xDC, 0xB4, 0x5C, 0xFF }, //
	                glm::ivec4 { 0x98, 0x7c, 0x3C, 0xFF }, //
	                glm::ivec4 { 0x4C, 0x3C, 0x1C, 0xFF } } };

	virtual const glm::ivec4 &getColor(uint8_t index) const override;
};

/**
 * A Game Boy palette which can be changed as needed to support games which manually change colours.
 */
struct EditablePalette : public Palette {
	explicit EditablePalette() = default;
	virtual ~EditablePalette() = default;

	glm::ivec4 col3 { 0x4C, 0x3C, 0x1C, 0xFF }; // usually darkest colour
	glm::ivec4 col2 { 0x98, 0x7c, 0x3C, 0xFF };
	glm::ivec4 col1 { 0xDC, 0xB4, 0x5C, 0xFF };
	glm::ivec4 col0 { 0xFC, 0xC8, 0x8C, 0xFF }; // usually lightest colour

	const std::array<glm::ivec4, 4> colNumToVec { { col0, col1, col2, col3 } };
	std::array<glm::ivec4, 4> palette { { col3, col2, col1, col0 } };

	virtual const glm::ivec4 &getColor(uint8_t index) const override;
};

/**
 * Handles palettes for the GB/GBC when provided with relevant memory writes.
 */
class PaletteManager {
public:
	explicit PaletteManager(Emulator * emulator_);
	~PaletteManager() = default;

	/**
	 * Sets the foreground obj0 palette to an existing palette managed by this PaletteManager.
	 *
	 * Note that bounds checking only happens in debug mode.
	 */
	void setOBJ0Palette(uint8_t index);

	/**
	 * Sets the foreground obj1 palette to an existing palette managed by this PaletteManager.
	 *
	 * Note that bounds checking only happens in debug mode.
	 */
	void setOBJ1Palette(uint8_t index);

	/**
	 * Sets the background palette to an existing palette managed by this PaletteManager.
	 *
	 * Note that bounds checking only happens in debug mode.
	 */
	void setBGPalette(uint8_t index);

	const Palette * getBGPalette() const {
		return bgPalettes[bgPaletteIndex_].get();
	}

	const Palette * getOBJ0Palette() const {
		return fgPalettes[obj0PaletteIndex_].get();
	}

	const Palette * getOBJ1Palette() const {
		return fgPalettes[obj1PaletteIndex_].get();
	}

	/**
	 * Handles a write to the BG palette register.
	 *
	 * Note that if this method is called at any point, the palette is likely to be permanently changed to a new palette.
	 * This means that the performance benefit of using the GBPalette class will be negated, but this isn't likely to
	 * affect performance much.
	 *
	 * Returns the palette index of the background palette.
	 */
	uint8_t handleBGPaletteRegisterWrite(const uint8_t &val);

	/**
	 * Handles a write to the OBJ0 palette register.
	 *
	 * See note for handleBGPaletteRegisterWrite.
	 *
	 * Returns the palette index of the obj0 palette.
	 */
	uint8_t handleOBJ0PaletteRegisterWrite(const uint8_t &val);

	/**
	 * Handles a write to the OBJ1 palette register.
	 *
	 * See note for handleBGPaletteRegisterWrite.
	 *
	 * Returns the palette index of the obj1 palette.
	 */
	uint8_t handleOBJ1PaletteRegisterWrite(const uint8_t &val);

private:
	Emulator * const emulator_;

	void ensureEditablePalettes();

	uint8_t bgPaletteIndex_ = 0u;
	uint8_t obj0PaletteIndex_ = 0u;
	uint8_t obj1PaletteIndex_ = 0u;

	std::vector<std::unique_ptr<Palette>> bgPalettes;
	std::vector<std::unique_ptr<Palette>> fgPalettes;

	uint8_t editableBGIndex = 255u;
	uint8_t editableOBJ0Index = 255u;
	uint8_t editableOBJ1Index = 255u;
};

}

#endif /* CODE_TRUNK_INCLUDE_PALETTEMANAGER_HPP_ */
