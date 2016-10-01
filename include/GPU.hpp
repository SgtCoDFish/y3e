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

#ifndef CODE_TRUNK_INCLUDE_GPU_HPP_
#define CODE_TRUNK_INCLUDE_GPU_HPP_

#include <cstdint>

#include <unordered_map>
#include <set>

#include "Sprite.hpp"
#include "Tile.hpp"

namespace y3e {
class Emulator;

/**
 * Some games (e.g. Zelda: Link to the Past) render 8x16 sprites instead
 * of the square 8x8 used by most games. This is set by the LCDC register.
 */
enum class SpriteSizeMode
	: uint8_t {
		SIZE_8_8 = 0, //!< SIZE_8_8
	SIZE_8_16 = 1 //!< SIZE_8_16
};

/**
 * The Game Boy's GPU can be modelled as looping through 4 states in a set pattern.
 * This enumeration abstracts this into a type-safe construction.
 */
enum class GPUMode
	: uint8_t {
		HBLANK = 0b00, //!< HBLANK
	VBLANK = 0b01, //!< VBLANK
	OAM = 0b10,    //!< OAM
	TRANSFER = 0b11    //!< TRANSFER
};

/**
 * Models the Game Boy's GPU; that is, commands for drawing and sprite details.
 *
 * The LCDC (LCD Control) register at 0xFF40 is very widely used to affect the GPU.
 * The STAT (LCD Status) register at 0xFF41 is also important. The GPU sets the STAT
 * register on its own given reguarly updates.
 *
 * See
 * http://www.codeslinger.co.uk/pages/projects/gameboy/lcd.html
 *
 * Note that emulator rendering is handled in a Renderer subclass in the backend,
 * and not by the GPU module.
 */
class GPU {
public:
	explicit GPU(Emulator * emulator_);
	~GPU() = default;

	/**
	 * Enables rendering the Game Boy's background.
	 *
	 * Note that using this function will impact emulation accuracy as rendering of
	 * the background should be handled ingame.
	 */
	void enableBackground();

	/**
	 * Disables rendering the Game Boy's background.
	 *
	 * Note that using this function will impact emulation accuracy as rendering of
	 * the background should be handled ingame.
	 */
	void disableBackground();

	/**
	 * Enables rendering the Game Boy's window. Should not be confused with any sort
	 * of GUI presented by the emulator; this is an entirely internal concept.
	 *
	 * Note that using this function will impact emulation accuracy as rendering of
	 * the window should be handled ingame.
	 */
	void enableWindow();
	/**
	 * Disables rendering the Game Boy's window. Should not be confused with any sort
	 * of GUI presented by the emulator; this is an entirely internal concept.
	 *
	 * Note that using this function will impact emulation accuracy as rendering of
	 * the window should be handled ingame.
	 */
	void disableWindow();

	/**
	 * Enables rendering loaded Game Boy sprites.
	 *
	 * Note that using this function will impact emulation accuracy as rendering of
	 * sprites should be handled ingame.
	 */
	void enableSprites();
	/**
	 * Disables rendering loaded Game Boy sprites.
	 *
	 * Note that using this function will impact emulation accuracy as rendering of
	 * sprites should be handled ingame.
	 */
	void disableSprites();

	/**
	 * Turns on the emulated LCD. Note that this has other side effects,
	 * and likely shouldn't be called for accuracy reasons.
	 */
	void lcdOn();

	/**
	 * Turns off the emulated LCD. Note that this has other side effects,
	 * and likely shouldn't be called for accuracy reasons.
	 */
	void lcdOff();

	/**
	 * Notifies the GPU that a write happened in areas of memory it is concerned with.
	 *
	 * This is useful for updating cached data as appropriate.
	 */
	void notifyVRAMWrite(const uint16_t &address);

	/**
	 * Should be called when there is a write to the LCDC register, which the GPU uses to
	 * control various emulated components. This is usually handled by the emulator itself.
	 */
	void handleLCDCWrite(const uint8_t &value);

	/**
	 * Called internally when GPU state changes and affects the LCD Status register. Will
	 * call out as appropriate to update the details in emulated memory.
	 */
	void handleLCDStatus(bool modeChanged);

	/**
	 * Called internally to handle a vblank, which usually entails rendering.
	 */
	void handleVBlank();

	/**
	 * Called when the OAM byte is set, which controls batch data copying and is usually used
	 * for updating tiles/sprites.
	 */
	void setOAMByte(uint16_t address, uint8_t value);

	/**
	 * Should be called every frame to notify how many clocks have passed, so that the GPU can be
	 * updated accordingly.
	 */
	void update(const int64_t &clocks);

//  Disabled for release
//	/**
//	 * Gets a Sprite as stored internally. May not be valid if any cache has not been updated.
//	 */
//	Sprite getSprite(uint8_t index);

	/**
	 * Returns a tile from memory (or possibly cached, but guaranteed to be correct as of the call)
	 * which matches the tile in the memory.
	 *
	 * Always uses tile table 0 (0x8000 - 0x8FFF).
	 */
	Tile getTileFromTable0(uint8_t index);

	/**
	 * Returns a tile from memory (or possibly cached, but guaranteed to be correct as of the call)
	 * which matches the tile in the memory.
	 *
	 * Always uses tile table 1 (0x8800 - 0x97FF).
	 */
	Tile getTileFromTable1(int8_t index);

	/**
	 * Gets a tile from the current table. If the current table is "table 1" (0x8800 - 0x97FF) then index is cast to a signed integer.
	 */
	Tile getTileFromCurrentTable(uint8_t index);

//  Not implemented for release.
//	void setTileTable0(uint8_t index, const Tile &tile);
//	void setTileTable1(int8_t index, const Tile &tile);

	/**
	 * 0xFF44 resets the scanline counter when written to.
	 */
	void handleFF44Write();

	/**
	 * Enables tile caching, in which tiles are stored internally in a more useful
	 * format so that they can be quickly retrieved later.
	 *
	 * Disabling tile caching has a strongly negative impact on performance.
	 */
	void enableTileCaching();

	/**
	 * Disables tile caching, in which tiles are stored internally in a more useful
	 * format so that they can be quickly retrieved later.
	 *
	 * Disabling tile caching has a strongly negative impact on performance.
	 */
	void disableTileCaching();

	bool tileCachingEnabled() const {
		return tileCachingEnabled_;
	}

	bool isLCDOn() const {
		return lcdOn_;
	}

	bool isBackgroundEnabled() const {
		return backgroundEnabled_;
	}

	bool isWindowEnabled() const {
		return windowEnabled_;
	}

	bool isSpriteRenderingEnabled() const {
		return spritesEnabled_;
	}

	uint16_t getBGTileMapBaseLocation() const {
		return bgTileMapBaseLocation_;
	}

	uint16_t getWindowTileMapBaseLocation() const {
		return windowTileMapBaseLocation_;
	}

	const std::unordered_map<uint16_t, Sprite> &getSpriteMap() const {
		return spriteMap;
	}

	SpriteSizeMode getSpriteSizeMode() const {
		return spriteSizeMode_;
	}

	int64_t getTicks() const {
		return tickCounter_;
	}

protected:
	Emulator * emulator;

	GPUMode mode { GPUMode::HBLANK };

	uint16_t windowTileMapBaseLocation_ = 0x9800;
	uint16_t bgWindowTileDataBaseLocation_ = 0x8000;
	uint16_t bgTileMapBaseLocation_ = 0x9800;

	SpriteSizeMode spriteSizeMode_ = SpriteSizeMode::SIZE_8_8;

	bool tileCachingEnabled_ = true;

	std::set<uint8_t> table0DirtyList;
	std::set<uint8_t> table1DirtyList;
	void handleDirtyLists();

	std::array<Tile, 256> table0Cache;
	std::array<Tile, 256> table1Cache;

	int64_t tickCounter_ = 0;

	bool lcdOn_ = true;

	bool spritesEnabled_ = false;
	bool backgroundEnabled_ = true;
	bool windowEnabled_ = false;

	void setStatus(uint8_t &status);

	void initSpriteMap();
	std::unordered_map<uint16_t, Sprite> spriteMap;
};

}

#endif /* CODE_TRUNK_INCLUDE_GPU_HPP_ */
