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

#include "GPU.hpp"
#include "Emulator.hpp"

namespace y3e {

GPU::GPU(Emulator * emulator_) :
		        emulator { emulator_ } {
	// The LCDC register is set to 0x91 at reset
	// so we handle it here
	handleLCDCWrite(0x91);
	initSpriteMap();
}

void GPU::update(const int64_t &clocks) {
	handleDirtyLists();

	if (!isLCDOn()) {
		/*
		 * LCD being off has several side effects:
		 * - Mode is set to mode 1 (VBLANK)
		 * - Current scanline is set to 0
		 * - Tick counter is set to 0.
		 */
		tickCounter_ = 0;
		// writing to 0xFF44 resets scanline
		emulator->memory.write(0xFF44, 0);

		mode = GPUMode::VBLANK;
		uint8_t status = emulator->memory.read(0xFF41);
		setStatus(status);
		emulator->memory.write(0xFF41, status);
		return;
	}

	bool modeChanged = false;

	tickCounter_ += clocks;
//	std::cout << "Mode is: " << (static_cast<uint16_t>(mode) & 0xff) << ", clocks: " << tickCounter_ << ", scanline: " << (static_cast<uint16_t>(emulator->memory.read(0xff44)) & 0xff) << std::endl;

	switch (mode) {
	case GPUMode::HBLANK: {
		if (tickCounter_ >= 204) {
			const auto scanline = emulator->memory.incLY();

			if (scanline == 144) {
				mode = GPUMode::VBLANK;
				modeChanged = true;

				if (isSpriteRenderingEnabled()) {
					emulator->renderer->renderSprites();
				}

				emulator->interruptHandler.triggerInterrupt(InterruptType::VBLANK);
				handleVBlank();
			} else {
				mode = GPUMode::OAM;
				modeChanged = true;
			}

			tickCounter_ -= 204;
		}

		break;
	}

	case GPUMode::VBLANK: {
		if (tickCounter_ >= 456) {
			auto scanline = 0u;
			if (0 != emulator->memory.read(0xFF44)) {
				scanline = emulator->memory.incLY();
			}

			if (scanline == 0) {
				// the scanline has reset so move to OAM state
				// may have been reset somewhere else by writing to ff44
				mode = GPUMode::OAM;
				modeChanged = true;
			}

			tickCounter_ -= 456;
		}
		break;
	}

	case GPUMode::OAM: {
		if (tickCounter_ >= 80) {
			mode = GPUMode::TRANSFER;
			modeChanged = true;

			tickCounter_ -= 80;
		}

		break;
	}

	case GPUMode::TRANSFER: {
		if (tickCounter_ >= 172) {
			mode = GPUMode::HBLANK;
			modeChanged = true;

			emulator->renderer->renderScanline();

			tickCounter_ -= 172;
		}
		break;
	}
	}

	if (isLCDOn()) {
		handleLCDStatus(modeChanged);
	}
}

void GPU::handleLCDCWrite(const uint8_t &value) {
	// Bit 7: LCD Control (1 = on)
	const bool lcdOnBit = value & (1 << 7);

	if (lcdOnBit && !lcdOn_) {
		lcdOn();
	} else if (!lcdOnBit && lcdOn_) {
		lcdOff();
	}

	// Bit 6: Window Tile Map Display select
	// 0: 0x9800 - 9BFF
	// 1: 0x9C00 - 9FFF
	windowTileMapBaseLocation_ = (value & (1 << 6) ? 0x9C00 : 0x9800);

	// Bit 5: Window Display On/Off (1 - on)
	value & (1 << 5) ? enableWindow() : disableWindow();

	// Bit 4: BG & Window Tile Data Select
	// 0: 0x8800 - 0x97FF
	// 1: 0x8000 - 0x8FFF
	bgWindowTileDataBaseLocation_ = (value & (1 << 4) ? 0x8000 : 0x8800);

	// Bit 3: BG Tile map Display
	// 0: 0x9800 - 0x9BFF
	// 1: 0x9C00 - 0x9FFF
	bgTileMapBaseLocation_ = (value & (1 << 3) ? 0x9C00 : 0x9800);

	// Bit 2: Sprite Size
	spriteSizeMode_ = (value & (1 << 2) ? SpriteSizeMode::SIZE_8_16 : SpriteSizeMode::SIZE_8_8);

	// Bit 1: Sprite display on/off (1 = on)
	value & (1 << 1) ? enableSprites() : disableSprites();

	// TODO: This has special behaviour under GBC but on GB it blanks the background
//	// Bit 0: Enable BG (1 = on)
	if (value & (1 << 0)) {
		enableBackground();
	} else {
		disableBackground();
	}
}

void GPU::handleLCDStatus(bool modeChanged) {
	uint8_t status = emulator->memory.read(0xFF41);

	if (!isLCDOn()) {
		return;
	}

	// if LCD is on, mode should already be set
	setStatus(status);

	const uint8_t bit2 = (emulator->memory.read(0xFF44) == emulator->memory.read(0xFF45));

	if (bit2) {
		status |= 0b00000100;
	} else {
		status &= 0b11111011;
	}

	emulator->memory.write(0xFF41, status);

	if (modeChanged && mode != GPUMode::TRANSFER) {
		// trigger LCDSTAT interrupt if enabled

		/*
		 * bit 3 = hblank int enable (hblank has value 0b00)
		 * bit 4 = vblank int enable (vblank has value 0b01)
		 * bit 5 = oam int enable    (oam has value    0b10)
		 */
		const uint8_t mask = (1 << (3 + static_cast<std::underlying_type<GPUMode>::type>(mode)));

		if (status & mask) {
			emulator->interruptHandler.triggerInterrupt(InterruptType::LCDC);
		}
	}

	if (bit2 && (status & 0b01000000)) {
		emulator->interruptHandler.triggerInterrupt(InterruptType::LCDC);
	}
}

void GPU::handleVBlank() {
	emulator->renderer->swap();
	emulator->signalVBlank();
}

void GPU::notifyVRAMWrite(const uint16_t &address) {
//	std::cout << "VRAM write at: " << address << std::endl;

	if (tileCachingEnabled()) {
		if (address >= 0x8000 && address < 0x9000) {
			// Get the id number of the tile
			table0DirtyList.insert((address - 0x8000) / 16);
		}

		if (address >= 0x8800 && address < 0x9800) {
			const uint16_t adjustedAddress = (address - 0x8800);
			const uint8_t id = adjustedAddress / 16;
//		std::cout << "Added " << std::dec << (uint16_t(id) & 0xFF) << " to table 1 cache\n" << std::hex;

			table1DirtyList.insert(id);
		}
	}

	if (address >= 0x9800 && address <= 0x9BFF) {
		// bg map data 1
	} else if (address >= 0x9C00 && address < 0xA000) {
		// bg map data 2
	}
}

Tile GPU::getTileFromTable0(uint8_t index) {
	if (table0DirtyList.empty() && tileCachingEnabled()) {
		return table0Cache[index];
	}

	return Tile::loadFromMemory(&emulator->memory, 0x8000 + index * 16);
}

Tile GPU::getTileFromTable1(int8_t index) {
	const uint8_t adjustedIndex = index + 128;

	// numbered from -128 -> 127
	if (table1DirtyList.empty() && tileCachingEnabled()) {
		return table1Cache[adjustedIndex];
	}

	return Tile::loadFromMemory(&emulator->memory, 0x8800 + adjustedIndex * 16);
}

Tile GPU::getTileFromCurrentTable(uint8_t index) {
	if (bgWindowTileDataBaseLocation_ == 0x8000) {
		return getTileFromTable0(index);
	} else {
		return getTileFromTable1(index);
	}
}

void GPU::setOAMByte(uint16_t address, uint8_t value) {
	// TODO: Handle 8x16 sprites
	const uint8_t byteNum = address % 4;
	const uint16_t spriteAddress = address - byteNum;
//	std::cout << "PC: " << emulator->reg.pc << " -- OAM write at: " << address << ", sprite address: " << spriteAddress << ", value: " << (static_cast<uint16_t>(value) & 0xFF) <<  std::endl;

	switch (byteNum) {
	case 0: {
		spriteMap[spriteAddress].yPos = value;
		break;
	}

	case 1: {
		spriteMap[spriteAddress].xPos = value;
		break;
	}

	case 2: {
		spriteMap[spriteAddress].patternNumber = value;
		break;
	}

	case 3: {
		spriteMap[spriteAddress].flags = value;
		break;
	}

	default: {
		// Impossible
		std::abort();
		break;
	}
	}
}

//Sprite GPU::getSprite(uint8_t index) {
//	// TODO: IMPL caching
//	return Sprite();
//}

void GPU::handleFF44Write() {
	tickCounter_ = 0;
	mode = GPUMode::HBLANK;
}

void GPU::enableBackground() {
	backgroundEnabled_ = true;
}

void GPU::disableBackground() {
	backgroundEnabled_ = false;
}

void GPU::enableWindow() {
	windowEnabled_ = true;
}

void GPU::disableWindow() {
	windowEnabled_ = false;
}

void GPU::enableSprites() {
	spritesEnabled_ = true;
}

void GPU::disableSprites() {
	spritesEnabled_ = false;
}

void GPU::lcdOn() {
//	std::cout << "LCD switched ON at PC: " << std::hex << emulator->reg.pc << std::endl;
	lcdOn_ = true;
	emulator->memory.write(0xff44, 0);
	mode = GPUMode::HBLANK;
	tickCounter_ = 0;
}

void GPU::lcdOff() {
//	std::cout << "LCD switched OFF at PC: " << std::hex << emulator->reg.pc << std::endl;
	lcdOn_ = false;
}

void GPU::setStatus(uint8_t &status) {
	status &= 0b11111000;

	status += static_cast<std::underlying_type<GPUMode>::type>(mode);
}

void GPU::initSpriteMap() {
	for (uint16_t i = 0xFE00; i < 0xFEA0; i += 4) {
		Sprite s;
		spriteMap.emplace(i, std::move(s));
	}
}

void GPU::handleDirtyLists() {
	for (const auto &id : table0DirtyList) {
		table0Cache[id] = Tile::loadFromMemory(&emulator->memory, uint16_t(0x8000 + id * 16));
	}

	for (const auto &id : table1DirtyList) {
//		std::cout << "id: " << (int16_t(id) & 0xFF) << std::endl;
		table1Cache[id] = Tile::loadFromMemory(&emulator->memory, uint16_t(0x8800 + id * 16));
	}

	table0DirtyList.clear();
	table1DirtyList.clear();
}

void GPU::enableTileCaching() {
	tileCachingEnabled_ = true;
}

void GPU::disableTileCaching() {
	tileCachingEnabled_ = false;
}

}
