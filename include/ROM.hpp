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

#ifndef CODE_TRUNK_INCLUDE_ROM_HPP_
#define CODE_TRUNK_INCLUDE_ROM_HPP_

#include <cstdint>

#include <iostream>
#include <memory>
#include <array>
#include <string>
#include <vector>

#include "Instructions.hpp"

namespace y3e {

/**
 * The different types of cartridge that are available. Only a subset may be supported.
 *
 * NOTE: After updating this list, you may need to update the cartridgeHasBattery function too.
 */
enum class CartridgeType {
	ROM_ONLY = 0x00,

	MBC1_ONLY = 0x01,
	MBC1_RAM = 0x02,
	MBC1_RAM_BATT = 0x03,

	MBC3_TIMER_BATTERY = 0x0F,
	MBC3_TIMER_RAM_BATTERY = 0x10,
	MBC3_ONLY = 0x11,
	MBC3_RAM = 0x12,
	MBC3_RAM_BATTERY = 0x13,

	MBC5_ONLY = 0x19,
	MBC5_RAM = 0x1A,
	MBC5_RAM_BATTERY = 0x1B,

	MBC5_RUMBLE_SRAM_BATTERY = 0x1E,

	// 0xC0 is not a valid type as given in the manual so used to signify unsupported types
	UNSUPPORTED = 0xC0,
};

/**
 * Abstraction of MBC types used by the memory manager to handle changing RAM/ROM banks.
 */
enum class MBCType {
	NONE, //!< NONE
	MBC1, //!< MBC1
	MBC2, //!< MBC2
	MBC3, //!< MBC3
	MBC5 //!< MBC5
};

/**
 * Compares the given byte read from a ROM and returns the appropriate cartridge type.
 *
 * Will raise an error if the given cart type is not supported.
 */
CartridgeType getCartridgeTypeFromFile(uint8_t fileByte);

/**
 * Returns the MBC type of a cartridge. For example, both MBC3_TIMER_RAM_BATTERY and
 * MBC3_RAM_BATTERY should return the same MBCType.
 */
MBCType getMBCTypeFromCartridgeType(const CartridgeType &type);

/**
 * Calculates a valid licensee code from the new and old licensee identification methods
 * present in carts.
 */
uint8_t calculateLicenseeCode(uint8_t newHigh, uint8_t newLow, uint8_t old);

/**
 * Converts a byte read from a ROM representing file size to the actual size of the RAM
 * on cart.
 *
 * If useMBC2Workaround is true, the RAM size will be set accurately for an MBC2 chip
 * which always has fileByte == 0x00 but has a nonzero amount of RAM.
 */
uint32_t getRAMSizeFromFile(uint8_t fileByte, bool useMBC2Workaround = false);

/**
 * Returns true if the given type supports a battery (i.e. has a save game feature,
 * persisting RAM between boots)
 */
bool cartridgeHasBattery(const CartridgeType &type);

/**
 * Holds details about the size of a ROM, as read from a ROM header.
 */
struct ROMSizeInfo {
	explicit ROMSizeInfo(uint8_t fileByte_);
	~ROMSizeInfo() = default;

	// The size, in bytes, of the ROM
	uint64_t romSize;

	// The number of banks this represents
	uint16_t bankCount;

	uint8_t byteFromFile;
};

/**
 * The header of a GB ROM file (.gb file)
 *
 * See http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf page 10 onwards for
 * documentation.
 */
struct ROMHeader {
	// the first 4 bytes, detailing the beginning code execution instructions
	std::array<int8_t, 0x4> initial;

	// scrolling graphic shown at power on
	std::array<int8_t, 0x30> logoGraphic;

	// stored in upper case ASCII
	std::string gameName;

	// a code identifying the licensee
	uint8_t licenseeCode;

	// is the rom for the Game Boy Color true if the value read in is 0x80, false otherwise
	// can sometimes be another value
	bool isGBC;

	// does the cart have superGB additions?
	bool superGB;

	// there are several types of cartridge available
	// WARNING: some are ignored for now
	CartridgeType cartridgeType;

	// used to help distinguish between the different actions that writing to various areas
	// of memory have under varying MBCs
	MBCType mbcType;

	// contains information about various available ROM sizes
	std::unique_ptr<ROMSizeInfo> sizeInfo;

	// the size of on chip RAM, in bytes
	uint32_t ramSize;

	// true if the cart is Japanese
	bool isJapanese;

	// usually 0, "mask ROM version number"
	uint8_t maskROMVersion;

	// verification for the header; GB does not run without this being correct
	uint8_t headerChecksum;

	// verification for the whole ROM; GB ignores this
	uint16_t globalChecksum;
};

/**
 * Holds a loaded ROM (from a .gb or .gbc file).
 *
 * The layout will depend on the MBC (memory bank controller) present in the cart.
 *
 * See http://gbdev.gg8.se/wiki/articles/Memory_Bank_Controllers
 */
struct ROM {
	explicit ROM() = default;
	~ROM() = default;

	ROMHeader header;
	uint64_t actualRomSize = 0;

	// always the same between different ROMs, 16kB block of instructions
	std::array<uint8_t, 0x4000> firstBlock;

	std::vector<std::array<uint8_t, 0x4000>> otherBlocks;
};

}

std::ostream &operator <<(std::ostream &stream, const y3e::ROMHeader &romHeader);
std::ostream &operator <<(std::ostream &stream, const y3e::ROM &rom);

#endif /* CODE_TRUNK_INCLUDE_ROM_HPP_ */
