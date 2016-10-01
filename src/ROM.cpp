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
#include <iomanip>
#include <string>

#include "ROM.hpp"

namespace y3e {

CartridgeType getCartridgeTypeFromFile(uint8_t fileByte) {
	switch (fileByte) {
	case 0x00:
		return CartridgeType::ROM_ONLY;

	case 0x01:
		return CartridgeType::MBC1_ONLY;

	case 0x02:
		return CartridgeType::MBC1_RAM;

	case 0x03:
		return CartridgeType::MBC1_RAM_BATT;

	case 0x0F:
		return CartridgeType::MBC3_TIMER_BATTERY;

	case 0x10:
		return CartridgeType::MBC3_TIMER_RAM_BATTERY;

	case 0x11:
		return CartridgeType::MBC3_ONLY;

	case 0x12:
		return CartridgeType::MBC3_RAM;

	case 0x13:
		return CartridgeType::MBC3_RAM_BATTERY;

	case 0x19:
		return CartridgeType::MBC5_ONLY;

	case 0x1A:
		return CartridgeType::MBC5_RAM;

	case 0x1B:
		return CartridgeType::MBC5_RAM_BATTERY;

	case 0x1E:
		return CartridgeType::MBC5_RUMBLE_SRAM_BATTERY;

	default:
		std::cout << "Unsupported cartridge parse type: " << (static_cast<uint16_t>(fileByte) & 0xFF) << std::endl;
		return CartridgeType::UNSUPPORTED;
	}
}

MBCType getMBCTypeFromCartridgeType(const CartridgeType &type) {
	switch (type) {
	case CartridgeType::ROM_ONLY:
		return MBCType::NONE;

	case CartridgeType::MBC1_ONLY:
	case CartridgeType::MBC1_RAM:
	case CartridgeType::MBC1_RAM_BATT:
		return MBCType::MBC1;

	case CartridgeType::MBC3_ONLY:
	case CartridgeType::MBC3_RAM:
	case CartridgeType::MBC3_TIMER_BATTERY:
	case CartridgeType::MBC3_RAM_BATTERY:
	case CartridgeType::MBC3_TIMER_RAM_BATTERY:
		return MBCType::MBC3;

	case CartridgeType::MBC5_ONLY:
	case CartridgeType::MBC5_RAM:
	case CartridgeType::MBC5_RAM_BATTERY:
	case CartridgeType::MBC5_RUMBLE_SRAM_BATTERY:
		return MBCType::MBC5;

	case CartridgeType::UNSUPPORTED:
	default:
		std::cout << "WARNING: No MBCType returned for cartridge type "
		        << static_cast<std::underlying_type<CartridgeType>::type>(type) << std::endl;
		return MBCType::NONE;
	}
}

uint32_t getRAMSizeFromFile(uint8_t fileByte, bool useMBC2Workaround) {
	if (useMBC2Workaround) {
		if (fileByte == 0x00) {
			// MBC2 chip always has byte == 0x00 but has 256 bytes of memory.
			return 256;
		}
	}

	switch (fileByte) {
	case 0x00:
		return 0;

	case 0x01:
		return 2000;

	case 0x02:
		return 8000;

	case 0x03:
		return 32000;

	case 0x04:
		return 128000;

	case 0x05:
		return 64000;

	default: {
		std::cerr << "Invalid byte passed to getRAMSizeFromFile: " << std::hex << fileByte << std::endl;
		std::abort();
		return 0;
	}
	}
}

uint8_t calculateLicenseeCode(uint8_t newHigh, uint8_t newLow, uint8_t old) {
	// TODO: Impl
	return 0x00;
}

bool cartridgeHasBattery(const CartridgeType &type) {
	if (type == CartridgeType::MBC1_RAM_BATT //
	|| type == CartridgeType::MBC3_TIMER_RAM_BATTERY //
	|| type == CartridgeType::MBC3_RAM_BATTERY
	|| type == CartridgeType::MBC3_TIMER_BATTERY
	|| type == CartridgeType::MBC5_RAM_BATTERY
	|| type == CartridgeType::MBC5_RUMBLE_SRAM_BATTERY) {
		return true;
	}

	return false;
}

ROMSizeInfo::ROMSizeInfo(uint8_t fileByte_) :
		        byteFromFile { fileByte_ } {
	if (byteFromFile <= 0x07) {
		// much simpler calculation for <= 0x07

		// note that for byteFromFile == 0x05 and 0x06 there are some differences
		// in bank count for certain cartridge types, see manual
		romSize = 32000 << byteFromFile;
		bankCount = 4 * byteFromFile;
		return;
	}

	// otherwise, handle the 3 odd cases
	// set to 1000 to start to save having to type lots of 0s later.
	romSize = 1000;

	switch (byteFromFile) {
	case 0x52: {
		romSize *= 1100; // 1.1MB
		bankCount = 72;
		break;
	}

	case 0x53: {
		romSize *= 1200; // 1.2MB
		bankCount = 80;
		break;
	}

	case 0x54: {
		romSize *= 1500; // 1.5MB
		bankCount = 96;
		break;
	}

	default: {
		std::cerr << "Invalid byte passed to ROMSizeInfo: " << std::hex << byteFromFile;
		std::abort();
		break;
	}
	}
}

}

std::ostream &operator <<(std::ostream &stream, const y3e::ROMHeader &romHeader) {
	stream << "Game Name: \"" << romHeader.gameName << "\"\n";
	stream << "Is GBC Game: " << romHeader.isGBC << "\n";
	stream << "Is Japanese: " << romHeader.isJapanese << "\n";

	stream << "ROM Size: 0x" << romHeader.sizeInfo->romSize << "B\n";
	stream << "RAM Size: 0x" << romHeader.ramSize << "B\n";

	stream << "Header checksum: " << std::hex << (int(romHeader.headerChecksum) & 0xFF) << "\nFile checksum: "
	        << romHeader.globalChecksum << std::endl;

	return stream;
}

std::ostream &operator <<(std::ostream &stream, const y3e::ROM &rom) {
	stream << rom.header << std::endl;
	stream << "Actual ROM Size: 0x" << rom.actualRomSize << std::endl;

	return stream;
}
