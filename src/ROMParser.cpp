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

#include <cstdlib>
#include <cctype>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <utility>
#include <string>
#include <sstream>

#include "ROMParser.hpp"
#include "Emulator.hpp"
#include "InstructionParser.hpp"

namespace y3e {

GBROMParser::GBROMParser() {

}

std::unique_ptr<ROM> GBROMParser::parseROM(const std::string &fileName) {
	std::ifstream f(fileName, std::ios::in | std::ios::binary);

	if (!f) {
		std::cerr << "Couldn't open " << fileName << std::endl;
		return nullptr;
	}

	return parseROM(f);
}

std::unique_ptr<ROM> GBROMParser::parseROM(std::ifstream &f) {
	auto ret = std::make_unique<ROM>();

	ret->header = parseHeader(f);

	if (ret->header.cartridgeType == CartridgeType::UNSUPPORTED) {
		return nullptr;
	}

	f.seekg(0x0000, std::ios_base::beg);
	const auto beginPos = f.tellg();

	f.read(reinterpret_cast<char *>(ret->firstBlock.data()), ret->firstBlock.size());

	f.seekg(0x0000, std::ios_base::end);

	// size should be accurate as we're open in a binary file
	const auto size = f.tellg();
	ret->actualRomSize = static_cast<uint64_t>(size - beginPos);

	const int otherBlockCount = (ret->actualRomSize - 0x4000) / 0x4000;

	std::cout << "ROM contains " << otherBlockCount << " other blocks\n";

	for (int i = 1; i <= otherBlockCount; i++) {
		ret->otherBlocks.emplace_back();
		f.seekg(i * 0x4000, std::ios_base::beg);
		f.read(reinterpret_cast<char *>(ret->otherBlocks.back().data()), ret->firstBlock.size());
	}

	return ret;
}

ROMHeader GBROMParser::parseHeader(std::ifstream &f) {
	ROMHeader ret;
	f.seekg(0x0100, std::ios_base::beg);

	// 0x0100 - 0x0103
	f.read(reinterpret_cast<char *>(ret.initial.data()), ret.initial.size());

	// 0x0104 - 0x0133
	f.read(reinterpret_cast<char *>(ret.logoGraphic.data()), ret.logoGraphic.size());

	std::stringstream ss;

	// 0x0134 - 0x0142
	for (int i = 0; i < 15; ++i) {
		const char value = static_cast<char>(f.get());

		if (value != 0x00) {
			// cast to char because it's stored as uppercase ASCII
			ss << value;
		}
	}

	ret.gameName = std::move(ss.str());

	// 0x0143
	uint8_t isGBC = uint8_t(f.get());
	// if == 0x80, the rom is a GBC rom.
	ret.isGBC = (isGBC == 0x80);

	// 0x0144
	char newVendorHigh = char(f.get());
	// 0x0145
	char newVendorLow = char(f.get());

	// vendor details are calculated all at once after 0x014B is read.

	// 0x0146
	// 0x03 implies superGB functions are provided in cart
	uint8_t superGB = uint8_t(f.get());
	ret.superGB = (superGB == 0x03);

	// 0x0147
	uint8_t cartridgeCode = uint8_t(f.get());
	ret.cartridgeType = getCartridgeTypeFromFile(cartridgeCode);

	if (ret.cartridgeType == CartridgeType::UNSUPPORTED) {
		std::cout << "Unsupported cartridge type: 0x" << std::hex << (int(cartridgeCode) & 0xFF) << std::endl;
	}

	ret.mbcType = getMBCTypeFromCartridgeType(ret.cartridgeType);

	// 0x0148
	uint8_t romSize = uint8_t(f.get());
	ret.sizeInfo = std::make_unique<ROMSizeInfo>(romSize);

	// 0x0149
	uint8_t ramSize = uint8_t(f.get());
	ret.ramSize = getRAMSizeFromFile(ramSize);

	// 0x014A
	// cart is Japanese if == 0x00
	uint8_t japanese = uint8_t(f.get());
	ret.isJapanese = (japanese == 0x00);

	// 0x014B
	// old licensee code, used with the new codes read above to make the actual licensee id code
	uint8_t oldVendor = uint8_t(f.get());
	ret.licenseeCode = calculateLicenseeCode(newVendorHigh, newVendorLow, oldVendor);

	// 0x014C
	// usually 0 and can be ignored
	ret.maskROMVersion = uint8_t(f.get());

	// 0x014D
	// important to the game boy, calculated based on the whole header
	ret.headerChecksum = uint8_t(f.get());

	// 0x014E-0x014F
	// global checksum of whole ROM, game boy ignores
	std::array<int8_t, 2> globalChecksum;
	f.read(reinterpret_cast<char*>(globalChecksum.data()), globalChecksum.size());

	ret.globalChecksum = 0x10 * globalChecksum[0] + globalChecksum[1];

	return ret;
}

}
