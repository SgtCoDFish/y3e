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

#include <string>

#include "PaletteManager.hpp"
#include "Emulator.hpp"

namespace y3e {

const glm::ivec4 &GBPalette::getColor(uint8_t index) const {
	if (index >= palette.size()) {
		index = palette.size() - 1;
	}

	return palette[index];
}

const glm::ivec4 &EditablePalette::getColor(uint8_t index) const {
	return palette[index];
}

PaletteManager::PaletteManager(Emulator * emulator) :
		emulator_ { emulator } {
	// start with two default gb palettes.
	fgPalettes.emplace_back(std::make_unique<GBPalette>());
	bgPalettes.emplace_back(std::make_unique<GBPalette>());

//	if(emulator_->getROM()->header.isGBC) {
//		// TODO: do something different if GBC
//	}
}

uint8_t PaletteManager::handleBGPaletteRegisterWrite(const uint8_t &val) {
	ensureEditablePalettes();

	const uint8_t col0 = (val & 0b00000011) >> 0;
	const uint8_t col1 = (val & 0b00001100) >> 2;
	const uint8_t col2 = (val & 0b00110000) >> 4;
	const uint8_t col3 = (val & 0b11000000) >> 6;

	const auto palette = (EditablePalette *) bgPalettes[editableBGIndex].get();

	palette->palette[0] = palette->colNumToVec[col0];
	palette->palette[1] = palette->colNumToVec[col1];
	palette->palette[2] = palette->colNumToVec[col2];
	palette->palette[3] = palette->colNumToVec[col3];

//	emulator_->debugPrint(std::string("BG Palette[0] set to colour ") + std::to_string(col0));
//	emulator_->debugPrint(std::string("BG Palette[1] set to colour ") + std::to_string(col1));
//	emulator_->debugPrint(std::string("BG Palette[2] set to colour ") + std::to_string(col2));
//	emulator_->debugPrint(std::string("BG Palette[3] set to colour ") + std::to_string(col3));

	return bgPaletteIndex_;
}

uint8_t PaletteManager::handleOBJ0PaletteRegisterWrite(const uint8_t &val) {
	ensureEditablePalettes();

	const uint8_t col0 = (val & 0b00000011) >> 0;
	const uint8_t col1 = (val & 0b00001100) >> 2;
	const uint8_t col2 = (val & 0b00110000) >> 4;
	const uint8_t col3 = (val & 0b11000000) >> 6;

	const auto palette = (EditablePalette *) fgPalettes[editableOBJ0Index].get();

	palette->palette[0] = palette->colNumToVec[col0];
	palette->palette[1] = palette->colNumToVec[col1];
	palette->palette[2] = palette->colNumToVec[col2];
	palette->palette[3] = palette->colNumToVec[col3];

//	emulator_->debugPrint(std::string("BG Palette[0] set to colour ") + std::to_string(col0));
//	emulator_->debugPrint(std::string("BG Palette[1] set to colour ") + std::to_string(col1));
//	emulator_->debugPrint(std::string("BG Palette[2] set to colour ") + std::to_string(col2));
//	emulator_->debugPrint(std::string("BG Palette[3] set to colour ") + std::to_string(col3));

	return obj0PaletteIndex_;
}

uint8_t PaletteManager::handleOBJ1PaletteRegisterWrite(const uint8_t &val) {
	ensureEditablePalettes();

	const uint8_t col0 = (val & 0b00000011) >> 0;
	const uint8_t col1 = (val & 0b00001100) >> 2;
	const uint8_t col2 = (val & 0b00110000) >> 4;
	const uint8_t col3 = (val & 0b11000000) >> 6;

	const auto palette =
			(EditablePalette *) fgPalettes[editableOBJ1Index].get();

	palette->palette[0] = palette->colNumToVec[col0];
	palette->palette[1] = palette->colNumToVec[col1];
	palette->palette[2] = palette->colNumToVec[col2];
	palette->palette[3] = palette->colNumToVec[col3];

//	emulator_->debugPrint(std::string("BG Palette[0] set to colour ") + std::to_string(col0));
//	emulator_->debugPrint(std::string("BG Palette[1] set to colour ") + std::to_string(col1));
//	emulator_->debugPrint(std::string("BG Palette[2] set to colour ") + std::to_string(col2));
//	emulator_->debugPrint(std::string("BG Palette[3] set to colour ") + std::to_string(col3));

	return obj1PaletteIndex_;
}

void PaletteManager::ensureEditablePalettes() {
	if (editableBGIndex == 255u) {
		editableBGIndex = bgPalettes.size();

		bgPalettes.emplace_back(std::make_unique<EditablePalette>());
	}

	if (editableOBJ0Index == 255u) {
		editableOBJ0Index = fgPalettes.size();

		fgPalettes.emplace_back(std::make_unique<EditablePalette>());
	}

	if (editableOBJ1Index == 255u) {
		editableOBJ1Index = fgPalettes.size();

		fgPalettes.emplace_back(std::make_unique<EditablePalette>());
	}

	bgPaletteIndex_ = editableBGIndex;
	obj0PaletteIndex_ = editableOBJ0Index;
	obj1PaletteIndex_ = editableOBJ1Index;
}

void PaletteManager::setOBJ0Palette(uint8_t index) {
	obj0PaletteIndex_ = index;
}

void PaletteManager::setOBJ1Palette(uint8_t index) {
	obj1PaletteIndex_ = index;
}

void PaletteManager::setBGPalette(uint8_t index) {
	bgPaletteIndex_ = index;
}

}
