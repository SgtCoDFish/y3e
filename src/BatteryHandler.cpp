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

#include <sstream>
#include <algorithm>
#include <array>

#include "BatteryHandler.hpp"
#include "Emulator.hpp"

#define MINIZ_HEADER_FILE_ONLY
#include "miniz.c"

namespace y3e {

BatteryHandler::BatteryHandler(Emulator * emulator_, bool verbose_) :
		        emulator { emulator_ },
	verbose{ verbose_ } {
}

void BatteryHandler::update(bool forceWrite) {
	const auto timeNow = std::chrono::high_resolution_clock::now();

	if (forceWrite) {
		stateLoaded = false;
	}

	if(!forceWrite && (stateLoaded || std::chrono::duration_cast<std::chrono::seconds>(timeNow - lastSave).count() < SAVE_DELAY_SECONDS)) {
		return;
	}

	if(forceWrite || emulator->memory.isRAMDirty()) {
		if (verbose) {
			std::cout << "Writing battery in update() " << (forceWrite ? "(forced)\n" : "\n");
		}

		writeBattery();
		emulator->memory.resetRAMDirtyFlag();
		lastSave = timeNow;
	}
}

void BatteryHandler::writeBattery(uint64_t fileVersion) {
	writeBattery(makeBatteryName(), fileVersion);
}

void BatteryHandler::writeBattery(const std::string &filename, uint64_t fileVersion) {
	if (stateLoaded) {
		return;
	}

	switch (fileVersion) {
	case 1:
		writeBatteryVersion1(filename);
		break;

	case 2:
		writeBatteryVersion2(filename);
		break;

	default: {
		std::cout << "Cannot save battery with version " << fileVersion << " (max supported is " << BATTERY_FILE_VERSION
		        << ").\n";
		return;
	}
	}
}

void BatteryHandler::writeBatteryVersion2(const std::string &filename) {
	std::ofstream of(filename, std::ios::out | std::ios::trunc | std::ios::binary);
	of << 'y' << '3' << 'e' << makeVersionNumber(2);

	const auto ramBanks = emulator->memory.getRawRAMBanks();
	const std::string bankString = StateManager::stringifyByteVector(ramBanks);

	auto compressedLength = mz_compressBound(bankString.size());
	auto ptr = std::make_unique<char[]>(compressedLength);

	const auto compressionResult = mz_compress((unsigned char *) ptr.get(), &compressedLength,
	        (const unsigned char *) bankString.data(), bankString.size());

	if (compressionResult != Z_OK) {
		std::cout << "Couldn't compress external RAM for battery file: " << mz_error(compressionResult) << "\n";
		return;
	}

	if (verbose) {
		const double compressionRatio = static_cast<double>(bankString.size()) / compressedLength;
		std::cout << std::dec << "Wrote battery v2, original length = " << bankString.size() << ", compressed length = " << compressedLength << " (ratio = " << compressionRatio << ")\n" << std::hex;
	}

	std::array<char, 9> numBuffer;
	std::snprintf(numBuffer.data(), numBuffer.size(), "%08lu", static_cast<unsigned long>(bankString.size()));

	of.write(numBuffer.data(), numBuffer.size() - 1);
	of.write(ptr.get(), compressedLength);
}

void BatteryHandler::writeBatteryVersion1(const std::string &filename) {
	const auto ramBanks = emulator->memory.getRawRAMBanks();

	std::ofstream of(filename, std::ios::out | std::ios::trunc);
	of << 'y' << '3' << 'e';
	of << makeVersionNumber(1);
	of << StateManager::stringifyByteVector(ramBanks);
}

void BatteryHandler::loadBattery() {
	loadBattery(makeBatteryName());
}

void BatteryHandler::loadBattery(const std::string &fileName) {
	std::ifstream f(fileName, std::ios::in | std::ios::binary);

	if (!f.is_open()) {
		std::cout << "No battery file found; creating new battery \"" << fileName << "\"\n";
		f.close();
		createDefaultBattery();
		return;
	}

	// for validation
	char val1, val2, val3;

	f >> val1;
	f >> val2;
	f >> val3;

	if (val1 != 'y' || val2 != '3' || val3 != 'e') {
		f.close();

		std::cout << "Invalid battery file format, creating new.\n";

		createDefaultBattery();
		return;
	}

	auto versionArr = getVersionArray();

	f.read(versionArr.data(), 8);

	const auto version = std::strtoul(versionArr.data(), nullptr, 10);
	bool loadingOld = false;

	if (version == 0) {
		std::cout << "Invalid battery file; bad version number. Creating new battery.\n";
		createDefaultBattery();
		return;
	} else if (version != BATTERY_FILE_VERSION) {
		loadingOld = true;
	}

	switch (version) {
	case 1:
		loadBatteryVersion1(f);
		break;

	case 2:
		loadBatteryVersion2(f);
		break;

	default:
		std::cout << "Unsupported version number\n";
		return;
	}

	if (loadingOld) {
		// if we got this far and we're loading an old battery, back it up because we're going to overwrite the default
		doSaveBackup(version);
	}

	batteryLoaded_ = true;
}

void BatteryHandler::loadBatteryVersion2(std::ifstream &f) {
	std::array<char, 9> lengthArray;
	f.read(lengthArray.data(), 8);
	auto uncompressedLen = std::strtoul(std::string(lengthArray.data(), 8).c_str(), nullptr, 10);

	if (uncompressedLen == 0) {
		std::cout << "Invalid length in battery file version 2.\n";
		return;
	}

	std::stringstream ss;
	ss << f.rdbuf();

	const std::string compressedData = ss.str();

	if (verbose) {
		std::cout << "Loading v2 battery, expecting length " << std::dec << uncompressedLen 
			<< ", got compressed len: " << compressedData.size() <<  
			", stream len: " << ss.tellp() << std::hex << std::endl;
	}

	auto uncompressedBuffer = std::make_unique<char[]>(uncompressedLen);
	const auto decompressionResult = mz_uncompress((unsigned char *) uncompressedBuffer.get(), &uncompressedLen,
	        (unsigned char *) compressedData.data(), compressedData.size());

	if (decompressionResult != Z_OK) {
		std::cout << "Decompression error in battery v2: " << mz_error(decompressionResult) << std::endl;
		return;
	}

	std::string batteryString(uncompressedBuffer.get(), uncompressedLen);

	std::vector<uint8_t> battery = StateManager::vectorFromByteString(batteryString);

	std::cout << "Successfully loaded v2 battery into memory!\n";
	emulator->memory.overrideRawRAMBanks(battery);
}

void BatteryHandler::loadBatteryVersion1(std::ifstream &f) {
	std::stringstream ss;

	ss << f.rdbuf();

	const std::vector<uint8_t> battery = StateManager::vectorFromByteString(ss.str());
	batteryLoaded_ = true;

	std::cout << "Successfully loaded v1 battery into memory!\n";
	emulator->memory.overrideRawRAMBanks(battery);
}

void BatteryHandler::doSaveBackup(uint64_t oldVersionNumber) {
	const std::string backupFileName = (makeBatteryName() + ".") + (std::to_string(oldVersionNumber) + ".bck");

	std::cout << "Battery loaded was from old version '" << oldVersionNumber << "'. Backing up to \"" << backupFileName
	        << "\".\n";
	writeBattery(backupFileName, oldVersionNumber);
}

void BatteryHandler::writeFinalBattery(uint64_t versionNumber) {
	writeBattery(versionNumber);

	if (!stateLoaded) {
		std::cout << "Writing battery at shutdown.\n";
	}
}

std::string BatteryHandler::makeVersionNumber(uint64_t number) {
	auto numArr = getVersionArray();

	std::snprintf(numArr.data(), numArr.size(), "%08lu", static_cast<unsigned long>(number));

	return std::string(numArr.data());
}

std::string BatteryHandler::makeBatteryName() {
	std::stringstream ss;

	ss << emulator->platformTools->getSaveLocation();

	std::string gameName = emulator->getROM()->header.gameName;
	gameName.erase(std::remove_if(gameName.begin(), gameName.end(), ::isspace), gameName.end());
	std::transform(gameName.begin(), gameName.end(), gameName.begin(), ::tolower);

	ss << gameName << ".y3ebat";

	return ss.str();
}

void BatteryHandler::createDefaultBattery() {
	writeBattery(BATTERY_FILE_VERSION);
}

void BatteryHandler::notifyStateLoad() {
	if (!stateLoaded) {
		std::cout << "WARNING: Because state has been changed, batteries will no longer be saved. A backup of the current battery has been made.\n";
		writeBattery();
	}

	stateLoaded = true;
}

}

