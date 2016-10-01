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

#include <cstring>
#include <cassert>

#include <iostream>
#include <fstream>
#include <sstream>

#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

#include "StateManager.hpp"
#include "Emulator.hpp"
#include "miniz.c"

namespace y3e {
const std::string StateManager::STATE_HEADER = "y3estate";

StateManager::StateManager(Emulator * const emulator_) :
		        emulator { emulator_ },
		        lastActionTime { std::chrono::high_resolution_clock::now() } {

}

void StateManager::saveState(const std::string &filename, uint64_t fileVersion) {
	assert("File version in save state must be <= STATE_FILE_VERSION" && fileVersion <= STATE_FILE_VERSION);

	if (!canTakeAction()) {
		return;
	}

	std::cout << "Saving state to " << filename << std::endl;

	std::ofstream fout(filename, std::ios::out | std::ios::trunc | std::ios::binary);

	if (!fout) {
		std::cout << "Couldn't open file \"" << filename << "\" for saving.\n";
		return;
	}

	std::string fileOutput;

	switch (fileVersion) {
	case 0:
		fileOutput = saveStateVersion0();
		break;

	default:
		std::cout << "Invalid file version when trying to save state.\n";
		return;
	}

	if (fileOutput == "") {
		std::cout << "Error while saving state.\n";
		return;
	}

	auto compressedLen = mz_compressBound(fileOutput.size());
	auto ptr = std::make_unique<char[]>(compressedLen);

	const auto compressionResult = mz_compress((unsigned char *) ptr.get(), &compressedLen,
	        (const unsigned char *) fileOutput.data(), fileOutput.size());

	if (compressionResult != Z_OK) {
		std::cout << "Couldn't compress JSON for state saving: " << mz_error(compressionResult) << std::endl;
		return;
	}

	std::array<char, 9> numBuffer;
	std::snprintf(numBuffer.data(), numBuffer.size(), "%08lu", static_cast<unsigned long>(fileOutput.size()));

	fout.write(STATE_HEADER.c_str(), STATE_HEADER.size());
	fout.write(numBuffer.data(), numBuffer.size() - 1); // subtract 1 to ignore NUL
	fout.write(ptr.get(), compressedLen);

	std::cout << "Saved, compressed data size: " << std::dec << compressedLen << " (down from " << fileOutput.size()
	        << ", ratio: " << ((float) fileOutput.size() / compressedLen) << ")\n" << std::hex;
}

std::string StateManager::saveStateVersion0() {
	const auto &rom = emulator->getROM();

	rapidjson::StringBuffer buffer;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);

	writer.StartObject();

	writer.String("version");
	writer.Uint64(0);

	writer.String("romName");
	writer.String(rom->header.gameName.c_str());

	writer.String("currentROMBank");
	writer.Uint(emulator->memory.getCurrentMemoryBank());

	writer.String("currentRAMBank");
	writer.Uint(emulator->memory.getCurrentRAMBank());

	writer.String("masterInterruptEnable");
	writer.Bool(emulator->interruptHandler.allInterruptsEnabled());

	writer.String("stackDepth");
	writer.Uint(emulator->stack.stackDepth);

	writer.String("registers");
	{
		writer.StartObject();

		writer.String("af");
		writer.Int(emulator->reg.af);

		writer.String("bc");
		writer.Int(emulator->reg.bc);

		writer.String("de");
		writer.Int(emulator->reg.de);

		writer.String("hl");
		writer.Int(emulator->reg.hl);

		writer.String("sp");
		writer.Int(emulator->reg.sp);

		writer.String("pc");
		writer.Int(emulator->reg.pc);

		writer.EndObject();
	}

	{
		writer.String("memory");
		const auto rawMem = emulator->memory.getRawMemory();
		std::stringstream ss;

		for (auto i = (rawMem + 0x8000); i < (rawMem + 0xFFFF); ++i) {
			ss << std::hex << std::setw(2) << std::setfill('0') << (static_cast<uint16_t>(*i) & 0xFF);
		}

		writer.String(ss.str().c_str());
	}

	writer.String("ram_banks");
	writer.String(stringifyByteVector(emulator->memory.getRawRAMBanks()).c_str());

	writer.EndObject();

	const std::string jsonString { buffer.GetString() };

	return jsonString;
}

bool StateManager::loadState(const std::string &filename) {
	if (!canTakeAction()) {
		return false;
	}

	std::cout << "Loading state from " << filename << std::endl;

	std::ifstream fin(filename, std::ios::in | std::ios::binary);

	if (!fin) {
		std::cout << "Couldn't open file \"" << filename << "\" for reading.\n";
		return false;
	}

	auto headerBuffer = std::make_unique<char[]>(STATE_HEADER.size());
	fin.read(headerBuffer.get(), STATE_HEADER.size());
	const std::string loadedHeader(headerBuffer.get(), STATE_HEADER.size());

	if (loadedHeader != STATE_HEADER) {
		std::cout << "Invalid header on state file.\n";
		return false;
	}

	std::array<char, 8> lenBuffer;
	fin.read(lenBuffer.data(), lenBuffer.size());

	std::string lenString(lenBuffer.data(), 8);
	mz_ulong uncompressedLen = std::stoul(lenString, nullptr, 10);

	std::stringstream ss;

	ss << fin.rdbuf();

	std::string compressedData { ss.str() };

	auto uncompressedBuffer = std::make_unique<char[]>(uncompressedLen);
	const auto decompressionResult = mz_uncompress((unsigned char *) uncompressedBuffer.get(), &uncompressedLen,
	        (unsigned char *) compressedData.data(), compressedData.size());

	if (decompressionResult != Z_OK) {
		std::cout << "Decompression error: " << mz_error(decompressionResult) << std::endl;
		return false;
	}

	std::string jsonString(uncompressedBuffer.get(), uncompressedLen);
//	std::cout << "Uncompressed data: " << jsonString << std::endl;

	rapidjson::Document d;

	d.Parse(jsonString.c_str());

	if (d.HasParseError()) {
		std::cout << "Couldn't parse " << filename << ": " << rapidjson::GetParseError_En(d.GetParseError())
		        << std::endl;
		return false;
	}

	const uint64_t versionNumber = d["version"].GetUint64();

	switch (versionNumber) {
	case 0:
		return loadStateVersion0(d);

	default: {
		std::cout << "Invalid file version (" << versionNumber << ") in file \"" << filename << "\".\n";
		return false;
	}
	}
}

bool StateManager::loadStateVersion0(const rapidjson::Document &d) {
	const auto &gameName = emulator->rom->header.gameName;

	if (std::strcmp(d["romName"].GetString(), gameName.c_str()) != 0) {
		std::cout << "Document doesn't contain a save state for the currently loaded game \"" << gameName
		        << "\" (found \"" << d["romName"].GetString() << "\")." << std::endl;
		return false;
	}

	const bool masterInterruptEnable = d["masterInterruptEnable"].GetBool();

	const auto &reg = d["registers"];
	const uint16_t af = (reg["af"].GetUint() & 0xFFFF);

	const uint16_t bc = (reg["bc"].GetUint() & 0xFFFF);
	const uint16_t de = (reg["de"].GetUint() & 0xFFFF);
	const uint16_t hl = (reg["hl"].GetUint() & 0xFFFF);
	const uint16_t sp = (reg["sp"].GetUint() & 0xFFFF);
	const uint16_t pc = (reg["pc"].GetUint() & 0xFFFF);

	const auto currentROMBank = d["currentROMBank"].GetUint();
	const auto currentRAMBank = d["currentRAMBank"].GetUint();

	const auto stackDepth = d["stackDepth"].GetUint();

	const auto memoryString = std::string(d["memory"].GetString());
	const auto ramBanksString = std::string(d["ram_banks"].GetString());

	emulator->batteryHandler.notifyStateLoad();

	std::vector<uint8_t> memory = vectorFromByteString(memoryString);
	std::vector<uint8_t> ramBanks = vectorFromByteString(ramBanksString);

	masterInterruptEnable ?
	        emulator->interruptHandler.enableAllInterrupts() : emulator->interruptHandler.disableAllInterrupts();

	emulator->memory.setMemoryBank(currentROMBank);
	emulator->memory.setRAMBank(currentRAMBank);
	emulator->stack.stackDepth = stackDepth;

	emulator->reg.af = af;
	emulator->reg.bc = bc;
	emulator->reg.de = de;
	emulator->reg.hl = hl;
	emulator->reg.sp = sp;
	emulator->reg.pc = pc;

	for (auto i = 0u; i < memory.size(); ++i) {
		const auto targetAddress = 0x8000 + i;

		if ((targetAddress >= 0x8000 && targetAddress < 0x9800) || targetAddress == 0xFF47 || targetAddress == 0xFF48
		        || targetAddress == 0xFF49) {
			emulator->memory.write(targetAddress, memory[i]);
		} else {
			emulator->memory.rawWrite(targetAddress, memory[i]);
		}
	}

	emulator->memory.overrideRawRAMBanks(ramBanks);
	return true;
}

bool StateManager::canTakeAction() {
	const auto now = std::chrono::high_resolution_clock::now();

	const auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastActionTime).count();

	if (delta > 2500) {
		lastActionTime = now;
		return true;
	}

	return false;
}

std::string StateManager::stringifyByteVector(const std::vector<uint8_t> &vec) {
	std::stringstream ss;

	for (size_t i = 0u; i < vec.size(); ++i) {
		ss << std::hex << std::setw(2) << std::setfill('0') << (static_cast<uint16_t>(vec[i]) & 0xFF);
	}

	return ss.str();
}

std::vector<uint8_t> StateManager::vectorFromByteString(const std::string &str) {
	std::vector<uint8_t> vec;
	vec.reserve(str.size() / 2);

	for (std::string::size_type i = 0; i < str.size(); i += 2) {
		const auto subString = str.substr(i, 2);

		vec.emplace_back(static_cast<uint8_t>(std::strtoul(subString.c_str(), nullptr, 16) & 0xFF));
	}

	return vec;
}

}
