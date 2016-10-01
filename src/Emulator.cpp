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

#include <cstdint>
#include <cctype>

#include <iostream>
#include <unordered_map>
#include <iomanip>
#include <utility>
#include <sstream>
#include <thread>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <thread>

#include "Emulator.hpp"
#include "InstructionParser.hpp"
#include "tinyfiledialogs.h"
#include "Y3EPlatformOptions.hpp"

namespace y3e {

bool Emulator::run() {
	if (renderer == nullptr) {
		std::cout << "Cannot start emulator with null renderer.\n";
		return false;
	}

	if (input == nullptr) {
		std::cout << "Cannot start emulator with null input handler.\n";
		return false;
	}

	if (platformTools == nullptr) {
		std::cout << "Cannot start emulator with null platform tools.\n";
		return false;
	}

	if (audio == nullptr) {
		std::cout << "Warning: Defaulted to do-nothing audio handler.\n";

		audio = std::make_unique<AudioHandler>(this);
	}

	if (serialHandler == nullptr) {
		std::cout << "Warning: defaulted to do-nothing serial handler.\n";

		serialHandler = std::make_unique<SerialHandler>(this);
	}

	if (rom == nullptr) {
		std::cout << "Cannot start emulator without a loaded ROM.\n";
		return false;
	}

	doRun();

	return true;
}

void Emulator::doRun() {
	ArrayInstructionParser ip(this, getROM()->firstBlock.data() + 0x100);

	handleBatteryInitial();

	while (!shouldQuit_) {
		frameBegin();

		if (quitRequested_) {
			// last parameter doesn't seem to work in GTK
			const int choice = tinyfd_messageBox("Quit Y3E?", "Really quit Y3E?", "yesno", "question", 0);

			if (choice == 1) {
				shouldQuit_ = true;
			}

			quitRequested_ = false;
		}

		int state = input->shouldSaveState();

		if (state >= 0) {
			stateManager.saveState(makeStateName(state));
		} else {
			state = input->shouldLoadState();

			if (state >= 0) {
				stateManager.loadState(makeStateName(state));
			}
		}

		uint32_t clocks = 0u;
		if (interruptHandler.handleInterrupts(true)) {
			// if true, an interrupt was handled
			haltMode_ = false;
			clocks += 12;
		}

		if (!haltMode_) {
			clocks += doCPU(ip);
//			std::cout << std::dec << clocks << std::hex << std::endl;
		} else {
			clocks = 4;
		}

		addClocks(clocks);

		gpu.update(clocks);
		memory.handleInternalTimeRegisters(clocks);

		audio->update(clocks);

		serialHandler->update(clocks);

		frameEnd();
	}

	if (cartridgeHasBattery(rom->header.cartridgeType)) {
		batteryHandler.writeFinalBattery();
	}
}

uint8_t Emulator::doCPU(ArrayInstructionParser& ip) {
	//static std::unordered_map<uint8_t, int> parseCount;

	if (stopMode_) {
		if (input->anyPressed()) {
			stopMode_ = false;
		} else {
			return 0;
		}
	}

	uint8_t cyclesTaken = 0u;

	if (reg.pc >= 0x8000) {
		ip.setPos(memory.getRawMemory() + reg.pc);
	} else if (reg.pc >= 0x4000) {
		ip.setPos((uint8_t *) (getROM()->otherBlocks[memory.getCurrentMemoryBank() - 1].data() + (reg.pc - 0x4000)));
	} else {
		ip.setPos((uint8_t *) (getROM()->firstBlock.data() + reg.pc));
	}

	if (hasBreakPoint_) {
		if (reg.pc == breakPoint_) {
			std::cout << "Encountered break point at 0x" << std::hex << breakPoint_ << ", entering debug mode.\n";
			debugMode_ = true;
		}
	}

	auto instr = ip.parse();

	//parseCount[instr->opcode]++;

	if (instr != nullptr) {
		if (debugMode_ || debugPrintAllInstructions_) {
			std::cout << "Parsed: " << instr->getHumanReadable() << " at 0x" << std::hex << reg.pc << " (block "
			        << (reg.pc < 0x4000 ? 0 : memory.getCurrentMemoryBank()) << ") ";
			std::cout << "(GPU Ticks: " << std::dec << gpu.getTicks() << "), ly = " << std::hex
			        << (static_cast<uint16_t>(memory.read(0xFF44)) & 0xFF) << "\n";
		}

		handleDebug();

		instr->execute();
		cyclesTaken = instr->getCyclesTaken();
	} else {
		std::cout << "nullptr instruction!\n";
	}

	return cyclesTaken;
}

void Emulator::signalVBlank() {
	if (cartridgeHasBattery(rom->header.cartridgeType)) {
		batteryHandler.update();
	}

	end = std::chrono::high_resolution_clock::now();

	// in ms
	double deltaTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

	// this is the time to execute an instruction, not render a frame.
	if (fpsLimit_ && deltaTime < fpsTargetGB) {
		const auto diff = (fpsTargetGB - deltaTime);
		const auto diffInMs = diff * 1000.0;

		platformTools->delay(std::floor(diffInMs));
		deltaTime += diff;
	}

	if (timeCounter == Y3E_COUNT_MAX) {
		// we have to be sure to throw away the first values as they tend to be garbage
		// so only the 2nd and beyond fps counts are worth anything
		const double avg = std::accumulate(timeArray.begin(), timeArray.end(), 0.0) / timeCounter;
		timeCounter = 0;

		const double avgFPS = (1.0 / avg);

		if (accumulationCounter != 0) {
			std::cout << "Average FPS: " << avgFPS << std::endl;
		}

		if (accumulationCounter <= averageArray.size() + 1) {
			++accumulationCounter;

			if (accumulationCounter > 1) {
				averageArray[accumulationCounter - 2] = avgFPS;
			}
		}
	}

	timeArray[timeCounter] = deltaTime;
	++timeCounter;

	start = std::chrono::high_resolution_clock::now();
}

void Emulator::setFrameLimitRate(uint8_t val) {
	if (val > 4u) {
		val = 4u;
	}

	fpsTargetGB = 1.0 / (60.0 * (double) val);
}

void Emulator::doHalt() {
	haltMode_ = true;
}

void Emulator::handleDebug() {
	if (debugMode_) {
		bool debugDone = false;

		while (!debugDone) {
			std::cout << "Enter (d)ump, (m)emdump, (n)ewdebug, (c)ontinue, (s)topdebug, debug a(l)l or (q)uit: ";
			std::string in;

			std::cin >> in;

			switch (in[0]) {
			case 'd':
			case 'D': {
				std::cout << "Reg dump:\n" << reg << std::endl;
				break;
			}

			case 'c':
			case 'C': {
				debugDone = true;
				break;
			}

			case 'q':
			case 'Q': {
				signalQuit();
				debugDone = true;
				break;
			}

			case 's':
			case 'S': {
				debugDone = true;
				debugMode_ = false;
				hasBreakPoint_ = false;
				break;
			}

			case 'n':
			case 'N': {
				std::string addressStr;

				std::cout << "Enter address to set next break at (hex): ";
				std::cin >> addressStr;

				std::stringstream ss;
				ss << std::hex << addressStr;

				uint16_t address;
				ss >> address;

				debugDone = true;
				debugMode_ = false;
				breakPoint_ = address;
				hasBreakPoint_ = true;

				break;
			}

			case 'm':
			case 'M': {
				std::string addressStr;

				std::cout << "Enter address to dump (hex): ";
				std::cin >> addressStr;

				std::stringstream ss;
				ss << std::hex << addressStr;

				uint16_t address;
				ss >> address;

				memory.dumpMemory(address);
				break;
			}

			case 'l':
			case 'L': {
				debugPrintAllInstructions_ = true;
				break;
			}

			default: {
				break;
			}
			}
		}
	}
}

void Emulator::debugPrint(const std::string &message) {
//	if (debugPrint_) {
//		std::string actualString(message);
//
//		for (auto it = 0u; it < actualString.length(); ++it) {
//			if (actualString[it] == '<') {
//				const auto ending = actualString.find_first_of('>', it);
//
//				if (ending == actualString.npos) {
//					debug_error_message(message, "no ending found");
//					return;
//				}
//
//				try {
//					const auto sub = actualString.substr(it + 1, ending);
//
//					const auto value = std::stoul(sub, nullptr, 16);
//
//					if (value > UINT16_MAX) {
//						debug_error_message(message, "value out of range");
//						return;
//					}
//
//					const uint16_t memVal = memory.read(static_cast<uint16_t>(value));
//
//					actualString.replace(it, ending + 1 - it, std::to_string(memVal & 0xFF));
//				} catch (...) {
//					debug_error_message(message, "not a valid number");
//					return;
//				}
//			}
//		}
//
//		std::cout << "DBG: " << actualString << std::endl;
//	}
}

void Emulator::loadROM(std::unique_ptr<ROM> &&newRom) {
	rom = std::move(newRom);

// TODO: Re-enable for GBC support
//	if (rom->header.isGBC) {
//		Registers::initializeRegistersGBC(reg);
//	} else {
	Registers::initializeRegistersGB(reg);
//	}

//	stack = std::make_unique<StackHandler>(this);

	memory.resetGB();
}

void Emulator::loadROM(const std::string &fileName) {
	GBROMParser parser;

	auto rom = parser.parseROM(fileName);

	if (rom == nullptr) {
		setErrorState("Couldn't load ROM file.");
		return;
	}

	loadROM(std::move(rom));
}

void Emulator::unloadROM() {
	rom.reset();
}

void Emulator::addBreakPoint(uint16_t breakPoint) {
	breakPoint_ = breakPoint;
	hasBreakPoint_ = true;
}

void Emulator::signalQuit() {
	shouldQuit_ = true;
}

void Emulator::doStop() {
	stopMode_ = true;
}

void Emulator::handleBatteryInitial() {
	if (!cartridgeHasBattery(rom->header.cartridgeType)) {
		return;
	}

	batteryHandler.loadBattery();
}

void Emulator::setErrorState(const std::string &errorMessage) {
	hasError_ = true;
	errorMessage_ = errorMessage;
}

void Emulator::resetErrorState() {
	hasError_ = false;
	errorMessage_ = "No error";
}

bool Emulator::hasError() const {
	return hasError_;
}

std::string Emulator::getErrorMessage() const {
	return errorMessage_;
}

std::string Emulator::makeStateName(int64_t stateNumber) {
	std::stringstream ss;

	ss << platformTools->getSaveLocation();

	std::string gameName = rom->header.gameName;
	gameName.erase(std::remove_if(gameName.begin(), gameName.end(), ::isspace), gameName.end());
	std::transform(gameName.begin(), gameName.end(), gameName.begin(), ::tolower);

	ss << gameName;

	if (stateNumber >= 0) {
		ss << stateNumber;
	}

	ss << ".y3estate";

	return ss.str();
}

double Emulator::getAverageFPS() const {
	if (accumulationCounter == 0) {
		return -1.0;
	}

	double avg = 0.0;

	const size_t limit = std::min((size_t) accumulationCounter - 1, (size_t) averageArray.size());

	for (size_t i = 0; i < limit; ++i) {
		avg += averageArray[i];
	}

	avg /= limit;

	return avg;
}

void Emulator::signalQuitRequested() {
#ifdef Y3E_FORCE_QUIT
	signalQuit();
#else
	quitRequested_ = true;
#endif
}

}
