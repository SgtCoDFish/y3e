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
#include <cstring>
#include <cstdlib>

#include <iostream>
#include <iomanip>

#include "MemoryManager.hpp"
#include "ROM.hpp"
#include "Emulator.hpp"

namespace y3e {

MemoryManager::MemoryManager(Emulator * emulator_) :
		        emulator { emulator_ } {
	// TODO: Fix this to change based on the inserted cartridge
//	ramBanks.resize(ramBankSize * 4);
	ramBanks.resize(2 << 16);
}

uint8_t MemoryManager::read(uint16_t address) {
	/*
	 * TODO:
	 * http://gbdev.gg8.se/wiki/articles/Gameboy_sound_hardware#Register_Reading
	 * Audio registers need special handling technically when being read.
	 */

	if (address >= 0x4000 && address < 0x8000) {
		// Read from the selected ROM bank.
		return (emulator->getROM()->otherBlocks[getCurrentMemoryBank() - 1].at(address - 0x4000));
	} else if (address >= 0xA000 && address < 0xC000) {
		return getFromRAMBank(address);
	}

	if (address >= 0xE000 && address < 0xFE00) {
		// echo RAM, accesses the same as 0xC000 to 0xDE00.
		address -= 0x2000;
	}

	if (address >= 0xFF10 && address < 0xFF40) {
		// audio might be different if the audio handler takes control
		uint8_t temp = 0u;
		if (emulator->audio->handleAudioRegisterRead(address, temp)) {
			// returns true if the audio subsystem manages its own memory
			return temp;
		}
	}

	return memory[address];
}

void MemoryManager::write(uint16_t address, uint8_t value) {
	// HANDLE AUDIO REGISTERS
	// this could probably be impoved using a hash map of some kind
	// but that's an optimisation for the future.
	// + it's likely to be optimised to some kind of hash table by
	// the compiler (in release modes)
	if (address >= 0xFF10 && address < 0xFF40) {
		emulator->audio->handleAudioRegisterWrite(address, value);

		if (address == 0xFF26) {
			const auto existing = memory[address];
			memory[address] = (value & 0x80) + (existing & 0xF);
			return;
		}
	}

	// HANDLE SERIAL REGISTERS
	if (address == 0xFF01 || address == 0xFF02) {
//		std::cout << "Serial write @ 0x" << std::hex << address << ": 0x" << (static_cast<uint16_t>(value) & 0xFF)
//		        << std::endl;
		memory[address] = value;
		emulator->serialHandler->handleMemoryWrite(address, value);
		return;
	}

	if (address < 0xFE00 && address >= 0xE000) {
		// echo RAM, accesses the same as 0xC000 to 0xDE00.
		address -= 0x2000;
	} else if (address == 0xFF00) {
		handleInputRegisterWrite(value);
	} else if (address >= 0xA000 && address < 0xC000) {
		writeToRAMBank(address, value);
	} else if (address < 0x8000) {
		handleMBC(address, value);
		return;
	}

	// HANDLE TIMER REGISTERS
	if (address == 0xFF04) {
		// handle DIV
		// DIV register gets reset on write
		memory[address] = 0;
		return;
	} else if (address == 0xFF05) {
		// handle TIMA; nothing special
	} else if (address == 0xFF06) {
		// handle TMA; nothing special
	} else if (address == 0xFF07) {
		// handle TAC
		handleTACWrite(value);
	}

	// HANDLE INTERRUPT REGISTERS
	if (address == 0xFF0F) {
		// handle IF; nothing special
		emulator->interruptHandler.handleIFWrite(value);
	} else if (address == 0xFFFF) {
		emulator->interruptHandler.handleIEWrite(value);
	}

	// HANDLE GRAPHICS REGISTERS AND VRAM
	if (address == 0xFF40) {
		// LCDC
		emulator->gpu.handleLCDCWrite(value);
	} else if (address == 0xFF41) {
		// handle STAT
	} else if (address == 0xFF42) {
		// handle SCY
	} else if (address == 0xFF43) {
		// handle SCX
	} else if (address == 0xFF44) {
		// handle LY
		// writing to FF44 resets the current scanline rather than overwriting.
		memory[address] = 0;
		emulator->gpu.handleFF44Write();
		return;
	} else if (address == 0xFF45) {
		// handle LYC
	} else if (address == 0xFF46) {
		handleDMA(value);
	} else if (address == 0xFF47) {
		// handle BGP
		emulator->paletteManager.handleBGPaletteRegisterWrite(value);
	} else if (address == 0xFF48) {
		// handle OBP0
		emulator->paletteManager.handleOBJ0PaletteRegisterWrite(value);
	} else if (address == 0xFF49) {
		// handle OBP1
		emulator->paletteManager.handleOBJ1PaletteRegisterWrite(value);
	} else if (address == 0xFF4A) {
		// handle WY
//		std::cout << "WY changed to " << std::dec << (uint16_t(value) & 0xFF) << std::hex << std::endl;
	} else if (address == 0xFF4B) {
		// handle WX
//		std::cout << "WX changed to " << std::dec << (uint16_t(value) & 0xFF) << std::hex << std::endl;
	} else if (address >= 0xFE00 && address < 0xFEA0) {
		emulator->gpu.setOAMByte(address, value);
	} else if (address < 0xA000 && address >= 0x8000) {
		emulator->gpu.notifyVRAMWrite(address);
	}

	memory[address] = value;
}

void MemoryManager::rawWrite(uint16_t address, uint8_t value) {
	memory[address] = value;
}

uint16_t MemoryManager::read16(uint16_t address) {
	const uint16_t byte1 = read(address + 0);
	const uint16_t byte2 = read(address + 1);

	const uint16_t value = byte1 | (byte2 << 8);

	if (emulator->debugEnabled()) {
		std::cout << "Read 0x" << std::hex << value << " from 0x" << address << std::endl;
	}

	return value;
}

void MemoryManager::write16(uint16_t address, uint16_t value) {
	if (emulator->debugEnabled()) {
		std::cout << "Writing 0x" << std::hex << value << " to 0x" << address << std::endl;
	}

	const uint8_t byte1 = (value & 0x00FF);
	const uint8_t byte2 = (value & 0xFF00) >> 8;

	write(address + 0, byte1);
	write(address + 1, byte2);
}

uint8_t MemoryManager::incLY() {
	(*regLY)++;

	if (*regLY > 153) {
		*regLY = 0;
	}

	return *regLY;
}

void MemoryManager::dumpMemory(uint16_t address, uint16_t numBytes) {
	constexpr const int BREAK_AT = 8;
	int lineBreaker = BREAK_AT;
	unsigned int lineCount = 0;

	if (numBytes % 2 != 0) {
		numBytes++;
	}

	for (int32_t i = 0; i < numBytes; i += 2) {
		if (i + 2 > 0x8000) {
			break;
		}

		if (lineBreaker >= BREAK_AT) {
			std::cout << std::endl << "0x" << std::hex << (address + BREAK_AT * lineCount) << ": ";
			lineCount++;
			lineBreaker = 0;
		}

		const auto c = readRaw(address + i + 1);

		std::cout << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(c & 0xFF);

		const auto c2 = readRaw(address + i + 0);

		std::cout << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(c2 & 0xFF) << " ";

		lineBreaker += 2;
	}

	std::cout << std::endl;
}

void MemoryManager::handleInputRegisterWrite(uint8_t &value) {
	static const uint8_t bothRails = 0b00110000;
	static const uint8_t rail4 = 0b00010000;
	static const uint8_t rail5 = 0b00100000;

	if ((value & bothRails) == bothRails) {
		// if both bits set, no meaning to lower nibble
		value = 0xFF;
	} else if ((value & rail4) == rail4) {
		value = emulator->input->getRail4Byte();
	} else if ((value & rail5) == rail5) {
		value = emulator->input->getRail5Byte();
	} else {
		// neither bits. no meaning to lower nibble
		value = 0xFF;
	}
}

void MemoryManager::handleDMA(uint8_t value) {
	const uint16_t srcAddress = (value << 8);

	for (uint16_t i = 0; i < 0xA0; i++) {
		write(0xFE00 + i, read(srcAddress + i));
	}
}

void MemoryManager::handleTACWrite(const uint8_t &value) {
	timerEnabled = value & 0x4; // bit 2 = timer enable

	switch (value & 0x3) { // bits 0 and 1 = clock mode select
	case 0: {
		timerClockMode = TimerClockMode::MODE_4096Hz;
		break;
	}

	case 1: {
		timerClockMode = TimerClockMode::MODE_262144Hz;
		break;
	}

	case 2: {
		timerClockMode = TimerClockMode::MODE_65536Hz;
		break;
	}

	case 3: {
		timerClockMode = TimerClockMode::MODE_16384Hz;
		break;
	}

		// no other cases possible
	default: {
		std::abort();
		break;
	}
	}
}

void MemoryManager::handleInternalTimeRegisters(const uint16_t &clocks, bool isSuperGB) {
	// ignore isSuperGB for now
	// written following the specification of:
	// https://github.com/zid/gameboy/blob/master/timer.c

	timerCycles += clocks; // 4 cycles = 1 timer tick

	if (timerCycles >= 16) {
		timerCycles -= 16;

		// do a "tick"
		++timerTicks;

		if (timerTicks % 16 == 0) {
			++memory[0xFF04];
		}

		if (timerEnabled) {
			// the clock mode is the number of ticks for one increase in the counter (TIMA, 0xFF05)
			if ((timerTicks % static_cast<std::underlying_type<TimerClockMode>::type>(timerClockMode)) == 0) {
				uint16_t counter = memory[0xFF05];
				++counter;

				if (counter == 0x100) {
					// on overflow, request an interrupt and then set TIMA to TMA (modulo)
					memory[0xFF05] = memory[0xFF06];
					emulator->interruptHandler.triggerInterrupt(InterruptType::TIMER_OVERFLOW);
				} else {
					memory[0xFF05] = counter;
				}
			}
		}

		if (timerTicks >= 64) {
			// 64 is the largest it can get for any TimerClockMode, so reset here to avoid integer overflow
			// which is admittedly unlikely but worth preparing for.
			timerTicks -= 64;
		}
	}
}

void MemoryManager::handleMBC(const uint16_t &address, const uint8_t &value) {
	switch (emulator->getROM()->header.mbcType) {
	case MBCType::NONE: {
//		std::cout << "Warning: write to address < 0x8000 with no MBC type (addr = " << address << ")\n";
//		memory[address] = value;
		break;
	}

	case MBCType::MBC1: {
		if (address < 0x2000) {
			const auto lower4 = value & 0b1111;

			if (lower4 == 0xA) {
				ramEnabled_ = true;
			} else if (lower4 == 0x0) {
				ramEnabled_ = false;
			}
		} else if (address < 0x4000) {
			// the lower 5 bits of the ROM bank number

			// a quirk is that 0 is always translated into 1,
			// which means that the following rom banks are inaccessible
			// 0x0  -> 0x01
			// 0x20 -> 0x21
			// 0x40 -> 0x41
			// 0x60 -> 0x60
			const uint8_t low5Select = 0b11111;
			uint8_t lower5 = value & low5Select;

			if (lower5 == 0x00) {
				lower5 = 0x01;
			}

			const uint8_t upper3 = memoryBank & 0b01100000;

			memoryBank = upper3 + lower5;
		} else if (address < 0x6000) {
			if (mbc1Mode == 0x00) {
				// ROM mode, select bits 5+6 for bank number
				const uint8_t bits = value & 0b11;

				memoryBank = (memoryBank & 0b00011111) + (bits << 5);
			} else if (mbc1Mode == 0x01) {
				// TODO: Check?
				ramBank = value & 0b11;
			} else {
				std::cout << "Warning: invalid mbc1mode: " << mbc1Mode << std::endl;
			}
		} else if (address < 0x8000) {
			// 0x00 = ROM banking mode (default) (only RAM bank 1)
			// 0x01 = RAM banking mode (all RAM, only ROM 0x00-0x1F)
			if (value <= 0x01) {
				mbc1Mode = value;
			} else {
//				std::cout << "Warning: attempting to write invalid mbc1mode: " << (static_cast<uint16_t>(value) & 0xFF) << " at: 0x" << address
//				        << std::endl;
			}
		}

		break;
	}

	case MBCType::MBC2: {
		std::cout << "Warning: MBC2 NYI. Things will break.\n";
		break;
	}

	case MBCType::MBC3: {
		if (address < 0x2000) {
			// RAM/Timer enable
			// 0xA in lower 4 bits enables, 0x0 disables
			const auto lower4 = value & 0xF;
			if (lower4 == 0xA) {
				ramEnabled_ = true;
			} else if (lower4 == 0x0) {
				ramEnabled_ = false;
			}

			// ignore other possible values
		} else if (address < 0x4000) {
			// 7 bit bank number
			uint8_t bankNumber = value & 0b01111111;

			if (bankNumber == 0) {
				bankNumber = 1;
			}

			memoryBank = bankNumber;
		} else if (address < 0x6000) {
			if (value <= 3) {
				ramBank = value;
				rtcRegisterEnabled = false;
			} else if (value >= 0x8 && value <= 0xC) {
				rtcRegisterEnabled = true;

				handleRTCRegisterEnable(value);
			}
//				else {
//					std::cout << "MBC3: Invalid write to ram bank selection, value = " << value << std::endl;
//				}
		} else if (address < 0x8000) {
			// technically always less than 0x8000 because we check before this function is called
			// but we'll be explicit for clarity

			// latch clock data: current RTC values are latched into memory in the RTC registers
			// requires the program to write 0x00 and then 0x01 to this memory space.

			handleRTCLatch(value);
		}

		break;
	}

	case MBCType::MBC5: {
		if (address < 0x2000) {
			const auto lower4 = value & 0xF;

			if (lower4 == 0xA) {
				ramEnabled_ = true;
			} else if (lower4 == 0x0) {
				ramEnabled_ = false;
			}

			// ignore other possible values
		} else if (address < 0x3000) {
			memoryBank &= 0x100; // keep 9th bit

			memoryBank += value; // value is 8-bit so this is safe
		} else if (address < 0x4000) {
			memoryBank &= 0xFF;

			memoryBank += (value & 1) << 9;
		} else if (address < 0x6000) {
			ramBank = value & 0xF;
		}

		break;
	}
	}
}

void MemoryManager::handleRTCRegisterEnable(const uint8_t &value) {
	return;
}

void MemoryManager::handleRTCLatch(const uint8_t &value) {
	if (value == 0x01) {
		if (lastRTCLatchWrite == 0x00) {
			// must write 0x00 and then 0x01 to latch RTC

		}
	}
}

uint8_t MemoryManager::getFromRAMBank(const uint16_t &address) const {
	const uint16_t offset = address - 0xA000;

	return ramBanks[ramBankSize * ramBank + offset];
}

void MemoryManager::overrideRawRAMBanks(const std::vector<uint8_t> &newBanks) {
	if (newBanks.size() != ramBanks.size()) {
//		std::cout << "WARNING: Overwriting RAM banks with different size.\n";
//		std::cout << "This is highly unlikely to have the desired effect." << std::endl;
	}

	ramBanks.clear();

	ramBanks.assign(newBanks.begin(), newBanks.end());
}

void MemoryManager::writeToRAMBank(const uint16_t &address, const uint8_t &value) {
	const uint16_t offset = address - 0xA000;

//	std::cout << "Write to ram bank " << uint16_t(ramBank) << " at 0x" << std::hex << address << " with value "
//	        << (static_cast<uint16_t>(value) & 0xFF) << "\n";

	ramBanks[ramBankSize * ramBank + offset] = value;
	ramDirty_ = true;
}

void MemoryManager::resetGB() {
	memory[0xFF26] = 0xF1; // NR52
	resetCommon();
}

void MemoryManager::resetSGB() {
	memory[0xFF26] = 0xF0; // NR52
	resetCommon();
}

void MemoryManager::resetCommon() {
	memory[0xFF00] = 0xCF; // Joypad Reg
	memory[0xFF01] = 0x00; // Serial Data
	memory[0xFF02] = 0x7E; // Serial Ctrl
	memory[0xFF03] = 0xFF; // ??
	memory[0xFF04] = 0xAF; // div

	memory[0xFF05] = 0x00; // TIMA
	memory[0xFF06] = 0x00; // TMA
	memory[0xFF07] = 0x00; // TAC

	memory[0xFF10] = 0x80; // NR10
	memory[0xFF11] = 0xBF; // NR11
	memory[0xFF12] = 0xF3; // NR12
	memory[0xFF14] = 0xBF; // NR14
	memory[0xFF16] = 0x3F; // NR21
	memory[0xFF17] = 0x00; // NR22
	memory[0xFF19] = 0xBF; // NR24
	memory[0xFF1A] = 0x7F; // NR30
	memory[0xFF1B] = 0xFF; // NR31
	memory[0xFF1C] = 0x9F; // NR32
	memory[0xFF1E] = 0xBF; // NR33
	memory[0xFF20] = 0xFF; // NR41
	memory[0xFF21] = 0x00; // NR42
	memory[0xFF22] = 0x00; // NR43
	memory[0xFF23] = 0xBF; // NR30
	memory[0xFF24] = 0x77; // NR50
	memory[0xFF25] = 0xF3; // NR51

	memory[0xFF40] = 0x91; // LCDC
	memory[0xFF42] = 0x00; // SCY
	memory[0xFF43] = 0x00; // SCX
	memory[0xFF45] = 0x00; // LYC
	memory[0xFF47] = 0xFC; // BGP
	memory[0xFF48] = 0xFF; // OBP0
	memory[0xFF49] = 0xFF; // OBP1
	memory[0xFF4A] = 0x00; // WY
	memory[0xFF4B] = 0x00; // WX
	memory[0xFFFF] = 0x00; // IE

	auto rom_ = emulator->getROM();

	std::memcpy(memory.data(), rom_->firstBlock.data(), rom_->firstBlock.size());
}

uint16_t MemoryManager::readRaw(uint16_t address) {
	return memory[address];
}

}
