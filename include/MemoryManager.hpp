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

#ifndef CODE_TRUNK_INCLUDE_MEMORYMANAGER_HPP_
#define CODE_TRUNK_INCLUDE_MEMORYMANAGER_HPP_

#include <cstdint>

#include <vector>
#include <array>

namespace y3e {
class Emulator;

/**
 * Abstraction of the various clock modes of the Game Boy timer.
 */
enum class TimerClockMode {
	MODE_4096Hz = 64, //!< MODE_4096Hz
	MODE_262144Hz = 1, //!< MODE_262144Hz
	MODE_65536Hz = 4, //!< MODE_65536Hz
	MODE_16384Hz = 16, //!< MODE_16384Hz
};

/**
 * Holds data for time registers on RTC enabled carts.
 */
struct RTCRegister {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t days;

	uint8_t mix;

	uint16_t getDays() const {
		// 9 bit number of days
		return days + ((mix & 1) << 9);
	}

	bool isHalted() const {
		return mix & 0b01000000;
	}

	bool hasCarry() const {
		return mix & 0b10000000;
	}
};

/**
 * Models the whole memory map of the Game Boy. Note that this involves creating an array of size at least 65k
 * and so this class shouldn't be instantiated much (ideally once per program execution).
 */
class MemoryManager {
public:
	explicit MemoryManager(Emulator * emulator_);
	~MemoryManager() = default;

	/**
	 * Returns a value from the emulated memory at the given address.
	 *
	 * Note that this has side effects for certain memory addresses and as such external use should be limited
	 * to maximise emulation accuracy.
	 *
	 * Also note that some memory addresses will not return correct values as some addresses are mapped
	 * dynamically to other areas of memory to support MBCs.
	 */
	uint8_t read(uint16_t address);

	/**
	 * Writes a value into memory at the given address.
	 *
	 * Note that this has side effects for certain memory addresses and as such external use should be limited
	 * to maximise emulation accuracy. For certain values the side effects can be very important.
	 */
	void write(uint16_t address, uint8_t value);

	/**
	 * Write a raw value into memory.
	 *
	 * Shouldn't normally be used as this circumvents several side effects of writing to memory.
	 */
	void rawWrite(uint16_t address, uint8_t value);

	/**
	 * Reads a 16 bit value from memory. This should be used sparingly as side effects from reading are
	 * preserved.
	 */
	uint16_t read16(uint16_t address);

	/**
	 * Writes a 16 bit value into memory. This should be used sparingly as side effects from writing are
	 * preserved.
	 */
	void write16(uint16_t address, uint16_t value);

	/**
	 * Sets the memory as it should be initialised after a reset on the original GB
	 */
	void resetGB();

	/**
	 * Sets the memory as it should be initialised after a reset on the Super GB.
	 */
	void resetSGB();

	/**
	 * Increment the LY register at 0xFF44 and reset if it goes past its max of 153.
	 */
	uint8_t incLY();

	/**
	 * Handle DMA transfer of memory to OAM.
	 */
	void handleDMA(uint8_t value);

	/**
	 * For debugging: dump numBytes bytes of memory at given address.
	 */
	void dumpMemory(uint16_t address, uint16_t numBytes = 0x10);

	uint8_t getCurrentMemoryBank() const {
		return memoryBank;
	}

	/**
	 * Note that using this excessively will lead to confusing code. Games have all the tools neccessary to
	 * control memory banks, and using this will change it internally.
	 */
	void setMemoryBank(const uint8_t &memoryBank_) {
		memoryBank = memoryBank_;
	}

	uint8_t getCurrentRAMBank() const {
		return ramBank;
	}

	bool isRAMDirty() const {
		return ramDirty_;
	}

	void resetRAMDirtyFlag() {
		ramDirty_ = false;
	}

	/**
	 * Note that using this excessively will lead to confusing code. Games have all the tools neccessary to
	 * control memory banks, and using this will change it internally.
	 */
	void setRAMBank(const uint8_t &ramBank_) {
		ramBank = ramBank_;
	}

	/**
	 * Returns a raw pointer to the memory data. Changing the data returned is likely to break the emulator.
	 * @return
	 */
	uint8_t * getRawMemory() {
		return memory.data();
	}

	/**
	 * Returns a copy of the internal store of the ram banks in the MBC cartridge.
	 */
	std::vector<uint8_t> getRawRAMBanks() const {
		return ramBanks;
	}

	size_t getRAMBankSize() const {
		return ramBanks.size();
	}

	/**
	 * Not intended for general use, this function is mainly to facilitate state saving and loading.
	 * It overwrites the entirety of currently loaded RAM banks with the contents of newBanks, with
	 * minimal sanity checking. Using this incorrectly will lead to very strange, incorrect
	 * results in the emulator.
	 */
	void overrideRawRAMBanks(const std::vector<uint8_t> &newBanks);

	/**
	 * Increments DIV (0xFF04) and TIMA (0xFF05) as appropriate,
	 * and handles overflow as needed.
	 */
	void handleInternalTimeRegisters(const uint16_t &clocks, bool isSuperGB = false);

	/**
	 * Handles a write to 0xFF07 (TAC) which controls internal timer functionality.
	 */
	void handleTACWrite(const uint8_t &value);

	/**
	 * Handles a write to memory addresses less than 0x8000.
	 */
	void handleMBC(const uint16_t &address, const uint8_t &value);

	/**
	 * Latches the current RTC values into the RTC register if 0x00 and 0x01 are written to
	 * 0x6000-0x7FFF.
	 */
	void handleRTCLatch(const uint8_t &value);

	/**
	 * Maps the given RTC register into memory from 0xA000-0xBFFF.
	 */
	void handleRTCRegisterEnable(const uint8_t &value);

	/**
	 * Aware of RAM banking concerns
	 */
	uint8_t getFromRAMBank(const uint16_t &address) const;

	/**
	 * Aware of RAM banking concerns.
	 */
	void writeToRAMBank(const uint16_t &address, const uint8_t &value);

private:
	Emulator * const emulator;

	// the whole memory of the GB
	// note that certain areas will not match up entirely accurately, as read-only
	// areas are not copied for performance reasons.
	// this means that 0x0000-0x8000 for example may not contain anything.
	std::array<uint8_t, 0x10000> memory;

	// which memory bank is currently loaded at 0x4000
	uint16_t memoryBank = 1;

	// external RAM must be enabled first
	bool ramEnabled_ = false;

	// which RAM bank, if any, is loaded
	uint8_t ramBank = 0;

	// Must be accessed using the ram bank size obtained from the cartridge
	std::vector<uint8_t> ramBanks;

	// Set to true when RAM is written and can be reset by the emulator.
	bool ramDirty_ = false;

	// the size of one RAM bank
	uint16_t ramBankSize = 8192;

	// only relevant for MBC1; writing to 0x6000 <= x < 0x8000 changes the mode
	uint8_t mbc1Mode = 0x00;

	// Only relevant for timer-enabled carts; allows the game to read/write
	// the RTC register at 0xA000-0xBFFF.
	bool rtcRegisterEnabled = false;

	// if set to 0x00 (a 0x00 write occurred to 0x6000-0x7FFF) and 0x01 is then
	// written to the same area, RTC is latched to the RTC register until the same
	// happens again
	uint8_t lastRTCLatchWrite = 0xFF;

	/*
	 * Pointers referring to different parts of memory for reading,
	 * see http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf page 8.
	 *
	 * Areas for writing are quite different to these pointers
	 */
	// ROM bank #0, always the first bank of memory in the cartridge
	// 16k
	uint8_t * romBank0 = memory.data() + 0x0000;

	// Switchable ROM bank; can point to various different areas in memory
	// depending on which was chosen and the MBC type.
	// Without MBC, refers to the second bank of data on the cart.
	// 16k
	uint8_t * romBankSwitch = memory.data() + 0x4000;

	// Video RAM
	// 8k
	uint8_t * vram = memory.data() + 0x8000;

	// Switchable RAM bank, may point to various different parts of data depending on what
	// RAM bank is selected
	// 8k
	uint8_t * ramBankSwitch = memory.data() + 0xA000;

	// Internal RAM of the machine
	// 8k
	uint8_t * internalRAM = memory.data() + 0xC000;

	// Register holding the current verical line on the LCD to
	// which data is being transferred
	uint8_t * const regLY = memory.data() + 0xFF44;

	// some more in between

	// high ram, originally meant to be for stack but often repurposed for micro, high speed RAM
	// 128B
	uint8_t * highRAM = memory.data() + 0xFF80;

	bool timerEnabled = false;
	TimerClockMode timerClockMode = TimerClockMode::MODE_4096Hz;

	// the number of cycles the timer has gone through
	int64_t timerCycles = 0x00;

	// one tick is 1/262144 seconds, used to track timer functions
	int64_t timerTicks = 0x00;

	void resetCommon();

	/**
	 * Since reading from some memory areas may have side effects, this reads memory without
	 * any side effects.
	 *
	 * Primarily for debugging.
	 */
	uint16_t readRaw(uint16_t address);

	/*
	 * Input rail select
	 * If bit 4 (0b00010000) is set, the lower nibble represents START, SELECT, B, A
	 * If bit 5 (0b00100000) is set, the lower nibble represents DOWN, UP, LEFT, RIGHT
	 */
	void handleInputRegisterWrite(uint8_t &value);
};

}

#endif /* CODE_TRUNK_INCLUDE_MEMORYMANAGER_HPP_ */
