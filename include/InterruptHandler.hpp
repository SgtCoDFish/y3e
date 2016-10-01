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

#ifndef CODE_TRUNK_INCLUDE_INTERRUPTHANDLER_HPP_
#define CODE_TRUNK_INCLUDE_INTERRUPTHANDLER_HPP_

#include <cstdint>

#include <type_traits>

namespace y3e {
class Emulator;

/**
 * Abstraction of various types of interrupt.
 */
enum class InterruptType
	: uint8_t {
		VBLANK = 0,           //!< VBLANK
	LCDC = 1,              //!< LCDC
	TIMER_OVERFLOW = 2,    //!< TIMER_OVERFLOW
	SERIAL_IO_COMPLETE = 3,    //!< SERIAL_IO_COMPLETE
	PIN_TRANSITION = 4     //!< PIN_TRANSITION
};

/**
 * Handles interrupt processing and the manipulation of the program counter as appropriate.
 */
class InterruptHandler {
public:
	explicit InterruptHandler(Emulator * emulator_);
	~InterruptHandler() = default;

	/**
	 * Sets the "master interrupt enable" flag which disables all interrupts.
	 *
	 * Note that saving state requires that this flag be saved to a file.
	 */
	void enableAllInterrupts();

	/**
	 * Resets the "master interrupt enable" flag which disables all interrupts.
	 *
	 * Note that saving state requires that this flag be saved to a file.
	 */
	void disableAllInterrupts();

	/**
	 * Enables a specific type of interrupt; this is largely set through instruction parsing
	 * and should likely not be called externally.
	 */
	void enableInterrupt(InterruptType type);

	/**
	 * Disables a specific type of interrupt; this is largely set through instruction parsing
	 * and should likely not be called externally.
	 */
	void disableInterrupt(InterruptType type);

	/**
	 * Return true if the given interrupt type is enabled.
	 */
	bool interruptEnabled(InterruptType type);

	/**
	 * Starts execution of a given interrupt; this will affect emulation and should probably not
	 * be called externally.
	 */
	void triggerInterrupt(InterruptType type);

	bool allInterruptsEnabled() const {
		return interruptsEnabled_;
	}

	/**
	 * Should be called after each parsed instruction to handle instructions if needed.
	 */
	bool handleInterrupts(bool debug_);

	/**
	 * Handle a write to 0xFF0F
	 */
	void handleIFWrite(const uint8_t &value);

	/**
	 * Handle a write to 0xFFFF
	 */
	void handleIEWrite(const uint8_t &value);

private:
	Emulator * const emulator;

	bool interruptsEnabled_ = true;

	using interrupt_type_t = std::underlying_type<InterruptType>::type;

	interrupt_type_t resolveRegisterBit(InterruptType type) const;

	void writeIF(uint8_t value);

	int8_t pendingCounter = 0;
};

}

#endif /* CODE_TRUNK_INCLUDE_INTERRUPTHANDLER_HPP_ */
