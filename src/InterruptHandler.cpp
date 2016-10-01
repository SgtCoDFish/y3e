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

#include "InterruptHandler.hpp"
#include "Emulator.hpp"

namespace y3e {

InterruptHandler::InterruptHandler(Emulator * emulator_) :
		        emulator { emulator_ } {

}

void InterruptHandler::enableAllInterrupts() {
//	std::cout.flush();
//	std::cout << "All interrupts enabled at PC: " << emulator->reg.pc << std::endl;
	interruptsEnabled_ = true;
	pendingCounter = 2;
}

void InterruptHandler::disableAllInterrupts() {
//	std::cout.flush();
//	std::cout << "All interrupts disabled at PC: " << emulator->reg.pc << std::endl;
	interruptsEnabled_ = false;
}

void InterruptHandler::enableInterrupt(InterruptType type) {
	emulator->memory.write(0xFFFF, emulator->memory.read(0xFFFF) | resolveRegisterBit(type));
}

void InterruptHandler::disableInterrupt(InterruptType type) {
	emulator->memory.write(0xFFFF, emulator->memory.read(0xFFFF) & (~resolveRegisterBit(type)));
}

bool InterruptHandler::interruptEnabled(InterruptType type) {
	return emulator->memory.read(0xFFFF) & resolveRegisterBit(type);
}

InterruptHandler::interrupt_type_t InterruptHandler::resolveRegisterBit(InterruptType type) const {
	return (1 << static_cast<interrupt_type_t>(type));
}

void InterruptHandler::triggerInterrupt(InterruptType type) {
	if (interruptsEnabled_) {
		if (interruptEnabled(type)) {
			emulator->memory.write(0xFF0F, emulator->memory.read(0xFF0F) | resolveRegisterBit(type));
		}
	}
}

bool InterruptHandler::handleInterrupts(bool debug_) {
	if (pendingCounter >= 2) {
		pendingCounter--;
		return false;
	}

	pendingCounter = 0;

	bool ret = false;

	if (!allInterruptsEnabled()) {
		// if interrupts are disabled, we need to resume from a halt
		return true;
	}
	const uint8_t ifReg = emulator->memory.read(0xFF0F);

	if (ifReg != 0x00) {
		// First, check if any interrupt is going to be handled
		if (ifReg & emulator->memory.read(0xFFFF)) {
			ret = true;
			// this reduces code duplication across each if statement below.

			emulator->addClocks(20);
			emulator->setHalt(false);
			disableAllInterrupts();
			emulator->stack.push(emulator->reg.pc);
//				std::cout << "Interrupt: " << (uint16_t(ifReg) & 0xFF) << std::endl;

			if ((ifReg & resolveRegisterBit(InterruptType::VBLANK)) && interruptEnabled(InterruptType::VBLANK)) {
				emulator->memory.write(0xFF0F, ifReg & (~resolveRegisterBit(InterruptType::VBLANK)));
				emulator->reg.pc = 0x0040;
			} else if ((ifReg & resolveRegisterBit(InterruptType::LCDC)) && interruptEnabled(InterruptType::LCDC)) {
				emulator->memory.write(0xFF0F, ifReg & (~resolveRegisterBit(InterruptType::LCDC)));
				emulator->reg.pc = 0x0048;
			} else if ((ifReg & resolveRegisterBit(InterruptType::TIMER_OVERFLOW))
			        && interruptEnabled(InterruptType::TIMER_OVERFLOW)) {
				emulator->memory.write(0xFF0F, ifReg & (~resolveRegisterBit(InterruptType::TIMER_OVERFLOW)));
				emulator->reg.pc = 0x0050;
			} else if ((ifReg & resolveRegisterBit(InterruptType::SERIAL_IO_COMPLETE))
			        && interruptEnabled(InterruptType::SERIAL_IO_COMPLETE)) {
				emulator->memory.write(0xFF0F, ifReg & (~resolveRegisterBit(InterruptType::SERIAL_IO_COMPLETE)));
				emulator->reg.pc = 0x0058;
			} else if ((ifReg & resolveRegisterBit(InterruptType::PIN_TRANSITION))
			        && interruptEnabled(InterruptType::PIN_TRANSITION)) {
				emulator->memory.write(0xFF0F, ifReg & (~resolveRegisterBit(InterruptType::PIN_TRANSITION)));
				emulator->reg.pc = 0x0060;
			}
		}
	}

	return ret;
}

void InterruptHandler::handleIFWrite(const uint8_t &value) {
	// do nothing for now
}

void InterruptHandler::handleIEWrite(const uint8_t &value) {
	// do nothing for now
}

}
