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

#include <iostream>

#include "Emulator.hpp"
#include "Instructions.hpp"

namespace y3e {

void NOP::executeImpl() {
}

void STOP::executeImpl() {
	emulator->doStop();
}

void HALT::executeImpl() {
	if (emulator->interruptHandler.allInterruptsEnabled()) {
		emulator->doHalt();
//		emulator->interruptHandler.enableAllInterrupts();
	}
//		else {
//		emulator->reg.pc++;
//	}
}

void EI::executeImpl() {
	Instruction::queueEI();
}

void DI::executeImpl() {
	Instruction::queueDI();
}

void CCF::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.setFCarry(!(emulator->reg.isFCarry()));
}

void DAA::executeImpl() {
	uint16_t endVal = emulator->reg.a;

	const bool halfCarry = emulator->reg.isFHalfCarry();
	const bool carry = emulator->reg.isFCarry();

	emulator->reg.resetFHalfCarry();

	if (emulator->reg.isFSubtract()) {
		if (halfCarry) {
			endVal = (endVal - 0x06) & 0xFF;
		}

		if (carry) {
			endVal -= 0x60;
		}
	} else {
		if (halfCarry || ((endVal & 0x0F) > 9)) {
			endVal += 0x06;
		}

		if (carry || endVal > 0x9F) {
			endVal += 0x60;
		}
	}

	emulator->reg.a = static_cast<uint8_t>(endVal & 0xFF);
	emulator->reg.setFZero(emulator->reg.a == 0);

	if (endVal >= 0x100) {
		emulator->reg.setFCarry();
	}
}

void SCF::executeImpl() {
	emulator->reg.setFCarry();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFSubtract();
}

}
