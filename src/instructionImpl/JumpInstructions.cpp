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

#include "Emulator.hpp"
#include "Instructions.hpp"

namespace y3e {

void JRr8::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);
	emulator->reg.pc += value;
}

void JRNZr8::executeImpl() {
	// do normal pc increment anyway
	Instruction::normalHandler(emulator, instructionLength);

	if (!emulator->reg.isFZero()) {
		// do relative jump if not zero
		emulator->reg.pc += value;
	}
}

uint8_t JRNZr8::determineCycles() const {
	// takes 12 cycles to execute a relative jump if not zero
	return !emulator->reg.isFZero() ? 12 : 8;
}

void JRNCr8::executeImpl() {
	// do normal pc increment anyway
	Instruction::normalHandler(emulator, instructionLength);

	if (!emulator->reg.isFCarry()) {
		// do relative jump if not zero
		emulator->reg.pc += value;
	}
}

uint8_t JRNCr8::determineCycles() const {
	// takes 12 cycles to execute a relative jump if not zero
	return !emulator->reg.isFCarry() ? 12 : 8;
}

void JRZr8::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);

	if (emulator->reg.isFZero()) {
		emulator->reg.pc += value;
	}
}

uint8_t JRZr8::determineCycles() const {
	return emulator->reg.isFZero() ? 12 : 8;
}

void JRCr8::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);

	if (emulator->reg.isFCarry()) {
		emulator->reg.pc += value;
	}
}

uint8_t JRCr8::determineCycles() const {
	return emulator->reg.isFCarry() ? 12 : 8;
}

void JP16::executeImpl() {
	emulator->reg.pc = address;
}

void CALLNZa16::executeImpl() {
	if (!emulator->reg.isFZero()) {
		// Push next instruction's address onto stack
		emulator->stack.push(emulator->reg.pc + instructionLength);

		emulator->reg.pc = address;
	} else {
		Instruction::normalHandler(emulator, instructionLength);
	}
}

uint8_t CALLNZa16::determineCycles() const {
	return !emulator->reg.isFZero() ? 24 : 12;
}

void CALLZa16::executeImpl() {
	if (emulator->reg.isFZero()) {
		// Push next instruction's address onto stack
		emulator->stack.push(emulator->reg.pc + instructionLength);

		emulator->reg.pc = address;
	} else {
		Instruction::normalHandler(emulator, instructionLength);
	}
}

uint8_t CALLZa16::determineCycles() const {
	return emulator->reg.isFZero() ? 24 : 12;
}

void CALLCa16::executeImpl() {
	if (emulator->reg.isFCarry()) {
		// Push next instruction's address onto stack
		emulator->stack.push(emulator->reg.pc + instructionLength);

		emulator->reg.pc = address;
	} else {
		Instruction::normalHandler(emulator, instructionLength);
	}
}

uint8_t CALLCa16::determineCycles() const {
	return emulator->reg.isFCarry() ? 24 : 12;
}

void CALLNCa16::executeImpl() {
	if (!emulator->reg.isFCarry()) {
		// Push next instruction's address onto stack
		emulator->stack.push(emulator->reg.pc + instructionLength);

		emulator->reg.pc = address;
	} else {
		Instruction::normalHandler(emulator, instructionLength);
	}
}

uint8_t CALLNCa16::determineCycles() const {
	return !emulator->reg.isFCarry() ? 24 : 12;
}

void CALLa16::executeImpl() {
	// Push next instruction's address onto stack
	emulator->stack.push(emulator->reg.pc + instructionLength);

	emulator->reg.pc = address;
}

void JPHL::executeImpl() {
	emulator->reg.pc = emulator->reg.hl;
}

void RETNZ::executeImpl() {
	if (!emulator->reg.isFZero()) {
		const auto address = emulator->stack.pop();
		emulator->reg.pc = address;
	} else {
		emulator->reg.pc += instructionLength;
	}
}

uint8_t RETNZ::determineCycles() const {
	return !emulator->reg.isFZero() ? 20 : 8;
}

void RETNC::executeImpl() {
	if (!emulator->reg.isFCarry()) {
		const auto address = emulator->stack.pop();
		emulator->reg.pc = address;
	} else {
		emulator->reg.pc += instructionLength;
	}
}

uint8_t RETNC::determineCycles() const {
	return !emulator->reg.isFCarry() ? 20 : 8;
}

void RETZ::executeImpl() {
	if (emulator->reg.isFZero()) {
		const auto address = emulator->stack.pop();
		emulator->reg.pc = address;
	} else {
		emulator->reg.pc += instructionLength;
	}
}

uint8_t RETZ::determineCycles() const {
	return emulator->reg.isFZero() ? 20 : 8;
}

void RET::executeImpl() {
	const auto addr = emulator->stack.pop();

	emulator->reg.pc = addr;
}

void RETC::executeImpl() {
	if (emulator->reg.isFCarry()) {
		const auto address = emulator->stack.pop();
		emulator->reg.pc = address;
	} else {
		emulator->reg.pc += instructionLength;
	}
}

uint8_t RETC::determineCycles() const {
	return emulator->reg.isFCarry() ? 20 : 8;
}

void RETI::executeImpl() {
	const auto addr = emulator->stack.pop();

	emulator->reg.pc = addr;
	emulator->interruptHandler.enableAllInterrupts();
//	std::cout << "RETI\n";
}

void JPNZa16::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);

	if (!emulator->reg.isFZero()) {
		emulator->reg.pc = address;
	}
}

uint8_t JPNZa16::determineCycles() const {
	return !emulator->reg.isFZero() ? 16 : 12;
}

void JPNCa16::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);

	if (!emulator->reg.isFCarry()) {
		emulator->reg.pc = address;
	}
}

uint8_t JPNCa16::determineCycles() const {
	return !emulator->reg.isFCarry() ? 16 : 12;
}

void JPZa16::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);

	if (emulator->reg.isFZero()) {
		emulator->reg.pc = value;
	}
}

uint8_t JPZa16::determineCycles() const {
	return emulator->reg.isFZero() ? 16 : 12;
}

void JPCa16::executeImpl() {
	Instruction::normalHandler(emulator, instructionLength);

	if (emulator->reg.isFCarry()) {
		emulator->reg.pc = value;
	}
}

uint8_t JPCa16::determineCycles() const {
	return emulator->reg.isFCarry() ? 16 : 12;
}

void RST00::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0000;
}

void RST08::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0008;
}

void RST10::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0010;
}

void RST18::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0018;
}

void RST20::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0020;
}

void RST28::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0028;
}

void RST30::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0030;
}

void RST38::executeImpl() {
	emulator->stack.push(emulator->reg.pc + instructionLength);
	emulator->reg.pc = 0x0038;
}

}
