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
#include <cstdint>

#include <iostream>
#include <iomanip>

#include "Emulator.hpp"
#include "Instructions.hpp"

namespace y3e {

PCHandlerType Instruction::doNothingHandler = [](Emulator * emulator, uint8_t instructionLength) {};
PCHandlerType Instruction::normalHandler =
        [](Emulator * emulator, uint8_t instructionLength) {emulator->reg.pc += instructionLength;};

int8_t Instruction::eiQueue = -1;
int8_t Instruction::diQueue = -1;

Instruction::Instruction(Emulator * emulator_, uint8_t opcode_, uint8_t instructionLength_, uint8_t cyclesTaken_,
        const char * humanReadable_, bool normalPCHandling_, bool normalCycleHandling_) :
		        emulator { emulator_ },
		        opcode { opcode_ },
		        instructionLength { instructionLength_ },
		        cyclesTaken { cyclesTaken_ },
		        humanReadable { humanReadable_ },
		        normalPCHandling { normalPCHandling_ },
		        pcHandler(normalPCHandling ? Instruction::normalHandler : Instruction::doNothingHandler),
		        normalCycleHandling { normalCycleHandling_ } {

	if (cyclesTaken % 4 != 0) {
		std::cout << "Warning: cycles taken is non-multiple of 4 in Instruction constructor for " << humanReadable
		        << std::endl;
	}

	if (instructionLength == 0 || instructionLength > 3) {
		std::cout << "Warning: invalid instruction length detected in Instruction constructor for " << humanReadable
		        << std::endl;
	}
}

void Instruction::execute() {
	executeImpl();
	pcHandler(emulator, instructionLength);

	if (eiQueue >= 0) {
		Instruction::eiQueue--;
	}

	if (diQueue >= 0) {
		Instruction::diQueue--;
	}

	if (Instruction::eiQueue == 0) {
		doEI();
	}

	if (Instruction::diQueue == 0) {
		doDI();
	}
}

void Instruction::doEI() {
	emulator->interruptHandler.enableAllInterrupts();
}

void Instruction::doDI() {
	emulator->interruptHandler.disableAllInterrupts();
}

void Instruction::queueEI() {
	eiQueue = 2;
}

void Instruction::queueDI() {
	diQueue = 2;
}

}
