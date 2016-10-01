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

void ADDHLBC::executeImpl() {
	const uint32_t tval = emulator->reg.hl + emulator->reg.bc;

	emulator->reg.setFCarry(tval > 0xFFFF);
	emulator->reg.setFHalfCarry((tval & 0x7FF) < (emulator->reg.hl & 0x7FF));
	emulator->reg.resetFSubtract();

	emulator->reg.hl = static_cast<uint16_t>(tval);
}

void ADDHLDE::executeImpl() {
	const uint32_t tval = emulator->reg.hl + emulator->reg.de;

	emulator->reg.setFCarry(tval > 0xFFFF);
	emulator->reg.setFHalfCarry((tval & 0x7FF) < (emulator->reg.hl & 0x7FF));
	emulator->reg.resetFSubtract();

	emulator->reg.hl = static_cast<uint16_t>(tval);
}

void ADDHLHL::executeImpl() {
	const uint32_t tval = emulator->reg.hl + emulator->reg.hl;

	emulator->reg.setFCarry(tval > 0xFFFF);
	emulator->reg.setFHalfCarry((tval & 0x7FF) < (emulator->reg.hl & 0x7FF));
	emulator->reg.resetFSubtract();

	emulator->reg.hl = static_cast<uint16_t>(tval);
}

void ADDHLSP::executeImpl() {
	const uint32_t tval = emulator->reg.hl + emulator->reg.sp;

	emulator->reg.setFCarry(tval > 0xFFFF);
	emulator->reg.setFHalfCarry((tval & 0x7FF) < (emulator->reg.hl & 0x7FF));
	emulator->reg.resetFSubtract();

	emulator->reg.hl = static_cast<uint16_t>(tval);
}

#define MAKE_STANDARD_INC16(clname, _reg) void clname::executeImpl() {\
		emulator->reg._reg++;\
}

MAKE_STANDARD_INC16(INCBC, bc);
MAKE_STANDARD_INC16(INCDE, de);
MAKE_STANDARD_INC16(INCHL, hl);
MAKE_STANDARD_INC16(INCSP, sp);

#define MAKE_STANDARD_DEC16(clname, _reg) void clname::executeImpl() {\
		emulator->reg._reg--;\
}

MAKE_STANDARD_DEC16(DECBC, bc);
MAKE_STANDARD_DEC16(DECDE, de);
MAKE_STANDARD_DEC16(DECHL, hl);
MAKE_STANDARD_DEC16(DECSP, sp);

void LDHLSPr8::executeImpl() {
	emulator->reg.resetFZero();
	emulator->reg.resetFSubtract();

	const uint32_t tval = emulator->reg.sp + value;

	emulator->reg.setFCarry((tval & 0xFF) < (emulator->reg.sp & 0xFF));
	emulator->reg.setFHalfCarry((tval & 0xF) < (emulator->reg.sp & 0xF));

	emulator->reg.hl = tval;

}

void ADDSPr8::executeImpl() {
	emulator->reg.resetFZero();
	emulator->reg.resetFSubtract();

	const uint32_t tval = emulator->reg.sp + value;

	emulator->reg.setFCarry((tval & 0xFF) < (emulator->reg.sp & 0xFF));
	emulator->reg.setFHalfCarry((tval & 0xF) < (emulator->reg.sp & 0xF));

	emulator->reg.sp = tval;
}

}
