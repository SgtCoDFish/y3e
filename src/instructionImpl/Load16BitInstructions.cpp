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

void LDa16vSP::executeImpl() {
	emulator->memory.write16(value, emulator->reg.sp);
}

void LDBCd16::executeImpl() {
	emulator->reg.bc = value;
}

void LDHLd16::executeImpl() {
	emulator->reg.hl = value;
}

void LDSPd16::executeImpl() {
	emulator->reg.sp = value;
}

void LDDEd16::executeImpl() {
	emulator->reg.de = value;
}

void LDa16A::executeImpl() {
	emulator->memory.write(value, emulator->reg.a);
}

void POPAF::executeImpl() {
	emulator->reg.af = (emulator->stack.pop() & 0xFFF0);
}

void POPBC::executeImpl() {
	emulator->reg.bc = emulator->stack.pop();
}

void POPDE::executeImpl() {
	emulator->reg.de = emulator->stack.pop();
}

void POPHL::executeImpl() {
	emulator->reg.hl = emulator->stack.pop();
}

void PUSHBC::executeImpl() {
	emulator->stack.push(emulator->reg.bc);
}

void PUSHDE::executeImpl() {
	emulator->stack.push(emulator->reg.de);
}

void PUSHHL::executeImpl() {
	emulator->stack.push(emulator->reg.hl);
}

void PUSHAF::executeImpl() {
	emulator->stack.push(emulator->reg.af);
}

void LDSPHL::executeImpl() {
	emulator->reg.sp = emulator->reg.hl;
}

}
