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

#define MAKE_STANDARD_LD8_IMPL(clname, reg1, reg2) void clname::executeImpl() {\
	emulator->reg.reg1 = emulator->reg.reg2;\
}

#define MAKE_STANDARD_LD8_HLv_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg._reg = emulator->memory.read(emulator->reg.hl);\
}

#define MAKE_STANDARD_LDHLv_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->memory.write(emulator->reg.hl, emulator->reg._reg);\
}

namespace y3e {

void LDBCvA::executeImpl() {
	emulator->memory.write(emulator->reg.bc, emulator->reg.a);
}

void LDAd8::executeImpl() {
	emulator->reg.a = value;
}

void LDBd8::executeImpl() {
	emulator->reg.b = value;
}

void LDDd8::executeImpl() {
	emulator->reg.d = value;
}

void LDEd8::executeImpl() {
	emulator->reg.e = value;
}

void LDLd8::executeImpl() {
	emulator->reg.l = value;
}

void LDHd8::executeImpl() {
	emulator->reg.h = value;
}

void LDDEvA::executeImpl() {
	emulator->memory.write(emulator->reg.de, emulator->reg.a);
}

void LDDAHLv::executeImpl() {
	const auto val = emulator->memory.read(emulator->reg.hl);

	emulator->reg.a = val;

	emulator->reg.hl--;
}

void LDIHLvA::executeImpl() {
	emulator->memory.write(emulator->reg.hl, emulator->reg.a);
	emulator->reg.hl++;
}

void LDIAHL::executeImpl() {
	emulator->reg.a = emulator->memory.read(emulator->reg.hl);
	emulator->reg.hl++;
}

void LDHa8A::executeImpl() {
	emulator->memory.write(0xFF00 + value, emulator->reg.a);
}

void LDHLd8::executeImpl() {
	emulator->memory.write(emulator->reg.hl, value);
}

void LDHAa8::executeImpl() {
	emulator->reg.a = emulator->memory.read(0xFF00 + value);
}

void LDCVA::executeImpl() {
	emulator->memory.write(0xFF00 + emulator->reg.c, emulator->reg.a);
}

void LDDHLA::executeImpl() {
	emulator->memory.write(emulator->reg.hl, emulator->reg.a);
	emulator->reg.hl--;
}

void LDABCv::executeImpl() {
	emulator->reg.a = emulator->memory.read(emulator->reg.bc);
}

void LDADEv::executeImpl() {
	emulator->reg.a = emulator->memory.read(emulator->reg.de);
}

void LDCd8::executeImpl() {
	emulator->reg.c = value;
}

void LDAa16v::executeImpl() {
	emulator->reg.a = emulator->memory.read(value);
}

MAKE_STANDARD_LD8_IMPL(LDBB, b, b);
MAKE_STANDARD_LD8_IMPL(LDBC, b, c);
MAKE_STANDARD_LD8_IMPL(LDBD, b, d);
MAKE_STANDARD_LD8_IMPL(LDBE, b, e);
MAKE_STANDARD_LD8_IMPL(LDBH, b, h);
MAKE_STANDARD_LD8_IMPL(LDBL, b, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDBHLv, b);
MAKE_STANDARD_LD8_IMPL(LDBA, b, a);

MAKE_STANDARD_LD8_IMPL(LDCB, c, b);
MAKE_STANDARD_LD8_IMPL(LDCC, c, c);
MAKE_STANDARD_LD8_IMPL(LDCD, c, d);
MAKE_STANDARD_LD8_IMPL(LDCE, c, e);
MAKE_STANDARD_LD8_IMPL(LDCH, c, h);
MAKE_STANDARD_LD8_IMPL(LDCL, c, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDCHLv, c);
MAKE_STANDARD_LD8_IMPL(LDCA, c, a);

MAKE_STANDARD_LD8_IMPL(LDDB, d, b);
MAKE_STANDARD_LD8_IMPL(LDDC, d, c);
MAKE_STANDARD_LD8_IMPL(LDDD, d, d);
MAKE_STANDARD_LD8_IMPL(LDDE, d, e);
MAKE_STANDARD_LD8_IMPL(LDDH, d, h);
MAKE_STANDARD_LD8_IMPL(LDDL, d, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDDHLv, d);
MAKE_STANDARD_LD8_IMPL(LDDA, d, a);

MAKE_STANDARD_LD8_IMPL(LDEB, e, b);
MAKE_STANDARD_LD8_IMPL(LDEC, e, c);
MAKE_STANDARD_LD8_IMPL(LDED, e, d);
MAKE_STANDARD_LD8_IMPL(LDEE, e, e);
MAKE_STANDARD_LD8_IMPL(LDEH, e, h);
MAKE_STANDARD_LD8_IMPL(LDEL, e, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDEHLv, e);
MAKE_STANDARD_LD8_IMPL(LDEA, e, a);

MAKE_STANDARD_LD8_IMPL(LDHB, h, b);
MAKE_STANDARD_LD8_IMPL(LDHC, h, c);
MAKE_STANDARD_LD8_IMPL(LDHD, h, d);
MAKE_STANDARD_LD8_IMPL(LDHE, h, e);
MAKE_STANDARD_LD8_IMPL(LDHH, h, h);
MAKE_STANDARD_LD8_IMPL(LDHL, h, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDHHLv, h);
MAKE_STANDARD_LD8_IMPL(LDHA, h, a);

MAKE_STANDARD_LD8_IMPL(LDLB, l, b);
MAKE_STANDARD_LD8_IMPL(LDLC, l, c);
MAKE_STANDARD_LD8_IMPL(LDLD, l, d);
MAKE_STANDARD_LD8_IMPL(LDLE, l, e);
MAKE_STANDARD_LD8_IMPL(LDLH, l, h);
MAKE_STANDARD_LD8_IMPL(LDLL, l, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDLHLv, l);
MAKE_STANDARD_LD8_IMPL(LDLA, l, a);

MAKE_STANDARD_LDHLv_IMPL(LDHLvB, b);
MAKE_STANDARD_LDHLv_IMPL(LDHLvC, c);
MAKE_STANDARD_LDHLv_IMPL(LDHLvD, d);
MAKE_STANDARD_LDHLv_IMPL(LDHLvE, e);
MAKE_STANDARD_LDHLv_IMPL(LDHLvH, h);
MAKE_STANDARD_LDHLv_IMPL(LDHLvL, l);
MAKE_STANDARD_LDHLv_IMPL(LDHLvA, a);

MAKE_STANDARD_LD8_IMPL(LDAB, a, b);
MAKE_STANDARD_LD8_IMPL(LDAC, a, c);
MAKE_STANDARD_LD8_IMPL(LDAD, a, d);
MAKE_STANDARD_LD8_IMPL(LDAE, a, e);
MAKE_STANDARD_LD8_IMPL(LDAH, a, h);
MAKE_STANDARD_LD8_IMPL(LDAL, a, l);
MAKE_STANDARD_LD8_HLv_IMPL(LDAHLv, a);
MAKE_STANDARD_LD8_IMPL(LDAA, a, a);

void LDACv::executeImpl() {
	emulator->reg.a = emulator->memory.read(0xFF00 + emulator->reg.c);
}

}
