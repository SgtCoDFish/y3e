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

void DECC::executeImpl() {
	// because we only sub 1, we know a carry can only occur if reg.b == 0
	// but this is more complicated for other subtractions.
	emulator->reg.setFHalfCarry(!(emulator->reg.c & 0xf));

	emulator->reg.c--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.c == 0);
}

void DECL::executeImpl() {
	// because we only sub 1, we know a carry can only occur if reg.l == 0
	// but this is more complicated for other subtractions.
	emulator->reg.setFHalfCarry(!(emulator->reg.l & 0xf));

	emulator->reg.l--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.l == 0);
}

void DECE::executeImpl() {
	// because we only sub 1, we know a carry can only occur if reg.e == 0
	// but this is more complicated for other subtractions.
	emulator->reg.setFHalfCarry(!(emulator->reg.e & 0xf));

	emulator->reg.e--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.e == 0);
}

void DECB::executeImpl() {
	// because we only sub 1, we know a carry can only occur if reg.b == 0
	// but this is more complicated for other subtractions.
	emulator->reg.setFHalfCarry(!(emulator->reg.b & 0xf));

	emulator->reg.b--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.b == 0);
}

void DECA::executeImpl() {
	emulator->reg.setFHalfCarry(!(emulator->reg.a & 0xf));

	emulator->reg.a--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.a == 0);
}

void DECD::executeImpl() {
	emulator->reg.setFHalfCarry(!(emulator->reg.d & 0xf));

	emulator->reg.d--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.d == 0);
}

void DECH::executeImpl() {
	emulator->reg.setFHalfCarry(!(emulator->reg.h & 0xf));

	emulator->reg.h--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(emulator->reg.h == 0);
}

void DECHLv::executeImpl() {
	auto mem = emulator->memory.read(emulator->reg.hl);
	emulator->reg.setFHalfCarry(!(mem & 0xf));

	mem--;

	// always set f.sub
	emulator->reg.setFSubtract();
	emulator->reg.setFZero(mem == 0);

	emulator->memory.write(emulator->reg.hl, mem);
}

void INCB::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.b & 0xf) == 0xf);

	emulator->reg.b++;

	emulator->reg.setFZero(emulator->reg.b == 0x00);
	emulator->reg.resetFSubtract();
}

void INCC::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.c & 0xf) == 0xf);

	emulator->reg.c++;

	emulator->reg.setFZero(emulator->reg.c == 0x00);
	emulator->reg.resetFSubtract();
}

void INCE::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.e & 0xf) == 0xf);

	emulator->reg.e++;

	emulator->reg.setFZero(emulator->reg.e == 0x00);
	emulator->reg.resetFSubtract();
}

void INCL::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.l & 0xf) == 0xf);

	emulator->reg.l++;

	emulator->reg.setFZero(emulator->reg.l == 0x00);
	emulator->reg.resetFSubtract();
}

void INCH::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.h & 0xf) == 0xf);

	emulator->reg.h++;

	emulator->reg.setFZero(emulator->reg.h == 0x00);
	emulator->reg.resetFSubtract();
}

void INCD::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.d & 0xf) == 0xf);

	emulator->reg.d++;

	emulator->reg.setFZero(emulator->reg.d == 0x00);
	emulator->reg.resetFSubtract();
}

void INCHLv::executeImpl() {
	auto mem = emulator->memory.read(emulator->reg.hl);
	emulator->reg.setFHalfCarry((mem & 0x0f) == 0x0f);

	mem++;

	// always reset f.sub
	emulator->reg.setFZero(mem == 0);
	emulator->reg.resetFSubtract();

	emulator->memory.write(emulator->reg.hl, mem);
}

void CPL::executeImpl() {
	// Sets N, H
	emulator->reg.a = ~emulator->reg.a;

	emulator->reg.setFHalfCarry();
	emulator->reg.setFSubtract();
}

void ADDAd8::executeImpl() {
	emulator->reg.setFCarry(emulator->reg.a > 0xFF - value);
	emulator->reg.setFHalfCarry((((emulator->reg.a & 0xF) + (value & 0xF)) & 0x10) == 0x10);
	emulator->reg.resetFSubtract();
	emulator->reg.a += value;

	emulator->reg.setFZero(emulator->reg.a == 0);
}

void INCA::executeImpl() {
	emulator->reg.setFHalfCarry((emulator->reg.a & 0xf) == 0xf);

	emulator->reg.a++;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
	emulator->reg.resetFSubtract();
}

#define MAKE_STANDARD_ADD_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.setFCarry(emulator->reg.a > 0xFF - emulator->reg._reg);\
	emulator->reg.setFHalfCarry((((emulator->reg.a & 0xF) + (emulator->reg._reg & 0xF)) & 0x10) == 0x10);\
	emulator->reg.a += emulator->reg._reg;\
\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
	emulator->reg.resetFSubtract();\
}

MAKE_STANDARD_ADD_IMPL(ADDAB, b)
MAKE_STANDARD_ADD_IMPL(ADDAC, c)
MAKE_STANDARD_ADD_IMPL(ADDAD, d)
MAKE_STANDARD_ADD_IMPL(ADDAE, e)
MAKE_STANDARD_ADD_IMPL(ADDAH, h)
MAKE_STANDARD_ADD_IMPL(ADDAL, l)
MAKE_STANDARD_ADD_IMPL(ADDAA, a)

void ADDAHLv::executeImpl() {
	const auto val = emulator->memory.read(emulator->reg.hl);

	emulator->reg.setFCarry(emulator->reg.a > 0xFF - val);
	emulator->reg.setFHalfCarry((((emulator->reg.a & 0xF) + (val & 0xF)) & 0x10) == 0x10);
	emulator->reg.a += val;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
	emulator->reg.resetFSubtract();
}

#define MAKE_STANDARD_ADC_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
\
	const auto val = emulator->reg.a + emulator->reg._reg + emulator->reg.isFCarry();\
\
	emulator->reg.setFHalfCarry(((emulator->reg.a & 0xF) + (emulator->reg._reg & 0xF) + emulator->reg.isFCarry()) >= 0x10);\
	emulator->reg.setFCarry(val > 0xFF);\
	emulator->reg.a = val;\
\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
}

MAKE_STANDARD_ADC_IMPL(ADCAB, b);
MAKE_STANDARD_ADC_IMPL(ADCAC, c);
MAKE_STANDARD_ADC_IMPL(ADCAD, d);
MAKE_STANDARD_ADC_IMPL(ADCAE, e);
MAKE_STANDARD_ADC_IMPL(ADCAH, h);
MAKE_STANDARD_ADC_IMPL(ADCAL, l);
MAKE_STANDARD_ADC_IMPL(ADCAA, a);

void ADCAHLv::executeImpl() {
	emulator->reg.resetFSubtract();

	const auto mem = emulator->memory.read(emulator->reg.hl);
	const auto val = emulator->reg.a + mem + emulator->reg.isFCarry();

	emulator->reg.setFHalfCarry(((emulator->reg.a & 0xF) + (mem & 0xF) + emulator->reg.isFCarry()) >= 0x10);
	emulator->reg.setFCarry(val > 0xFF);
	emulator->reg.a = val;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

void ADCAd8::executeImpl() {
	emulator->reg.resetFSubtract();

	const auto val = emulator->reg.a + value + emulator->reg.isFCarry();

	emulator->reg.setFHalfCarry(((emulator->reg.a & 0xF) + (value & 0xF) + emulator->reg.isFCarry()) >= 0x10);
	emulator->reg.setFCarry(val > 0xFF);
	emulator->reg.a = val;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

void SBCAd8::executeImpl() {
	emulator->reg.setFSubtract();
	const int32_t subVal = value + emulator->reg.isFCarry();

	emulator->reg.setFHalfCarry(((emulator->reg.a & 0x0F) - (value & 0x0F) - emulator->reg.isFCarry()) < 0);
	emulator->reg.setFCarry(emulator->reg.a - subVal < 0);
	emulator->reg.a -= subVal;
	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

void SBCAHLv::executeImpl() {
	const auto val = emulator->memory.read(emulator->reg.hl);
	emulator->reg.setFSubtract();
	const int32_t subVal = val + emulator->reg.isFCarry();

	emulator->reg.setFHalfCarry(((emulator->reg.a & 0x0F) - (val & 0x0F) - emulator->reg.isFCarry()) < 0);
	emulator->reg.setFCarry(emulator->reg.a - subVal < 0);
	emulator->reg.a -= subVal;
	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

#define MAKE_STANDARD_SBC_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.setFSubtract();\
	const int32_t subVal = emulator->reg._reg + emulator->reg.isFCarry();\
\
	emulator->reg.setFHalfCarry(((emulator->reg.a & 0x0F) - (emulator->reg._reg & 0x0F) - emulator->reg.isFCarry()) < 0);\
	emulator->reg.setFCarry(emulator->reg.a - subVal < 0);\
	emulator->reg.a -= subVal;\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
}

MAKE_STANDARD_SBC_IMPL(SBCAB, b)
MAKE_STANDARD_SBC_IMPL(SBCAC, c)
MAKE_STANDARD_SBC_IMPL(SBCAD, d)
MAKE_STANDARD_SBC_IMPL(SBCAE, e)
MAKE_STANDARD_SBC_IMPL(SBCAH, h)
MAKE_STANDARD_SBC_IMPL(SBCAL, l)
MAKE_STANDARD_SBC_IMPL(SBCAA, a)

#define MAKE_STANDARD_AND_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.a = emulator->reg._reg & emulator->reg.a;\
\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFCarry();\
	emulator->reg.setFHalfCarry();\
}

MAKE_STANDARD_AND_IMPL(ANDB, b);
MAKE_STANDARD_AND_IMPL(ANDC, c);
MAKE_STANDARD_AND_IMPL(ANDD, d);
MAKE_STANDARD_AND_IMPL(ANDE, e);
MAKE_STANDARD_AND_IMPL(ANDH, h);
MAKE_STANDARD_AND_IMPL(ANDL, l);
MAKE_STANDARD_AND_IMPL(ANDA, a);

void ANDHLv::executeImpl() {
	emulator->reg.a = emulator->memory.read(emulator->reg.hl) & emulator->reg.a;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
	emulator->reg.resetFSubtract();
	emulator->reg.resetFCarry();
	emulator->reg.setFHalfCarry();
}

#define MAKE_STANDARD_OR_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
	emulator->reg.resetFCarry();\
\
	emulator->reg.a = emulator->reg.a | emulator->reg._reg;\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
}

MAKE_STANDARD_OR_IMPL(ORB, b);
MAKE_STANDARD_OR_IMPL(ORC, c);
MAKE_STANDARD_OR_IMPL(ORD, d);
MAKE_STANDARD_OR_IMPL(ORE, e);
MAKE_STANDARD_OR_IMPL(ORH, h);
MAKE_STANDARD_OR_IMPL(ORL, l);
MAKE_STANDARD_OR_IMPL(ORA, a);

void ORHLv::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFCarry();

	emulator->reg.a = emulator->reg.a | emulator->memory.read(emulator->reg.hl);

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

#define MAKE_STANDARD_XOR_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.a = emulator->reg._reg ^ emulator->reg.a;\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFCarry();\
	emulator->reg.resetFHalfCarry();\
}

MAKE_STANDARD_XOR_IMPL(XORB, b);
MAKE_STANDARD_XOR_IMPL(XORC, c);
MAKE_STANDARD_XOR_IMPL(XORD, d);
MAKE_STANDARD_XOR_IMPL(XORE, e);
MAKE_STANDARD_XOR_IMPL(XORH, h);
MAKE_STANDARD_XOR_IMPL(XORL, l);
MAKE_STANDARD_XOR_IMPL(XORA, a);

void XORHLv::executeImpl() {
	emulator->reg.a ^= emulator->memory.read(emulator->reg.hl);
	emulator->reg.setFZero(emulator->reg.a == 0x00);
	emulator->reg.resetFSubtract();
	emulator->reg.resetFCarry();
	emulator->reg.resetFHalfCarry();
}

void ANDd8::executeImpl() {
	// Sets Z if appropriate, sets H
	emulator->reg.setFHalfCarry();
	emulator->reg.resetFSubtract();
	emulator->reg.setFZero((emulator->reg.a = emulator->reg.a & value) == 0);
	emulator->reg.resetFCarry();
}

#define MAKE_STANDARD_CP_IMPL(clname, reg_) void clname::executeImpl() {\
	const auto &a = emulator->reg.a;\
\
	emulator->reg.setFCarry(emulator->reg.reg_ > a);\
	emulator->reg.setFHalfCarry((emulator->reg.reg_ & 0xF) > (a & 0xF));\
	emulator->reg.setFZero(a == emulator->reg.reg_);\
	emulator->reg.setFSubtract();\
}

MAKE_STANDARD_CP_IMPL(CPB, b);
MAKE_STANDARD_CP_IMPL(CPC, c);
MAKE_STANDARD_CP_IMPL(CPD, d);
MAKE_STANDARD_CP_IMPL(CPE, e);
MAKE_STANDARD_CP_IMPL(CPH, h);
MAKE_STANDARD_CP_IMPL(CPwithL, l);
MAKE_STANDARD_CP_IMPL(CPA, a);

void CPHLv::executeImpl() {
	const auto &a = emulator->reg.a;
	const auto value = emulator->memory.read(emulator->reg.hl);

	emulator->reg.setFCarry(value > a);
	emulator->reg.setFHalfCarry((value & 0xF) > (a & 0xF));
	emulator->reg.setFZero(a == value);
	emulator->reg.setFSubtract();
}

#define MAKE_STANDARD_SUB_IMPL(clname, _reg) void clname::executeImpl() {\
	const auto value = emulator->reg._reg;\
\
	emulator->reg.setFSubtract();\
	emulator->reg.setFCarry(value > emulator->reg.a);\
	emulator->reg.setFHalfCarry((value & 0xF) > (emulator->reg.a & 0xF));\
\
	emulator->reg.a -= value;\
\
	emulator->reg.setFZero(emulator->reg.a == 0x00);\
}

MAKE_STANDARD_SUB_IMPL(SUBB, b);
MAKE_STANDARD_SUB_IMPL(SUBC, c);
MAKE_STANDARD_SUB_IMPL(SUBD, d);
MAKE_STANDARD_SUB_IMPL(SUBE, e);
MAKE_STANDARD_SUB_IMPL(SUBH, h);
MAKE_STANDARD_SUB_IMPL(SUBL, l);
MAKE_STANDARD_SUB_IMPL(SUBA, a);

void SUBHLv::executeImpl() {
	const auto value = emulator->memory.read(emulator->reg.hl);

	emulator->reg.setFSubtract();
	emulator->reg.setFCarry(value > emulator->reg.a);
	emulator->reg.setFHalfCarry((value & 0xF) > (emulator->reg.a & 0xF));

	emulator->reg.a -= value;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

void SUBd8::executeImpl() {
	emulator->reg.setFSubtract();
	emulator->reg.setFCarry(value > emulator->reg.a);
	emulator->reg.setFHalfCarry((value & 0xF) > (emulator->reg.a & 0xF));

	emulator->reg.a -= value;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

void CPd8::executeImpl() {
	const auto &a = emulator->reg.a;

	emulator->reg.setFCarry(value > a);
	emulator->reg.setFHalfCarry((value & 0xF) > (a & 0xF));
	emulator->reg.setFZero(a == value);
	emulator->reg.setFSubtract();
}

void ORd8::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFCarry();

	emulator->reg.a |= value;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

void XORd8::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFCarry();

	emulator->reg.a ^= value;

	emulator->reg.setFZero(emulator->reg.a == 0x00);
}

}
