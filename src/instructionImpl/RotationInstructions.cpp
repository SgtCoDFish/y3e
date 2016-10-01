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

#define MAKE_RLC_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
\
	const auto oldBit7 = emulator->reg._reg & 0b10000000;\
\
	emulator->reg.setFCarry(oldBit7);\
	emulator->reg._reg <<= 1;\
	emulator->reg._reg += !!(oldBit7);\
\
	emulator->reg.setFZero(emulator->reg._reg == 0);\
}

MAKE_RLC_IMPL(RLCB, b)
MAKE_RLC_IMPL(RLCC, c)
MAKE_RLC_IMPL(RLCD, d)
MAKE_RLC_IMPL(RLCE, e)
MAKE_RLC_IMPL(RLCH, h)
MAKE_RLC_IMPL(RLCL, l)
MAKE_RLC_IMPL(RLCACB, a)

// handles Z flag differently to the CB version
void RLCA::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFZero();

	const auto oldBit7 = emulator->reg.a & 0b10000000;

	emulator->reg.setFCarry(oldBit7);
	emulator->reg.a <<= 1;
	emulator->reg.a += !!(oldBit7);
}

void RLCHLv::executeImpl() {
	auto val = emulator->memory.read(emulator->reg.hl);

	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();

	const auto oldBit7 = val & 0b10000000;

	emulator->reg.setFCarry(oldBit7);
	val <<= 1;
	val += !!(oldBit7);

	emulator->reg.setFZero(val == 0);
	emulator->memory.write(emulator->reg.hl, val);
}

#define MAKE_RL_IMPL(clname, _reg) void clname::executeImpl() {\
	const bool carry = emulator->reg.isFCarry();\
	emulator->reg.setFCarry(emulator->reg._reg & 0x80);\
\
	emulator->reg._reg <<= 1;\
	emulator->reg._reg += carry;\
\
	emulator->reg.setFZero(emulator->reg._reg == 0);\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
}

MAKE_RL_IMPL(RLB, b)
MAKE_RL_IMPL(RLC, c)
MAKE_RL_IMPL(RLD, d)
MAKE_RL_IMPL(RLE, e)
MAKE_RL_IMPL(RLH, h)
MAKE_RL_IMPL(RLL, l)
MAKE_RL_IMPL(RLACB, a)

// Resets Z, as opposed to the CB version
void RLA::executeImpl() {
	const bool carry = emulator->reg.isFCarry();
	emulator->reg.setFCarry(emulator->reg.a & 0x80);

	emulator->reg.a <<= 1;
	emulator->reg.a += carry;

	emulator->reg.resetFZero();
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
}

void RLHLv::executeImpl() {
	auto val = emulator->memory.read(emulator->reg.hl);

	const bool carry = emulator->reg.isFCarry();
	emulator->reg.setFCarry(val & 0x80);

	val <<= 1;
	val += carry;

	emulator->reg.setFZero(val == 0);
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();

	emulator->memory.write(emulator->reg.hl, val);
}

#define MAKE_RR_IMPL(clname, _reg) void clname::executeImpl() {\
	const auto val = emulator->reg._reg;\
\
	const uint8_t newBit7 = (static_cast<uint8_t>(emulator->reg.isFCarry()) << 7);\
\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
	emulator->reg.setFCarry(val & 0x1);\
\
	const auto newVal = (val >> 1) + (newBit7);\
	emulator->reg.setFZero(newVal == 0);\
\
	emulator->reg._reg = newVal;\
}

MAKE_RR_IMPL(RRB, b)
MAKE_RR_IMPL(RRC, c)
MAKE_RR_IMPL(RRD, d)
MAKE_RR_IMPL(RRE, e)
MAKE_RR_IMPL(RRH, h)
MAKE_RR_IMPL(RRL, l)
MAKE_RR_IMPL(RRACB, a)

// Always resets Z, contrasting CB
void RRA::executeImpl() {
	const auto val = emulator->reg.a;
	const uint8_t newBit7 = (static_cast<uint8_t>(emulator->reg.isFCarry()) << 7);

	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFZero();
	emulator->reg.setFCarry(val & 0x1);

	emulator->reg.a = (val >> 1) + (newBit7);
}

void RRHLv::executeImpl() {
	auto val = emulator->memory.read(emulator->reg.hl);

	const uint8_t newBit7 = (static_cast<uint8_t>(emulator->reg.isFCarry()) << 7);

	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.setFCarry(val & 0x1);

	val = (val >> 1) + (newBit7);

	emulator->reg.setFZero(val == 0);\
	emulator->memory.write(emulator->reg.hl, val);
}

#define MAKE_RRC_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
\
	const bool bit0 = emulator->reg._reg & 0x1;\
\
	emulator->reg.setFCarry(bit0);\
	emulator->reg._reg >>= 1;\
	emulator->reg._reg += (bit0 << 7);\
\
	emulator->reg.setFZero(emulator->reg._reg == 0);\
}

MAKE_RRC_IMPL(RRCB, b)
MAKE_RRC_IMPL(RRCC, c)
MAKE_RRC_IMPL(RRCD, d)
MAKE_RRC_IMPL(RRCE, e)
MAKE_RRC_IMPL(RRCH, h)
MAKE_RRC_IMPL(RRCL, l)
MAKE_RRC_IMPL(RRCACB, a)

// Always resets Z, contrasting the CB impl
void RRCA::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFZero();

	const bool bit0 = emulator->reg.a & 0x1;

	emulator->reg.setFCarry(bit0);
	emulator->reg.a >>= 1;
	emulator->reg.a += (bit0 << 7);
}

void RRCHLv::executeImpl() {
	uint8_t val = emulator->memory.read(emulator->reg.hl);

	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();

	const bool bit0 = val & 0x1;

	emulator->reg.setFCarry(bit0);
	val >>= 1;
	val += (bit0 << 7);

	emulator->reg.setFZero(val == 0);

	emulator->memory.write(emulator->reg.hl, val);
}

#define MAKE_SLA_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
\
	emulator->reg.setFCarry(emulator->reg._reg & 0x80);\
	emulator->reg._reg <<= 1;\
	emulator->reg.setFZero(emulator->reg._reg == 0);\
}

MAKE_SLA_IMPL(SLAB, b)
MAKE_SLA_IMPL(SLAC, c)
MAKE_SLA_IMPL(SLAD, d)
MAKE_SLA_IMPL(SLAE, e)
MAKE_SLA_IMPL(SLAH, h)
MAKE_SLA_IMPL(SLAL, l)
MAKE_SLA_IMPL(SLAA, a)

#define MAKE_SRA_IMPL(clname, _reg) void clname::executeImpl() {\
	const auto value = emulator->reg._reg;\
\
	emulator->reg.resetFHalfCarry();\
	emulator->reg.resetFSubtract();\
\
	emulator->reg.setFCarry(value & 0x01);\
\
	emulator->reg._reg = (value & 0x80) | (value >> 1);\
	emulator->reg.setFZero(emulator->reg._reg == 0x00);\
}

MAKE_SRA_IMPL(SRAB, b)
MAKE_SRA_IMPL(SRAC, c)
MAKE_SRA_IMPL(SRAD, d)
MAKE_SRA_IMPL(SRAE, e)
MAKE_SRA_IMPL(SRAH, h)
MAKE_SRA_IMPL(SRAL, l)
MAKE_SRA_IMPL(SRAA, a)

void SRAHLv::executeImpl() {
	auto val = emulator->memory.read(emulator->reg.hl);

	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFSubtract();

	emulator->reg.setFCarry(val & 0x01);

	val = (val & 0x80) | (val >> 1);
	emulator->reg.setFZero(val == 0x00);

	emulator->memory.write(emulator->reg.hl, val);
}

#define MAKE_SRL_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
\
	emulator->reg.setFCarry(emulator->reg._reg & 0x01);\
\
	emulator->reg._reg >>= 1;\
	emulator->reg.setFZero(emulator->reg._reg == 0x00);\
}

MAKE_SRL_IMPL(SRLB, b);
MAKE_SRL_IMPL(SRLC, c);
MAKE_SRL_IMPL(SRLD, d);
MAKE_SRL_IMPL(SRLE, e);
MAKE_SRL_IMPL(SRLH, h);
MAKE_SRL_IMPL(SRLL, l);
MAKE_SRL_IMPL(SRLA, a);

void SRLHLv::executeImpl() {
	auto value = emulator->memory.read(emulator->reg.hl);

	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.setFCarry(value & 1);

	value >>= 1;
	emulator->reg.setFZero(value == 0);

	emulator->memory.write(emulator->reg.hl, value);
}

void SLAHLv::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();

	uint8_t value = emulator->memory.read(emulator->reg.hl);

	const bool lowBit = value & 0b10000000;
	emulator->reg.setFCarry(lowBit);

	value <<= 1;

	emulator->reg.setFZero(value == 0);
	emulator->memory.write(emulator->reg.hl, value);
}

#define MAKE_SWAP_IMPL(clname, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.resetFHalfCarry();\
	emulator->reg.resetFCarry();\
\
	emulator->reg.setFZero(emulator->reg._reg == 0x00);\
\
	const auto nibble1 = (emulator->reg._reg & 0b00001111);\
	const auto nibble2 = (emulator->reg._reg & 0b11110000) >> 4;\
\
	emulator->reg._reg = (nibble1 << 4) + nibble2;\
}

MAKE_SWAP_IMPL(SWAPA, a);
MAKE_SWAP_IMPL(SWAPB, b);
MAKE_SWAP_IMPL(SWAPC, c);
MAKE_SWAP_IMPL(SWAPD, d);
MAKE_SWAP_IMPL(SWAPE, e);
MAKE_SWAP_IMPL(SWAPH, h);
MAKE_SWAP_IMPL(SWAPL, l);

void SWAPHLv::executeImpl() {
	emulator->reg.resetFSubtract();
	emulator->reg.resetFHalfCarry();
	emulator->reg.resetFCarry();

	const auto val = emulator->memory.read(emulator->reg.hl);

	emulator->reg.setFZero(val == 0x00);

	const auto nibble1 = (val & 0b00001111);
	const auto nibble2 = (val & 0b11110000) >> 4;

	emulator->memory.write(emulator->reg.hl, (nibble1 << 4) + nibble2);
}

#define MAKE_BIT_IMPL(__ignore1, clname, __ignore2, bit, _reg) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.setFHalfCarry();\
\
	emulator->reg.setFZero((emulator->reg._reg & (1 << bit)) == 0);\
}

#define MAKE_BIT_IMPL_HLV(__ignore1, clname, __ignore2, bit) void clname::executeImpl() {\
	emulator->reg.resetFSubtract();\
	emulator->reg.setFHalfCarry();\
\
	emulator->reg.setFZero((emulator->memory.read(emulator->reg.hl) & (1 << bit)) == 0);\
}

MAKE_BIT_IMPL(0x40, BIT0B, 8, 0, b);
MAKE_BIT_IMPL(0x41, BIT0C, 8, 0, c);
MAKE_BIT_IMPL(0x42, BIT0D, 8, 0, d);
MAKE_BIT_IMPL(0x43, BIT0E, 8, 0, e);
MAKE_BIT_IMPL(0x44, BIT0H, 8, 0, h);
MAKE_BIT_IMPL(0x45, BIT0L, 8, 0, l);
MAKE_BIT_IMPL_HLV(0x46, BIT0HLv, 16, 0);
MAKE_BIT_IMPL(0x47, BIT0A, 8, 0, a);

MAKE_BIT_IMPL(0x40, BIT1B, 8, 1, b);
MAKE_BIT_IMPL(0x41, BIT1C, 8, 1, c);
MAKE_BIT_IMPL(0x42, BIT1D, 8, 1, d);
MAKE_BIT_IMPL(0x43, BIT1E, 8, 1, e);
MAKE_BIT_IMPL(0x44, BIT1H, 8, 1, h);
MAKE_BIT_IMPL(0x45, BIT1L, 8, 1, l);
MAKE_BIT_IMPL_HLV(0x46, BIT1HLv, 16, 1);
MAKE_BIT_IMPL(0x47, BIT1A, 8, 1, a);

MAKE_BIT_IMPL(0x40, BIT2B, 8, 2, b);
MAKE_BIT_IMPL(0x41, BIT2C, 8, 2, c);
MAKE_BIT_IMPL(0x42, BIT2D, 8, 2, d);
MAKE_BIT_IMPL(0x43, BIT2E, 8, 2, e);
MAKE_BIT_IMPL(0x44, BIT2H, 8, 2, h);
MAKE_BIT_IMPL(0x45, BIT2L, 8, 2, l);
MAKE_BIT_IMPL_HLV(0x46, BIT2HLv, 16, 2);
MAKE_BIT_IMPL(0x47, BIT2A, 8, 2, a);

MAKE_BIT_IMPL(0x40, BIT3B, 8, 3, b);
MAKE_BIT_IMPL(0x41, BIT3C, 8, 3, c);
MAKE_BIT_IMPL(0x42, BIT3D, 8, 3, d);
MAKE_BIT_IMPL(0x43, BIT3E, 8, 3, e);
MAKE_BIT_IMPL(0x44, BIT3H, 8, 3, h);
MAKE_BIT_IMPL(0x45, BIT3L, 8, 3, l);
MAKE_BIT_IMPL_HLV(0x46, BIT3HLv, 16, 3);
MAKE_BIT_IMPL(0x47, BIT3A, 8, 3, a);

MAKE_BIT_IMPL(0x40, BIT4B, 8, 4, b);
MAKE_BIT_IMPL(0x41, BIT4C, 8, 4, c);
MAKE_BIT_IMPL(0x42, BIT4D, 8, 4, d);
MAKE_BIT_IMPL(0x43, BIT4E, 8, 4, e);
MAKE_BIT_IMPL(0x44, BIT4H, 8, 4, h);
MAKE_BIT_IMPL(0x45, BIT4L, 8, 4, l);
MAKE_BIT_IMPL_HLV(0x46, BIT4HLv, 16, 4);
MAKE_BIT_IMPL(0x47, BIT4A, 8, 4, a);

MAKE_BIT_IMPL(0x40, BIT5B, 8, 5, b);
MAKE_BIT_IMPL(0x41, BIT5C, 8, 5, c);
MAKE_BIT_IMPL(0x42, BIT5D, 8, 5, d);
MAKE_BIT_IMPL(0x43, BIT5E, 8, 5, e);
MAKE_BIT_IMPL(0x44, BIT5H, 8, 5, h);
MAKE_BIT_IMPL(0x45, BIT5L, 8, 5, l);
MAKE_BIT_IMPL_HLV(0x46, BIT5HLv, 16, 5);
MAKE_BIT_IMPL(0x47, BIT5A, 8, 5, a);

MAKE_BIT_IMPL(0x40, BIT6B, 8, 6, b);
MAKE_BIT_IMPL(0x41, BIT6C, 8, 6, c);
MAKE_BIT_IMPL(0x42, BIT6D, 8, 6, d);
MAKE_BIT_IMPL(0x43, BIT6E, 8, 6, e);
MAKE_BIT_IMPL(0x44, BIT6H, 8, 6, h);
MAKE_BIT_IMPL(0x45, BIT6L, 8, 6, l);
MAKE_BIT_IMPL_HLV(0x46, BIT6HLv, 16, 6);
MAKE_BIT_IMPL(0x47, BIT6A, 8, 6, a);

MAKE_BIT_IMPL(0x40, BIT7B, 8, 7, b);
MAKE_BIT_IMPL(0x41, BIT7C, 8, 7, c);
MAKE_BIT_IMPL(0x42, BIT7D, 8, 7, d);
MAKE_BIT_IMPL(0x43, BIT7E, 8, 7, e);
MAKE_BIT_IMPL(0x44, BIT7H, 8, 7, h);
MAKE_BIT_IMPL(0x45, BIT7L, 8, 7, l);
MAKE_BIT_IMPL_HLV(0x46, BIT7HLv, 16, 7);
MAKE_BIT_IMPL(0x47, BIT7A, 8, 7, a);

// the ignores are used so we can copy paste almost directly from Instructions, saving a lot of typing
#define MAKE_RES_IMPL(__ignoreThis, clname, bit, _reg) void clname##bit::executeImpl() {\
	emulator->reg._reg &= (0xFF ^ (1 << bit));\
}

#define MAKE_RES_IMPL_HL(__ignoreThis, clname, bit) void clname##bit::executeImpl() {\
	auto value = emulator->memory.read(emulator->reg.hl);\
	value &= (0xFF ^ (1 << bit));\
	emulator->memory.write(emulator->reg.hl, value);\
}

MAKE_RES_IMPL(0x80, RESB, 0, b);
MAKE_RES_IMPL(0x81, RESC, 0, c);
MAKE_RES_IMPL(0x82, RESD, 0, d);
MAKE_RES_IMPL(0x83, RESE, 0, e);
MAKE_RES_IMPL(0x84, RESH, 0, h);
MAKE_RES_IMPL(0x85, RESL, 0, l);
MAKE_RES_IMPL_HL(0x86, RESHLv, 0);
MAKE_RES_IMPL(0x87, RESA, 0, a);

MAKE_RES_IMPL(0x88, RESB, 1, b);
MAKE_RES_IMPL(0x89, RESC, 1, c);
MAKE_RES_IMPL(0x8A, RESD, 1, d);
MAKE_RES_IMPL(0x8B, RESE, 1, e);
MAKE_RES_IMPL(0x8C, RESH, 1, h);
MAKE_RES_IMPL(0x8D, RESL, 1, l);
MAKE_RES_IMPL_HL(0x8E, RESHLv, 1);
MAKE_RES_IMPL(0x8F, RESA, 1, a);

MAKE_RES_IMPL(0x90, RESB, 2, b);
MAKE_RES_IMPL(0x91, RESC, 2, c);
MAKE_RES_IMPL(0x92, RESD, 2, d);
MAKE_RES_IMPL(0x93, RESE, 2, e);
MAKE_RES_IMPL(0x94, RESH, 2, h);
MAKE_RES_IMPL(0x95, RESL, 2, l);
MAKE_RES_IMPL_HL(0x96, RESHLv, 2);
MAKE_RES_IMPL(0x97, RESA, 2, a);

MAKE_RES_IMPL(0x98, RESB, 3, b);
MAKE_RES_IMPL(0x99, RESC, 3, c);
MAKE_RES_IMPL(0x9A, RESD, 3, d);
MAKE_RES_IMPL(0x9B, RESE, 3, e);
MAKE_RES_IMPL(0x9C, RESH, 3, h);
MAKE_RES_IMPL(0x9D, RESL, 3, l);
MAKE_RES_IMPL_HL(0x9E, RESHLv, 3);
MAKE_RES_IMPL(0x9F, RESA, 3, a);

MAKE_RES_IMPL(0xA0, RESB, 4, b);
MAKE_RES_IMPL(0xA1, RESC, 4, c);
MAKE_RES_IMPL(0xA2, RESD, 4, d);
MAKE_RES_IMPL(0xA3, RESE, 4, e);
MAKE_RES_IMPL(0xA4, RESH, 4, h);
MAKE_RES_IMPL(0xA5, RESL, 4, l);
MAKE_RES_IMPL_HL(0xA6, RESHLv, 4);
MAKE_RES_IMPL(0xA7, RESA, 4, a);

MAKE_RES_IMPL(0xA8, RESB, 5, b);
MAKE_RES_IMPL(0xA9, RESC, 5, c);
MAKE_RES_IMPL(0xAA, RESD, 5, d);
MAKE_RES_IMPL(0xAB, RESE, 5, e);
MAKE_RES_IMPL(0xAC, RESH, 5, h);
MAKE_RES_IMPL(0xAD, RESL, 5, l);
MAKE_RES_IMPL_HL(0xAE, RESHLv, 5);
MAKE_RES_IMPL(0xAF, RESA, 5, a);

MAKE_RES_IMPL(0xB0, RESB, 6, b);
MAKE_RES_IMPL(0xB1, RESC, 6, c);
MAKE_RES_IMPL(0xB2, RESD, 6, d);
MAKE_RES_IMPL(0xB3, RESE, 6, e);
MAKE_RES_IMPL(0xB4, RESH, 6, h);
MAKE_RES_IMPL(0xB5, RESL, 6, l);
MAKE_RES_IMPL_HL(0xB6, RESHLv, 6);
MAKE_RES_IMPL(0xB7, RESA, 6, a);

MAKE_RES_IMPL(0xB8, RESB, 7, b);
MAKE_RES_IMPL(0xB9, RESC, 7, c);
MAKE_RES_IMPL(0xBA, RESD, 7, d);
MAKE_RES_IMPL(0xBB, RESE, 7, e);
MAKE_RES_IMPL(0xBC, RESH, 7, h);
MAKE_RES_IMPL(0xBD, RESL, 7, l);
MAKE_RES_IMPL_HL(0xBE, RESHLv, 7);
MAKE_RES_IMPL(0xBF, RESA, 7, a);

// the ignores are used so we can copy paste almost directly from Instructions, saving a lot of typing
#define MAKE_SET_IMPL(__ignoreThis, clname, bit, _reg) void clname##bit::executeImpl() {\
	emulator->reg._reg |= (1 << bit);\
}

#define MAKE_SET_IMPL_HL(__ignoreThis, clname, bit) void clname##bit::executeImpl() {\
	auto value = emulator->memory.read(emulator->reg.hl);\
	value |= (1 << bit);\
	emulator->memory.write(emulator->reg.hl, value);\
}

MAKE_SET_IMPL(0x80, SETB, 0, b);
MAKE_SET_IMPL(0x81, SETC, 0, c);
MAKE_SET_IMPL(0x82, SETD, 0, d);
MAKE_SET_IMPL(0x83, SETE, 0, e);
MAKE_SET_IMPL(0x84, SETH, 0, h);
MAKE_SET_IMPL(0x85, SETL, 0, l);
MAKE_SET_IMPL_HL(0x86, SETHLv, 0);
MAKE_SET_IMPL(0x87, SETA, 0, a);

MAKE_SET_IMPL(0x88, SETB, 1, b);
MAKE_SET_IMPL(0x89, SETC, 1, c);
MAKE_SET_IMPL(0x8A, SETD, 1, d);
MAKE_SET_IMPL(0x8B, SETE, 1, e);
MAKE_SET_IMPL(0x8C, SETH, 1, h);
MAKE_SET_IMPL(0x8D, SETL, 1, l);
MAKE_SET_IMPL_HL(0x8E, SETHLv, 1);
MAKE_SET_IMPL(0x8F, SETA, 1, a);

MAKE_SET_IMPL(0x90, SETB, 2, b);
MAKE_SET_IMPL(0x91, SETC, 2, c);
MAKE_SET_IMPL(0x92, SETD, 2, d);
MAKE_SET_IMPL(0x93, SETE, 2, e);
MAKE_SET_IMPL(0x94, SETH, 2, h);
MAKE_SET_IMPL(0x95, SETL, 2, l);
MAKE_SET_IMPL_HL(0x96, SETHLv, 2);
MAKE_SET_IMPL(0x97, SETA, 2, a);

MAKE_SET_IMPL(0x98, SETB, 3, b);
MAKE_SET_IMPL(0x99, SETC, 3, c);
MAKE_SET_IMPL(0x9A, SETD, 3, d);
MAKE_SET_IMPL(0x9B, SETE, 3, e);
MAKE_SET_IMPL(0x9C, SETH, 3, h);
MAKE_SET_IMPL(0x9D, SETL, 3, l);
MAKE_SET_IMPL_HL(0x9E, SETHLv, 3);
MAKE_SET_IMPL(0x9F, SETA, 3, a);

MAKE_SET_IMPL(0xA0, SETB, 4, b);
MAKE_SET_IMPL(0xA1, SETC, 4, c);
MAKE_SET_IMPL(0xA2, SETD, 4, d);
MAKE_SET_IMPL(0xA3, SETE, 4, e);
MAKE_SET_IMPL(0xA4, SETH, 4, h);
MAKE_SET_IMPL(0xA5, SETL, 4, l);
MAKE_SET_IMPL_HL(0xA6, SETHLv, 4);
MAKE_SET_IMPL(0xA7, SETA, 4, a);

MAKE_SET_IMPL(0xA8, SETB, 5, b);
MAKE_SET_IMPL(0xA9, SETC, 5, c);
MAKE_SET_IMPL(0xAA, SETD, 5, d);
MAKE_SET_IMPL(0xAB, SETE, 5, e);
MAKE_SET_IMPL(0xAC, SETH, 5, h);
MAKE_SET_IMPL(0xAD, SETL, 5, l);
MAKE_SET_IMPL_HL(0xAE, SETHLv, 5);
MAKE_SET_IMPL(0xAF, SETA, 5, a);

MAKE_SET_IMPL(0xB0, SETB, 6, b);
MAKE_SET_IMPL(0xB1, SETC, 6, c);
MAKE_SET_IMPL(0xB2, SETD, 6, d);
MAKE_SET_IMPL(0xB3, SETE, 6, e);
MAKE_SET_IMPL(0xB4, SETH, 6, h);
MAKE_SET_IMPL(0xB5, SETL, 6, l);
MAKE_SET_IMPL_HL(0xB6, SETHLv, 6);
MAKE_SET_IMPL(0xB7, SETA, 6, a);

MAKE_SET_IMPL(0xB8, SETB, 7, b);
MAKE_SET_IMPL(0xB9, SETC, 7, c);
MAKE_SET_IMPL(0xBA, SETD, 7, d);
MAKE_SET_IMPL(0xBB, SETE, 7, e);
MAKE_SET_IMPL(0xBC, SETH, 7, h);
MAKE_SET_IMPL(0xBD, SETL, 7, l);
MAKE_SET_IMPL_HL(0xBE, SETHLv, 7);
MAKE_SET_IMPL(0xBF, SETA, 7, a);

}
