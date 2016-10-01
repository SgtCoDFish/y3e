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

#include "Registers.hpp"

#include <iostream>
#include <iomanip>

namespace y3e {
// http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf pages 17-18
void Registers::initializeRegistersGB(Registers &reg) {
	reg.a = 0x01;
	Registers::initializeRegistersCommon(reg);
}

void Registers::initializeRegistersGBP(Registers &reg) {
	reg.a = 0xFF;
	Registers::initializeRegistersCommon(reg);
}

void Registers::initializeRegistersGBC(Registers &reg) {
	reg.a = 0x11;
	Registers::initializeRegistersCommon(reg);
}

void Registers::initializeRegistersCommon(Registers &reg) {
	reg.f = 0xB0;

	reg.bc = 0x0013;
	reg.de = 0x00D8;
	reg.hl = 0x014D;

	reg.sp = 0xFFFE;
	reg.pc = 0x0100;
}

void Registers::setFZero() {
	this->f |= 0b10000000;
}

void Registers::setFSubtract() {
	this->f |= 0b01000000;
}

void Registers::setFHalfCarry() {
	this->f |= 0b00100000;
}

void Registers::setFCarry() {
	this->f |= 0b00010000;
}

void Registers::resetFZero() {
	this->f &= 0b01111111;
}

void Registers::resetFSubtract() {
	this->f &= 0b10111111;
}

void Registers::resetFHalfCarry() {
	this->f &= 0b11011111;
}
void Registers::resetFCarry() {
	this->f &= 0b11101111;
}

// TODO: this could probably be faster with shifts
void Registers::setFZero(bool set) {
	set ? setFZero() : resetFZero();
}

void Registers::setFSubtract(bool set) {
	set ? setFSubtract() : resetFSubtract();
}

void Registers::setFHalfCarry(bool set) {
	set ? setFHalfCarry() : resetFHalfCarry();
}

void Registers::setFCarry(bool set) {
	set ? setFCarry() : resetFCarry();
}

bool Registers::isFZero() const {
	return this->f & 0b10000000;
}

bool Registers::isFSubtract() const {
	return this->f & 0b01000000;
}

bool Registers::isFHalfCarry() const {
	return this->f & 0b00100000;
}

bool Registers::isFCarry() const {
	return this->f & 0b00010000;
}

}

std::ostream &operator<<(std::ostream &stream, const y3e::Registers &reg) {
	stream.setf(std::ios::hex);

	stream << std::hex << "A: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.a & 0xFF) //
	        << "\tF: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.f & 0xFF);

	// output flags register in more readable form

	stream << " (Z: " << reg.isFZero() << "\tN: " << reg.isFSubtract() << ")\n";

	stream << "B: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.b & 0xFF) //
	        << "\tC: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.c & 0xFF);

	stream << " (H: " << reg.isFHalfCarry() << "\tC: " << reg.isFCarry() << ")\n";

	stream << "D: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.d & 0xFF) //
	        << "\tE: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.e & 0xFF) << std::endl;

	stream << "H: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.h & 0xFF) //
	        << "\tL: 0x" << std::setw(2) << std::setfill('0') << static_cast<int>(reg.l & 0xFF) << std::endl;

	stream << "SP: 0x" << std::setw(4) << std::setfill('0') << static_cast<int>(reg.sp & 0xFFFF) << std::endl;
	stream << "PC: 0x" << std::setw(4) << std::setfill('0') << static_cast<int>(reg.pc & 0xFFFF) << std::endl;

	stream.unsetf(std::ios::hex);

	return stream;
}
