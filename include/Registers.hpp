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

#ifndef CODE_TRUNK_INCLUDE_REGISTERS_HPP_
#define CODE_TRUNK_INCLUDE_REGISTERS_HPP_

#include <cstdint>

#include <iostream>

namespace y3e {

/**
 * A structure to hold the GB registers, with use of unions to match various
 * access types present in actual GB code.
 *
 * Note that this structure is NOT portable with regards to endian-ness.
 *
 * The set* and reset* methods control the flags register. Note that writing to the flags
 * register directly using external code will lead issues where the f register appears to be
 * incorrect when compared with other emulators. As such, it should be left to be handled
 * internally if at all possible.
 */
struct Registers {
	/*
	 * http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf - page 61
	 *
	 * A, F (AF), B, C (BC), D, E (DE) and H, L (HL) can all be accessed
	 * either individually as 8-bit registers or together as 16 bit types.
	 *
	 * F (flags) is an exception which holds bit-level flags relating to
	 * mathematical results such as overflow. On hardware it isn't directly
	 * accessible by the programmer.
	 *
	 * F layout
	 * --------
	 * 7 6 5 4 3 2 1 0
	 * Z N H C 0 0 0 0
	 *
	 * Z = Zero Flag
	 * -- Set if a result is 0 (for quick checking)
	 * -- Set if two values match using a comparison instruction
	 *
	 * N = Subtract Flag
	 * -- Set if the last mathematical instruciton was a subtraction.
	 *
	 * H = Half Carry Flag
	 * -- Set if the last mathematical instruction resulted in a carry
	 *    in the lower nibble.
	 *
	 * C = Carry Flag
	 * -- Set if the last mathematical instruction resulted in a carry
	 * -- Set if A was the smaller of two registers when executing a
	 *    comparison instruction.
	 *
	 * 0 = Unused
	 */

	struct {
		union {
			struct {
				uint8_t f; // hardware should not access directly.
				uint8_t a;
			};

			uint16_t af;
		};
	};

	struct {
		union {
			struct {
				uint8_t c;
				uint8_t b;
			};

			uint16_t bc;
		};
	};

	struct {
		union {
			struct {
				uint8_t e;
				uint8_t d;
			};

			uint16_t de;
		};
	};

	struct {
		union {
			struct {
				uint8_t l;
				uint8_t h;
			};

			uint16_t hl;
		};
	};

	uint16_t sp; // stack pointer
	uint16_t pc; // program counter

	void setFZero();
	void setFSubtract();
	void setFHalfCarry();
	void setFCarry();

	void resetFZero();
	void resetFSubtract();
	void resetFHalfCarry();
	void resetFCarry();

	void setFZero(bool set);
	void setFSubtract(bool set);
	void setFHalfCarry(bool set);
	void setFCarry(bool set);

	bool isFZero() const;
	bool isFSubtract() const;
	bool isFHalfCarry() const;
	bool isFCarry() const;

	/**
	 * Initialize registers for the Game Boy classic and Super Game Boy
	 */
	static void initializeRegistersGB(Registers &reg);

	/**
	 * Initialize registers for the Game Boy Pocket.
	 */
	static void initializeRegistersGBP(Registers &reg);

	/**
	 * Initialize registers for the Game Boy Color.
	 */
	static void initializeRegistersGBC(Registers &reg);

private:
	/**
	 * Most registers are the same between different platforms so this function
	 * avoids duplication.
	 */
	static void initializeRegistersCommon(Registers &reg);
};

}

std::ostream &operator<<(std::ostream &stream, const y3e::Registers &registers);

#endif /* CODE_TRUNK_INCLUDE_REGISTERS_HPP_ */
