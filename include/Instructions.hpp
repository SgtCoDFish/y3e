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

#ifndef CODE_TRUNK_INCLUDE_INSTRUCTIONS_HPP_
#define CODE_TRUNK_INCLUDE_INSTRUCTIONS_HPP_

#include <cstdint>

#include <fstream>
#include <memory>
#include <functional>
#include <vector>

namespace y3e {
class Emulator;

/**
 * Used to handle differences in using the program counter between instructions.
 * Most increment the program counter by their size, but, e.g. jump instructions
 * do just change the program counter and do not increment.
 */
using PCHandlerType = std::function<void(Emulator *, uint8_t)>;

/**
 * Holds information about the various Z80 instructions capable by the GB.
 *
 * Subclasses should implement the execute method to add a new instruction to
 * the emulator.
 *
 * Can be parsed by an InstructionParser subclass.
 *
 * If normalPCHandling is true, the program counter will be incremented by 1 every time
 *
 * Members
 * =======
 *
 * opcode
 * ------
 * See http://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html
 * The opcode of the instruction; for example in LD B,B the opcode is 0x40.
 *
 * instructionLength
 * -----------------
 * The length of the instruction in bytes between 1-3 inclusive.
 * 2 => One argument
 * 3 => Two arguments
 *
 * E.g. for LD B,B the length is 1.
 *
 * cyclesTaken
 * -----------
 * How long, in cycles, the instruction takes to run. For example,
 * LD B, B takes 4 cycles.
 *
 * humanReadable
 * -------------
 * The human-readable instruction, handy for debugging. E.g. for
 * LD B,B the human readable instruction would be "LD B,B"
 */
class Instruction {
public:
	/**
	 * Create a new instruction with the given settings.
	 * @param emulator_ The emulator to which the instruction is linked, and on which the instruction will run.
	 * @param opcode_ The opcode of the instruction (may not be accurate for "CB" instructions.
	 * @param instructionLength_ The length, in bytes, of the instruction.
	 * @param cyclesTaken_ The number of cycles, a multiple of 4, which the instruction takes to complete.
	 * @param humanReadable_ A human readable string of the instruction in pseudo-assembly
	 * @param normalPCHandling If true, the program counter is incremented as normal after the instruction is run.
	 * @param normalInstructionCycleHandling If true, cyclesTaken is a simple number. If false, it's calculated based on context.
	 */
	explicit Instruction(Emulator * emulator_, uint8_t opcode_, uint8_t instructionLength_, uint8_t cyclesTaken_,
	        const char * humanReadable_, bool normalPCHandling = true, bool normalInstructionCycleHandling = true);
	virtual ~Instruction() = default;

	Emulator * const emulator;

	const uint8_t opcode;
	const uint8_t instructionLength;
	const uint8_t cyclesTaken;

	const char * const humanReadable;

	/**
	 * Execute the instruction on the linked Emulator class given at construction.
	 */
	void execute();

	/**
	 * Subclasses should override to implement the actual handling of the instruction.
	 */
	virtual void executeImpl() = 0;

	/**
	 * Return the human readable representation of the instruction.
	 */
	inline const char * getHumanReadable() const {
		return humanReadable;
	}

	/**
	 * Return the number of cycles taken by the instruction, which may vary depending on context.
	 */
	inline uint8_t getCyclesTaken() const {
		return (normalCycleHandling ? cyclesTaken : determineCycles());
	}

protected:
	static void queueEI();
	static void queueDI();

	void doEI();
	void doDI();

	static PCHandlerType normalHandler;
	static PCHandlerType doNothingHandler;

	bool normalPCHandling;
	PCHandlerType &pcHandler;

	bool normalCycleHandling;

	/**
	 * Some instructions can take a different amount of time depending on
	 * their operation (e.g. 0x20 JR NZ, a8). This function should be overriden if
	 * abnormal cycle time handling is required.
	 *
	 * @return the actual number of cycles taken by the instruction.
	 */
	virtual uint8_t determineCycles() const {
		// do nothing in default implementation
		return cyclesTaken;
	}

private:
	static uint8_t parseInstructionArgument8(std::ifstream &input);
	static uint16_t parseInstructionArgument16(std::ifstream &input);

	/**
	 * eiQueue and diQueue implement interrupt enabling/disabling.
	 *
	 * Since EI and DI only execute after the next instruction has been processed,
	 * they have to be cached and executed later. The value of ei/diQueue is the number
	 * of instructions that must be executed before an actual EI/DI will be done.
	 *
	 * So, setting diQueue to 2 means that in 2 instructions DI will run, including the DI
	 * as parsed.
	 *
	 * If we have 2 instructions, DI and NOP, then setting diQueue to 2 in executeImpl of DI will
	 * ensure that interrupts are actually disabled after the NOP.
	 */
	static int8_t eiQueue;
	static int8_t diQueue;
};

/***************************************************************************
 * BEGIN INSTRUCTION LIST - Regular Instructions
 *
 * Instructions are implmented in various .cpp files; the "category" of each
 * should be listed in the line above the class definition.
 **************************************************************************/
/**
 * NOP
 * Do nothing
 * Misc
 */
class NOP final: public Instruction {
public:
	explicit NOP(Emulator * emulator_) :
			        Instruction(emulator_, 0x00, 1, 4, "NOP") {
	}

	virtual void executeImpl() override;
};

/**
 * LD BC, d16
 * Load a 16-bit value into BC.
 * Load16
 */
class LDBCd16 final : public Instruction {
public:
	explicit LDBCd16(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0x01, 3, 12, "LD BC, d16"),
			        value { value_ } {

	}

	virtual void executeImpl() override;

	const uint16_t value;
};

/**
 * LD (BC), A
 * Load the contents of A into the memory address stored in BC.
 * Load8
 */
class LDBCvA final : public Instruction {
public:
	explicit LDBCvA(Emulator * emulator_) :
			        Instruction(emulator_, 0x02, 1, 8, "LD (BC), A") {

	}

	virtual void executeImpl() override;
};

/**
 * INC BC
 * Increment BC by 1.
 * Arith16
 */
class INCBC final : public Instruction {
public:
	explicit INCBC(Emulator * emulator_) :
			        Instruction(emulator_, 0x03, 1, 8, "INC BC") {
	}

	virtual void executeImpl() override;
};

/**
 * INC B
 * Increase B by 1. Resets N, can set Z and H.
 * Arith8
 */
class INCB final : public Instruction {
public:
	explicit INCB(Emulator * emulator_) :
			        Instruction(emulator_, 0x04, 1, 4, "INC B") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC B
 * Decrease B by 1.
 *
 * Sets Z, H if needed. Sets N.
 *
 * Arith8
 */
class DECB final : public Instruction {
public:
	explicit DECB(Emulator * emulator_) :
			        Instruction(emulator_, 0x05, 1, 4, "DEC B") {
	}

	virtual void executeImpl() override;
};

/**
 * LD B, d8
 *
 * Load 8-bit value into B
 * Load8
 */
class LDBd8 final : public Instruction {
public:
	explicit LDBd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x06, 2, 8, "LD b, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RLCA
 * Rotate A left, including the carry flag.
 * Bit 7 -> New carry flag and moved to bit 0.
 * Rot
 */
class RLCA final : public Instruction {
public:
	explicit RLCA(Emulator * emulator_) :
			        Instruction(emulator_, 0x07, 1, 4, "RLCA") {
	}

	virtual void executeImpl() override;
};

/**
 * LD (a16), SP
 * Load the contents of SP into the memory at the given 16-bit address.
 * Load16
 */
class LDa16vSP final : public Instruction {
public:
	explicit LDa16vSP(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0x08, 3, 20, "LD (a16), SP"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;
};

/**
 * ADD HL, BC
 * Add BC to HL
 * Arith16
 */
class ADDHLBC final : public Instruction {
public:
	explicit ADDHLBC(Emulator * emulator_) :
			        Instruction(emulator_, 0x09, 1, 8, "ADD HL, BC") {
	}

	virtual void executeImpl() override;
};

/**
 * LD A, (BC)
 * Load the value in memory at the address in BC into A.
 * Load8
 */
class LDABCv final : public Instruction {
public:
	explicit LDABCv(Emulator * emulator_) :
			        Instruction(emulator_, 0x0A, 1, 8, "LD A, (BC)") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC BC
 * Decrease BC by 1.
 * Doesn't change flags.
 * Arith16
 */
class DECBC final : public Instruction {
public:
	explicit DECBC(Emulator * emulator_) :
			        Instruction(emulator_, 0x0B, 1, 8, "DEC BC") {
	}

	virtual void executeImpl() override;
};

/**
 * INC C
 * Increase C by 1.
 * Sets Z, H as appropriate, resets N.
 * Arith8
 */
class INCC final : public Instruction {
public:
	explicit INCC(Emulator * emulator_) :
			        Instruction(emulator_, 0x0C, 1, 4, "INC C") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC C
 * Decrease C by 1.
 *
 * Sets Z, H if needed. Sets N.
 *
 * Arith8
 */
class DECC final : public Instruction {
public:
	explicit DECC(Emulator * emulator_) :
			        Instruction(emulator_, 0x0D, 1, 4, "DEC C") {
	}

	virtual void executeImpl() override;
};

/**
 * LD C, d8
 * Load 8-bit value into C
 * Load8
 */
class LDCd8 final : public Instruction {
public:
	explicit LDCd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x0E, 2, 8, "LD c, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RRCA
 * Rotate A right through Carry.
 * Rotation
 */
class RRCA final : public Instruction {
public:
	explicit RRCA(Emulator * emulator_) :
			        Instruction(emulator_, 0x0F, 1, 4, "RRCA") {

	}

	virtual void executeImpl() override;
};

/**
 * STOP
 * Stop all execution until a button is pressed.
 * Misc
 */
class STOP final : public Instruction {
public:
	explicit STOP(Emulator * emulator_) :
			        Instruction(emulator_, 0x10, 2, 4, "STOP") {
	}

	virtual void executeImpl() override;
};

/**
 * LD DE, d16
 * Load a 16-bit value into DE
 * Load16
 */
class LDDEd16 final : public Instruction {
public:
	explicit LDDEd16(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0x11, 3, 12, "LD DE, d16"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;
};

/**
 * LD (DE), A
 * Place A into memory at the address stored in DE.
 * Load8
 */
class LDDEvA final : public Instruction {
public:
	explicit LDDEvA(Emulator * emulator_) :
			        Instruction(emulator_, 0x12, 1, 8, "LD (DE), A") {
	}

	virtual void executeImpl() override;
};

/**
 * INC DE
 * Increment DE by 1.
 * Arith16
 */
class INCDE final : public Instruction {
public:
	explicit INCDE(Emulator * emulator_) :
			        Instruction(emulator_, 0x13, 1, 8, "INC DE") {
	}

	virtual void executeImpl() override;
};

/**
 * INC D
 * Increase D by 1.
 * Sets Z, H as appropriate, resets N.
 * Arith8
 */
class INCD final : public Instruction {
public:
	explicit INCD(Emulator * emulator_) :
			        Instruction(emulator_, 0x14, 1, 4, "INC D") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC D
 * Decrease D by 1.
 *
 * Sets Z, H if needed. Sets N.
 *
 * Arith8
 */
class DECD final : public Instruction {
public:
	explicit DECD(Emulator * emulator_) :
			        Instruction(emulator_, 0x15, 1, 4, "DEC D") {
	}

	virtual void executeImpl() override;
};

/**
 * LD B, d8
 * Load 8-bit value into B
 * Load8
 */
class LDDd8 final : public Instruction {
public:
	explicit LDDd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x16, 2, 8, "LD B, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RLA
 * Note that this is similar to the CB-prefix instruction
 * but has a shorter instruction length and running time.
 */
class RLA final : public Instruction {
public:
	explicit RLA(Emulator * emulator_) :
			        Instruction(emulator_, 0x17, 1, 4, "RLA") {
	}

	virtual void executeImpl() override;
};

class JRr8 final : public Instruction {
public:
	explicit JRr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0x18, 2, 12, "JR r8", false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const int8_t value;
};

/**
 * ADD HL, DE
 * Add DE to HL
 * Arith16
 */
class ADDHLDE final : public Instruction {
public:
	explicit ADDHLDE(Emulator * emulator_) :
			        Instruction(emulator_, 0x19, 1, 8, "ADD HL, DE") {
	}

	virtual void executeImpl() override;
};

/**
 * LD A, (DE)
 * Load the value in memory at the address in DE into A.
 * Load8
 */
class LDADEv final : public Instruction {
public:
	explicit LDADEv(Emulator * emulator_) :
			        Instruction(emulator_, 0x1A, 1, 8, "LD A, (DE)") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC DE
 * Decrease DE by 1.
 * Doesn't change flags.
 * Arith16
 */
class DECDE final : public Instruction {
public:
	explicit DECDE(Emulator * emulator_) :
			        Instruction(emulator_, 0x1B, 1, 8, "DEC DE") {
	}

	virtual void executeImpl() override;
};

/**
 * INC E
 * Increase E by 1.
 * Sets Z, H as appropriate, resets N.
 * Arith8
 */
class INCE final : public Instruction {
public:
	explicit INCE(Emulator * emulator_) :
			        Instruction(emulator_, 0x1C, 1, 4, "INC E") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC E
 * Decrease E by 1.
 *
 * Sets Z, H if needed. Sets N.
 *
 * Arith8
 */
class DECE final : public Instruction {
public:
	explicit DECE(Emulator * emulator_) :
			        Instruction(emulator_, 0x1C, 1, 4, "DEC E") {
	}

	virtual void executeImpl() override;
};

/**
 * LD E, d8
 * Load an 8-bit value into E
 * Load8
 */
class LDEd8 final : public Instruction {
public:
	explicit LDEd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x1E, 2, 8, "LD E, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RRA
 * Note that this is similar to a CB prefix instruction but has to be implemented differently
 * given the different length and execution time.
 * Rot
 */
class RRA final : public Instruction {
public:
	explicit RRA(Emulator * emulator_) :
			        Instruction(emulator_, 0x1F, 1, 4, "RRA") {
	}

	virtual void executeImpl() override;
};

/**
 * JR NZ, r8
 * Jump relative by r8 (signed) if Zero flag (Z) is not set.
 * Jump
 */
class JRNZr8 final : public Instruction {
public:
	explicit JRNZr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0x20, 2, 12, "JR NZ, r8", false, false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const int8_t value;

protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * LD HL, d16
 * Set HL to a 16-bit value
 * Load16
 */
class LDHLd16 final : public Instruction {
public:
	explicit LDHLd16(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0x21, 3, 12, "LD HL, d16"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;
};

/**
 * LDI (HL), A
 * Load the value of A into the memory at the address stored in HL
 * Load8
 */
class LDIHLvA final : public Instruction {
public:
	explicit LDIHLvA(Emulator * emulator_) :
			        Instruction(emulator_, 0x22, 1, 8, "LDI (HL), A") {

	}

	virtual void executeImpl() override;
};

/**
 * INC HL
 * Increment HL
 * Arith16
 */
class INCHL final : public Instruction {
public:
	explicit INCHL(Emulator * emulator_) :
			        Instruction(emulator_, 0x23, 1, 8, "INC HL") {
	}

	virtual void executeImpl() override;
};

/**
 * INC H
 * Increment H
 * Resets N, sets Z, H as appropriate.
 * Arith8
 */
class INCH final : public Instruction {
public:
	explicit INCH(Emulator * emulator_) :
			        Instruction(emulator_, 0x24, 1, 4, "INC H") {

	}

	virtual void executeImpl() override;
};

/**
 * DEC H
 * Decrease H by 1.
 *
 * Sets Z, H if needed. Sets N.
 *
 * Arith8
 */
class DECH final : public Instruction {
public:
	explicit DECH(Emulator * emulator_) :
			        Instruction(emulator_, 0x25, 1, 4, "DEC H") {
	}

	virtual void executeImpl() override;
};

/**
 * LD H, d8
 * Load an 8-bit value into H.
 * Load8
 */
class LDHd8 final : public Instruction {
public:
	explicit LDHd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x26, 2, 8, "LD H, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * DAA
 * Sets A to be in BCD form
 * https://en.wikipedia.org/wiki/Binary-coded_decimal
 * Misc
 */
class DAA final : public Instruction {
public:
	explicit DAA(Emulator * emulator_) :
			        Instruction(emulator_, 0x27, 1, 4, "DAA") {
	}

	virtual void executeImpl() override;
};

/**
 * JR Z, r8
 * Jump if Z flag is set by the given signed 8-bit value
 * Jump
 */
class JRZr8 final : public Instruction {
public:
	explicit JRZr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0x28, 2, 12, "JR Z, r8", false, false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const int8_t value;

protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * ADD HL, HL
 * Add HL to HL
 * Arith16
 */
class ADDHLHL final : public Instruction {
public:
	explicit ADDHLHL(Emulator * emulator_) :
			        Instruction(emulator_, 0x29, 1, 8, "ADD HL, HL") {
	}

	virtual void executeImpl() override;
};

/**
 * LDI a, (HL)
 * Load the value at the address stored in HL into A, then increment HL.
 * Load8
 */
class LDIAHL final : public Instruction {
public:
	explicit LDIAHL(Emulator * emulator_) :
			        Instruction(emulator_, 0x2A, 1, 8, "LDI A, (HL)") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC HL
 * Decrease HL by 1.
 * Doesn't change flags.
 * Arith16
 */
class DECHL final : public Instruction {
public:
	explicit DECHL(Emulator * emulator_) :
			        Instruction(emulator_, 0x2B, 1, 8, "DEC HL") {
	}

	virtual void executeImpl() override;
};

/**
 * INC L
 * Increase L by 1. Resets N, can set Z and H.
 * Arith8
 */
class INCL final : public Instruction {
public:
	explicit INCL(Emulator * emulator_) :
			        Instruction(emulator_, 0x2C, 1, 4, "INC L") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC L
 * Decrease L by 1. Sets N, can set Z and H.
 * Arith8
 */
class DECL final : public Instruction {
public:
	explicit DECL(Emulator * emulator_) :
			        Instruction(emulator_, 0x2D, 1, 4, "DEC L") {
	}

	virtual void executeImpl() override;
};

/**
 * LD L, d8
 * Load an 8 bit value into L.
 * Load8
 */
class LDLd8 final : public Instruction {
public:
	explicit LDLd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x2E, 2, 8, "LD L, d8"),
			        value { value_ } {

	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * CPL
 * Complement every bit in A
 * Arith8
 */
class CPL final : public Instruction {
public:
	explicit CPL(Emulator * emulator_) :
			        Instruction(emulator_, 0x2F, 1, 4, "CPL") {
	}

	virtual void executeImpl() override;
};

/**
 * JR NC, r8
 * Jump relative by r8 (signed) if Carry flag (C) is not set.
 * Jump
 */
class JRNCr8 final : public Instruction {
public:
	explicit JRNCr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0x30, 2, 12, "JR NC, r8", false, false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const int8_t value;

protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * LD SP, d16
 * Load a 16 bit value into SP
 * Load16
 */
class LDSPd16 final : public Instruction {
public:
	explicit LDSPd16(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0x31, 3, 12, "LD SP, d16"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;
};

/**
 * LDD (HL), A
 * Place A into memory address HL, then HL--
 * Load8
 */
class LDDHLA final : public Instruction {
public:
	explicit LDDHLA(Emulator * emulator_) :
			        Instruction(emulator_, 0x32, 1, 8, "LDD (HL), A") {
	}

	virtual void executeImpl() override;
};

/**
 * INC SP
 * Increment SP by 1.
 * Arith16
 */
class INCSP final : public Instruction {
public:
	explicit INCSP(Emulator * emulator_) :
			        Instruction(emulator_, 0x33, 1, 8, "INC SP") {
	}

	virtual void executeImpl() override;
};

/**
 * INC (HL)
 * Increment the value at the address stored in HL 1.
 * Arith16
 */
class INCHLv final : public Instruction {
public:
	explicit INCHLv(Emulator * emulator_) :
			        Instruction(emulator_, 0x34, 1, 12, "INC (HL)") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC (HL)
 * Decrement the value at the address stored in HL 1.
 * Arith16
 */
class DECHLv final : public Instruction {
public:
	explicit DECHLv(Emulator * emulator_) :
			        Instruction(emulator_, 0x35, 1, 12, "DEC (HL)") {
	}

	virtual void executeImpl() override;
};

/**
 * LD (HL), d8
 * Place a given value into the memory at the address in HL
 * Load8
 */
class LDHLd8 final : public Instruction {
public:
	explicit LDHLd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x36, 2, 12, "LD (HL), d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * SCF
 * Set carry flag; sets C to 1 and resets N and H.
 * Misc
 */
class SCF final : public Instruction {
public:
	explicit SCF(Emulator * emulator_) :
			        Instruction(emulator_, 0x37, 1, 4, "SCF") {
	}

	virtual void executeImpl() override;
};

/**
 * JR C, r8
 * Jump if Z flag is set by the given signed 8-bit value
 * Jump
 */
class JRCr8 final : public Instruction {
public:
	explicit JRCr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0x38, 2, 12, "JR C, r8", false, false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const int8_t value;

protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * ADD HL, SP
 * Add SP to HL
 * Arith16
 */
class ADDHLSP final : public Instruction {
public:
	explicit ADDHLSP(Emulator * emulator_) :
			        Instruction(emulator_, 0x39, 1, 8, "ADD HL, SP") {
	}

	virtual void executeImpl() override;
};

/**
 * LDD A, (HL)
 * Load the value at the address stored in HL into A, and then decrement HL by 1.
 * Load8
 */
class LDDAHLv final : public Instruction {
public:
	explicit LDDAHLv(Emulator * emulator_) :
			        Instruction(emulator_, 0x3A, 1, 8, "LDD A, (HL)") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC SP
 * Decrease SP by 1.
 * Doesn't change flags.
 * Arith16
 */
class DECSP final : public Instruction {
public:
	explicit DECSP(Emulator * emulator_) :
			        Instruction(emulator_, 0x3B, 1, 8, "DEC SP") {
	}

	virtual void executeImpl() override;
};

/**
 * INC A
 * Increase A by 1. Resets N, can set Z and H.
 * Arith8
 */
class INCA final : public Instruction {
public:
	explicit INCA(Emulator * emulator_) :
			        Instruction(emulator_, 0x3C, 1, 4, "INC A") {
	}

	virtual void executeImpl() override;
};

/**
 * DEC A
 * Decrease A by 1. Sets N, can set Z and H.
 * Arith8
 */
class DECA final : public Instruction {
public:
	explicit DECA(Emulator * emulator_) :
			        Instruction(emulator_, 0x3D, 1, 4, "DEC A") {
	}

	virtual void executeImpl() override;
};

/**
 * LD A, d8
 * Load an 8 bit value into A.
 * Load8
 */
class LDAd8 final : public Instruction {
public:
	explicit LDAd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0x3E, 2, 8, "LD A, d8"),
			        value { value_ } {

	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * CCF
 * Complement Carry Flag
 * FCarry = ~FCarry
 * Misc
 */
class CCF final : public Instruction {
public:
	explicit CCF(Emulator * emulator_) :
			        Instruction(emulator_, 0x3F, 1, 4, "CCF") {
	}

	virtual void executeImpl() override;
};

#define MAKE_STANDARD_LD8(opcode, clname, reg1, reg2) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_):\
		Instruction(emulator_, (opcode), 1, 4, "LD "#reg1", "#reg2) {\
	}\
\
	virtual void executeImpl() override;\
}

#define MAKE_LD8_HLV(opcode, clname, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_):\
		Instruction(emulator_, (opcode), 1, 8, "LD "#reg", (HL)") {\
	}\
\
	virtual void executeImpl() override;\
}

#define MAKE_HLV_LD8(opcode, clname, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_):\
		Instruction(emulator_, (opcode), 1, 8, "LD (HL),"#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_LD8(0x40, LDBB, b, b);
MAKE_STANDARD_LD8(0x41, LDBC, b, c);
MAKE_STANDARD_LD8(0x42, LDBD, b, d);
MAKE_STANDARD_LD8(0x43, LDBE, b, e);
MAKE_STANDARD_LD8(0x44, LDBH, b, h);
MAKE_STANDARD_LD8(0x45, LDBL, b, l);
MAKE_LD8_HLV(0x46, LDBHLv, b);
MAKE_STANDARD_LD8(0x47, LDBA, b, a);

MAKE_STANDARD_LD8(0x48, LDCB, c, b);
MAKE_STANDARD_LD8(0x49, LDCC, c, c);
MAKE_STANDARD_LD8(0x4A, LDCD, c, d);
MAKE_STANDARD_LD8(0x4B, LDCE, c, e);
MAKE_STANDARD_LD8(0x4C, LDCH, c, h);
MAKE_STANDARD_LD8(0x4D, LDCL, c, l);
MAKE_LD8_HLV(0x4E, LDCHLv, c);
MAKE_STANDARD_LD8(0x4F, LDCA, c, a);

MAKE_STANDARD_LD8(0x50, LDDB, d, b);
MAKE_STANDARD_LD8(0x51, LDDC, d, c);
MAKE_STANDARD_LD8(0x52, LDDD, d, d);
MAKE_STANDARD_LD8(0x53, LDDE, d, e);
MAKE_STANDARD_LD8(0x54, LDDH, d, h);
MAKE_STANDARD_LD8(0x55, LDDL, d, l);
MAKE_LD8_HLV(0x56, LDDHLv, d);
MAKE_STANDARD_LD8(0x57, LDDA, d, a);

MAKE_STANDARD_LD8(0x58, LDEB, e, b);
MAKE_STANDARD_LD8(0x59, LDEC, e, c);
MAKE_STANDARD_LD8(0x5A, LDED, e, d);
MAKE_STANDARD_LD8(0x5B, LDEE, e, e);
MAKE_STANDARD_LD8(0x5C, LDEH, e, h);
MAKE_STANDARD_LD8(0x5D, LDEL, e, l);
MAKE_LD8_HLV(0x5E, LDEHLv, e);
MAKE_STANDARD_LD8(0x5F, LDEA, e, a);

MAKE_STANDARD_LD8(0x60, LDHB, h, b);
MAKE_STANDARD_LD8(0x61, LDHC, h, c);
MAKE_STANDARD_LD8(0x62, LDHD, h, d);
MAKE_STANDARD_LD8(0x63, LDHE, h, e);
MAKE_STANDARD_LD8(0x64, LDHH, h, h);
MAKE_STANDARD_LD8(0x65, LDHL, h, l);
MAKE_LD8_HLV(0x66, LDHHLv, h);
MAKE_STANDARD_LD8(0x67, LDHA, h, a);

MAKE_STANDARD_LD8(0x68, LDLB, l, b);
MAKE_STANDARD_LD8(0x69, LDLC, l, c);
MAKE_STANDARD_LD8(0x6A, LDLD, l, d);
MAKE_STANDARD_LD8(0x6B, LDLE, l, e);
MAKE_STANDARD_LD8(0x6C, LDLH, l, h);
MAKE_STANDARD_LD8(0x6D, LDLL, l, l);
MAKE_LD8_HLV(0x6E, LDLHLv, l);
MAKE_STANDARD_LD8(0x6F, LDLA, l, a);

MAKE_HLV_LD8(0x70, LDHLvB, b);
MAKE_HLV_LD8(0x71, LDHLvC, c);
MAKE_HLV_LD8(0x72, LDHLvD, d);
MAKE_HLV_LD8(0x73, LDHLvE, e);
MAKE_HLV_LD8(0x74, LDHLvH, h);
MAKE_HLV_LD8(0x75, LDHLvL, l);
MAKE_HLV_LD8(0x77, LDHLvA, a);

/**
 * HALT
 * Halt the processor's execution until something happens.
 * Misc
 */
class HALT : public Instruction {
public:
	explicit HALT(Emulator * emulator_) :
			        Instruction(emulator_, 0x76, 1, 4, "HALT") {
	}

	virtual void executeImpl() override;
};

MAKE_STANDARD_LD8(0x78, LDAB, a, b);
MAKE_STANDARD_LD8(0x79, LDAC, a, c);
MAKE_STANDARD_LD8(0x7A, LDAD, a, d);
MAKE_STANDARD_LD8(0x7B, LDAE, a, e);
MAKE_STANDARD_LD8(0x7C, LDAH, a, h);
MAKE_STANDARD_LD8(0x7D, LDAL, a, l);
MAKE_LD8_HLV(0x7E, LDAHLv, a);
MAKE_STANDARD_LD8(0x7F, LDAA, a, a);

#define MAKE_STANDARD_ADD(opcode, clname, cycles, reg1, reg2) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
					Instruction(emulator_, (opcode), 1, (cycles), "ADD "#reg1", "#reg2) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_ADD(0x80, ADDAB, 4, a, b);
MAKE_STANDARD_ADD(0x81, ADDAC, 4, a, c);
MAKE_STANDARD_ADD(0x82, ADDAD, 4, a, d);
MAKE_STANDARD_ADD(0x83, ADDAE, 4, a, e);
MAKE_STANDARD_ADD(0x84, ADDAH, 4, a, h);
MAKE_STANDARD_ADD(0x85, ADDAL, 4, a, l);
MAKE_STANDARD_ADD(0x86, ADDAHLv, 8, a, hlv);
MAKE_STANDARD_ADD(0x87, ADDAA, 4, a, a);

#define MAKE_STANDARD_ADC(opcode, clname, length, cycles, reg) class clname final : public Instruction{\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), (length), (cycles), "ADC A, "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_ADC(0x88, ADCAB, 1, 4, b);
MAKE_STANDARD_ADC(0x89, ADCAC, 1, 4, c);
MAKE_STANDARD_ADC(0x8A, ADCAD, 1, 4, d);
MAKE_STANDARD_ADC(0x8B, ADCAE, 1, 4, e);
MAKE_STANDARD_ADC(0x8C, ADCAH, 1, 4, h);
MAKE_STANDARD_ADC(0x8D, ADCAL, 1, 4, l);
MAKE_STANDARD_ADC(0x8E, ADCAHLv, 1, 8, hlv);
MAKE_STANDARD_ADC(0x8F, ADCAA, 1, 4, a);

#define MAKE_STANDARD_SUB(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
					Instruction(emulator_, (opcode), 1, (cycles), "SUB "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_SUB(0x90, SUBB, 4, b);
MAKE_STANDARD_SUB(0x91, SUBC, 4, c);
MAKE_STANDARD_SUB(0x92, SUBD, 4, d);
MAKE_STANDARD_SUB(0x93, SUBE, 4, e);
MAKE_STANDARD_SUB(0x94, SUBH, 4, h);
MAKE_STANDARD_SUB(0x95, SUBL, 4, l);
MAKE_STANDARD_SUB(0x96, SUBHLv, 8, hlv);
MAKE_STANDARD_SUB(0x97, SUBA, 4, a);

#define MAKE_STANDARD_SBC(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 1, (cycles), "SBC A, "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_SBC(0x98, SBCAB, 4, b);
MAKE_STANDARD_SBC(0x99, SBCAC, 4, c);
MAKE_STANDARD_SBC(0x9A, SBCAD, 4, d);
MAKE_STANDARD_SBC(0x9B, SBCAE, 4, e);
MAKE_STANDARD_SBC(0x9C, SBCAH, 4, h);
MAKE_STANDARD_SBC(0x9D, SBCAL, 4, l);
MAKE_STANDARD_SBC(0x9E, SBCAHLv, 8, hlv);
MAKE_STANDARD_SBC(0x9F, SBCAA, 4, a);

#define MAKE_STANDARD_AND(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
					Instruction(emulator_, (opcode), 1, (cycles), "AND "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_AND(0xA0, ANDB, 4, b);
MAKE_STANDARD_AND(0xA1, ANDC, 4, b);
MAKE_STANDARD_AND(0xA2, ANDD, 4, b);
MAKE_STANDARD_AND(0xA3, ANDE, 4, b);
MAKE_STANDARD_AND(0xA4, ANDH, 4, b);
MAKE_STANDARD_AND(0xA5, ANDL, 4, b);
MAKE_STANDARD_AND(0xA6, ANDHLv, 8, b);
MAKE_STANDARD_AND(0xA7, ANDA, 4, a);

#define MAKE_STANDARD_XOR(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
			        Instruction(emulator_, (opcode), 1, (cycles), "XOR "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_XOR(0xA8, XORB, 4, b);
MAKE_STANDARD_XOR(0xA9, XORC, 4, c);
MAKE_STANDARD_XOR(0xAA, XORD, 4, d);
MAKE_STANDARD_XOR(0xAB, XORE, 4, e);
MAKE_STANDARD_XOR(0xAC, XORH, 4, h);
MAKE_STANDARD_XOR(0xAD, XORL, 4, l);
MAKE_STANDARD_XOR(0xAE, XORHLv, 8, hlv);
MAKE_STANDARD_XOR(0xAF, XORA, 4, a);

#define MAKE_STANDARD_OR(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 1, (cycles), "OR "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

// Arith8
MAKE_STANDARD_OR(0xB0, ORB, 4, b);
MAKE_STANDARD_OR(0xB1, ORC, 4, c);
MAKE_STANDARD_OR(0xB2, ORD, 4, d);
MAKE_STANDARD_OR(0xB3, ORE, 4, e);
MAKE_STANDARD_OR(0xB4, ORH, 4, h);
MAKE_STANDARD_OR(0xB5, ORL, 4, l);
MAKE_STANDARD_OR(0xB6, ORHLv, 8, hlv);
MAKE_STANDARD_OR(0xB7, ORA, 4, a);

#define MAKE_STANDARD_CP(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 1, (cycles), "CP "#reg) {\
		}\
\
	virtual void executeImpl() override;\
}

MAKE_STANDARD_CP(0xB8, CPB, 4, b);
MAKE_STANDARD_CP(0xB9, CPC, 4, c);
MAKE_STANDARD_CP(0xBA, CPD, 4, d);
MAKE_STANDARD_CP(0xBB, CPE, 4, e);
MAKE_STANDARD_CP(0xBC, CPH, 4, h);
// Weird name to not clash with CPL instruction
MAKE_STANDARD_CP(0xBD, CPwithL, 4, l);
MAKE_STANDARD_CP(0xBE, CPHLv, 8, hlv);
MAKE_STANDARD_CP(0xBF, CPA, 4, a);

/**
 * RET NZ
 * Return if Z is not set.
 * Jump
 */
class RETNZ final : public Instruction {
public:
	explicit RETNZ(Emulator * emulator_) :
			        Instruction(emulator_, 0xC0, 1, 20, "RET NZ", false, false) {
	}

	virtual void executeImpl() override;

protected:
	virtual uint8_t determineCycles() const override;
};

#define MAKE_POP(opcode, clname, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 1, 12, "POP "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_POP(0xC1, POPBC, bc);

/**
 * JP NZ a16
 * Jump conditionally to a given 16-bit address if Z is not set.
 * Jump
 */
class JPNZa16 final : public Instruction {
public:
	explicit JPNZa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xC2, 3, 16, "JP NZ a16", false, false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;
protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * JP a16
 * Jump to 16 bit address
 * Jump
 */
class JP16 final : public Instruction {
public:
	explicit JP16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xC3, 3, 16, "JP a16", false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;
};

/**
 * CALL NZ, a16
 * Call a function at a given 16-bit address if Z is reset.
 *
 * This involves pushing the address of the next instruction onto the stack, and then
 * jumping to the given address.
 *
 * Jump
 */
class CALLNZa16 final : public Instruction {
public:
	explicit CALLNZa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xC4, 3, 24, "CALL NZ, a16", false, false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;

protected:
	uint8_t determineCycles() const override;
};

#define MAKE_PUSH(opcode, clname, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 1, 16, "PUSH "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_PUSH(0xC5, PUSHBC, bc);

/**
 * ADD A, d8
 * Add an 8 bit value to A.
 * Arith8
 */
class ADDAd8 final : public Instruction {
public:
	explicit ADDAd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xC6, 2, 8, "ADD A, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RST 00
 * Push address to stack, jump to 0x0000.
 * Jump
 */
class RST00 final : public Instruction {
public:
	explicit RST00(Emulator * emulator_) :
			        Instruction(emulator_, 0xC7, 1, 16, "RST 00", false) {
	}

	virtual void executeImpl() override;
};

/**
 * RET Z
 * Return if Z is set.
 * Jump
 */
class RETZ final : public Instruction {
public:
	explicit RETZ(Emulator * emulator_) :
			        Instruction(emulator_, 0xC8, 1, 20, "RET Z", false, false) {

	}

	virtual void executeImpl() override;

protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * RET
 * Pop an address from the stack and jump to it.
 * Jump
 */
class RET final : public Instruction {
public:
	explicit RET(Emulator * emulator_) :
			        Instruction(emulator_, 0xC9, 1, 16, "RET", false) {
	}

	virtual void executeImpl() override;
};

/**
 * JP Z, a16
 * Jump to the given address if Z is set.
 * Jump
 */
class JPZa16 final : public Instruction {
public:
	explicit JPZa16(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0xCA, 3, 16, "JP Z, a16", false, false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;

protected:
	uint8_t determineCycles() const override;
};

/**
 * CALL Z, a16
 * Call a function at a given 16-bit address if Z is set.
 *
 * This involves pushing the address of the next instruction onto the stack, and then
 * jumping to the given address.
 *
 * Jump
 */
class CALLZa16 final : public Instruction {
public:
	explicit CALLZa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xCC, 3, 24, "CALL Z, a16", false, false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;

protected:
	uint8_t determineCycles() const override;
};

/**
 * CALL a16
 * Call a function at a given 16-bit address.
 *
 * This involves pushing the address of the next instruction onto the stack, and then
 * jumping to the given address.
 *
 * Jump
 */
class CALLa16 final : public Instruction {
public:
	explicit CALLa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xCD, 3, 24, "CALL a16", false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;
};

/**
 * ADC A, d8
 * Add an 8 bit value and the value of the C flag to A.
 * Arith8
 */
class ADCAd8 final : public Instruction {
public:
	explicit ADCAd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xCE, 2, 8, "ADC A, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RST 0x08
 * Push current address onto stack, jump to 0x08
 * Jump
 */
class RST08 final : public Instruction {
public:
	explicit RST08(Emulator * emulator_) :
			        Instruction(emulator_, 0xCF, 1, 16, "RST 0x08", false) {
	}

	virtual void executeImpl() override;
};

/**
 * RET NC
 * Return if the carry flag is not set.
 * Jump
 */
class RETNC final : public Instruction {
public:
	explicit RETNC(Emulator * emulator_) :
			        Instruction(emulator_, 0xD0, 1, 20, "RET NC", false, false) {
	}

	virtual void executeImpl() override;

protected:
	virtual uint8_t determineCycles() const override;
};

MAKE_POP(0xD1, POPDE, de);

/**
 * JP NC a16
 * Jump conditionally to a given 16-bit address if C is not set.
 * Jump
 */
class JPNCa16 final : public Instruction {
public:
	explicit JPNCa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xD2, 3, 16, "JP NC a16", false, false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;
protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * CALL NC, a16
 * Call a 16-bit address if C is not set.
 * Jump
 */
class CALLNCa16 final : public Instruction {
public:
	explicit CALLNCa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xD4, 3, 24, "CALL NC, a16", false, false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;

protected:
	virtual uint8_t determineCycles() const override;
};

MAKE_PUSH(0xD5, PUSHDE, de);

/**
 * SUBd8
 * Subtract an 8 bit value from A
 * Arith8
 */
class SUBd8 final : public Instruction {
public:
	explicit SUBd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xD6, 2, 8, "SUB d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;
	const uint8_t value;
};

/**
 * RST 10
 * Push address onto stack, jump to 0x0010.
 * Jump
 */
class RST10 final : public Instruction {
public:
	explicit RST10(Emulator * emulator_) :
			        Instruction(emulator_, 0xD7, 1, 16, "RST 10", false) {
	}

	virtual void executeImpl() override;
};

/**
 * RET C
 * Return if C is set.
 * Jump
 */
class RETC final : public Instruction {
public:
	explicit RETC(Emulator * emulator_) :
			        Instruction(emulator_, 0xD8, 1, 20, "RET C", false, false) {

	}

	virtual void executeImpl() override;

protected:
	virtual uint8_t determineCycles() const override;
};

/**
 * RETI
 * Return and enable interrupts.
 * Jump
 */
class RETI final : public Instruction {
public:
	explicit RETI(Emulator * emulator_) :
			        Instruction(emulator_, 0xD9, 1, 16, "RETI", false) {
	}

	virtual void executeImpl() override;
};

/**
 * JP C, a16
 * Jump to the given address if C is set.
 * Jump
 */
class JPCa16 final : public Instruction {
public:
	explicit JPCa16(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0xDA, 3, 16, "JP C, a16", false, false),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;

protected:
	uint8_t determineCycles() const override;
};

/**
 * CALL C, a16
 * Call a function at a given 16-bit address if C is set.
 *
 * This involves pushing the address of the next instruction onto the stack, and then
 * jumping to the given address.
 *
 * Jump
 */
class CALLCa16 final : public Instruction {
public:
	explicit CALLCa16(Emulator * emulator_, uint16_t address_) :
			        Instruction(emulator_, 0xDC, 3, 24, "CALL C, a16", false, false),
			        address { address_ } {
	}

	virtual void executeImpl() override;

	const uint16_t address;

protected:
	uint8_t determineCycles() const override;
};

/**
 * SBC A, d8
 * Subract a value from A, and also subtract the value of the carry flag.
 * Arith8
 */
class SBCAd8 final : public Instruction {
public:
	explicit SBCAd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xDE, 2, 8, "SBC A, d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RST 0x18
 * Push current address onto stack, jump to 0x18
 * Jump
 */
class RST18 final : public Instruction {
public:
	explicit RST18(Emulator * emulator_) :
			        Instruction(emulator_, 0xDF, 1, 16, "RST 0x18", false) {
	}

	virtual void executeImpl() override;
};

/**
 * LDH (a8), A
 * Put the contents of A into memory address 0xFF00 + value.
 * Load8
 */
class LDHa8A final: public Instruction {
public:
	explicit LDHa8A(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xE0, 2, 12, "LDH (a8), A"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

MAKE_POP(0xE1, POPHL, hl);
MAKE_PUSH(0xE5, PUSHHL, hl);

/**
 * LD (C), A
 * Put A into memory at address 0xFF00 + C.
 * Load8
 */
class LDCVA final : public Instruction {
public:
	explicit LDCVA(Emulator * emulator_) :
			        Instruction(emulator_, 0xE2, 1, 8, "LD (C), A") {
	}

	virtual void executeImpl() override;
};

/**
 * AND d8
 * Calculate register A AND an 8 bit value
 * Arith8
 */
class ANDd8 final : public Instruction {
public:
	explicit ANDd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xE6, 2, 8, "AND d8"),
			        value { value_ } {

	}

	virtual void executeImpl() override;
	const uint8_t value;
};

/**
 * RST 20
 * Jump to 0x0020.
 * Jump
 */
class RST20 final : public Instruction {
public:
	explicit RST20(Emulator * emulator_) :
			        Instruction(emulator_, 0xE7, 1, 16, "RST 20", false) {
	}

	virtual void executeImpl() override;
};

/**
 * ADD SP, r8
 * Adds a signed 8 bit value to the stack pointer register.
 * Arith16
 */
class ADDSPr8 final : public Instruction {
public:
	explicit ADDSPr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0xE8, 2, 16, "ADD SP, r8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const int8_t value;
};

class JPHL final : public Instruction {
public:
	explicit JPHL(Emulator * emulator_) :
			        Instruction(emulator_, 0xE9, 1, 4, "JP HL", false) {
	}

	virtual void executeImpl() override;
};

/**
 * LD (a16), A
 * Put A into memory at address a16.
 * Load16
 */
class LDa16A final: public Instruction {
public:
	explicit LDa16A(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0xEA, 3, 16, "LD (a16), A"),
			        value { value_ } {
	}

	virtual void executeImpl() override;
	const uint16_t value;
};

/**
 * XOR d8
 * Calculate A = A XOR d8
 * Arith8
 */
class XORd8 final : public Instruction {
public:
	explicit XORd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xEE, 2, 8, "XOR d8"),
			        value { value_ } {

	}

	virtual void executeImpl() override;
	const uint8_t value;
};

/**
 * RST 0x38
 * Push current address onto stack, jump to 0x28
 * Jump
 */
class RST28 final : public Instruction {
public:
	explicit RST28(Emulator * emulator_) :
			        Instruction(emulator_, 0xEF, 1, 16, "RST 0x28", false) {
	}

	virtual void executeImpl() override;
};

/**
 * LDH A, a8
 * Load into A the value from memory at 0xFF00 + a8.
 * Load8
 */
class LDHAa8 final : public Instruction {
public:
	explicit LDHAa8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xF0, 2, 12, "LDH A, (a8)"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

MAKE_POP(0xF1, POPAF, af);

/**
 * LD A, (C)
 * Load the value at (0xFF00 + the value in C) into A.
 * Load8
 */
class LDACv final : public Instruction {
public:
	explicit LDACv(Emulator * emulator_) :
			        Instruction(emulator_, 0xF2, 2, 8, "LD A, (C)") {
	}

	virtual void executeImpl() override;
};

/**
 * DI
 * Disable interrupts after the next instruction is executed.
 * Misc
 */
class DI final: public Instruction {
public:
	explicit DI(Emulator * emulator_) :
			        Instruction(emulator_, 0xF3, 1, 4, "DI") {
	}

	virtual void executeImpl() override;
};

MAKE_PUSH(0xF5, PUSHAF, af);

/**
 * OR d8
 * Executes A | some value.
 * Arith8
 */
class ORd8 final : public Instruction {
public:
	explicit ORd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xF6, 2, 8, "OR d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RST 30
 * Push address on stack, jump to 0x0030.
 * Jump
 */
class RST30 final : public Instruction {
public:
	explicit RST30(Emulator * emulator_) :
			        Instruction(emulator_, 0xF7, 1, 16, "RST 30", false) {
	}

	virtual void executeImpl() override;
};

/**
 * LD HL, SP+r8
 * Load SP + an 8-bit signed integer into HL
 * Arith16
 */
class LDHLSPr8 final : public Instruction {
public:
	explicit LDHLSPr8(Emulator * emulator_, int8_t value_) :
			        Instruction(emulator_, 0xF8, 2, 12, "LD HL, SP+r8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;
	const int8_t value;
};

/**
 * LD SP, HL
 * Load SP into HL
 * Load16
 */
class LDSPHL final : public Instruction {
public:
	explicit LDSPHL(Emulator * emulator_) :
			        Instruction(emulator_, 0xF9, 1, 8, "LD SP, HL") {
	}

	virtual void executeImpl() override;
};

/**
 * LD A, (a16)
 * Load the value at the address given into A.
 * Load8
 */
class LDAa16v final : public Instruction {
public:
	explicit LDAa16v(Emulator * emulator_, uint16_t value_) :
			        Instruction(emulator_, 0xFA, 3, 16, "LD A, (a16)"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint16_t value;
};

/**
 * EI
 * Enable interrupts after the next instruction is executed.
 * Misc
 */
class EI final : public Instruction {
public:
	explicit EI(Emulator * emulator_) :
			        Instruction(emulator_, 0xFB, 1, 4, "EI") {
	}

	virtual void executeImpl() override;
};

/**
 * CP d8
 * Compare A with d8, setting the Z flag if equal.
 * Sets N, can set H and C.
 * Arith8
 */
class CPd8 final: public Instruction {
public:
	explicit CPd8(Emulator * emulator_, uint8_t value_) :
			        Instruction(emulator_, 0xFE, 2, 8, "CP d8"),
			        value { value_ } {
	}

	virtual void executeImpl() override;

	const uint8_t value;
};

/**
 * RST 0x38
 * Push current address onto stack, jump to 0x38
 * Jump
 */
class RST38 final : public Instruction {
public:
	explicit RST38(Emulator * emulator_) :
			        Instruction(emulator_, 0xFF, 1, 16, "RST 0x38", false) {
	}

	virtual void executeImpl() override;
};

/***************************************************************************
 * BEGIN INSTRUCTION LIST - CB Instructions
 *
 * Implmented in RotationInstructions
 *
 * Instructions are implmented in various .cpp files; the "category" of each
 * should be listed in the line above the class definition.
 **************************************************************************/

#define MAKE_RLC(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "RLC "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_RLC(0x00, RLCB, 8, b);
MAKE_RLC(0x01, RLCC, 8, c);
MAKE_RLC(0x02, RLCD, 8, d);
MAKE_RLC(0x03, RLCE, 8, e);
MAKE_RLC(0x04, RLCH, 8, h);
MAKE_RLC(0x05, RLCL, 8, l);
MAKE_RLC(0x06, RLCHLv, 16, hlv);
MAKE_RLC(0x07, RLCACB, 8, a);

#define MAKE_RRC(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "RRC "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_RRC(0x08, RRCB, 8, b);
MAKE_RRC(0x09, RRCC, 8, c);
MAKE_RRC(0x0A, RRCD, 8, d);
MAKE_RRC(0x0B, RRCE, 8, e);
MAKE_RRC(0x0C, RRCH, 8, h);
MAKE_RRC(0x0D, RRCL, 8, l);
MAKE_RRC(0x0E, RRCHLv, 16, hlv);
MAKE_RRC(0x0F, RRCACB, 8, a); // RRCA is also defined at regular 0x0F but is different!

#define MAKE_RL(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "RL "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_RL(0x10, RLB, 8, b);
MAKE_RL(0x11, RLC, 8, c);
MAKE_RL(0x12, RLD, 8, d);
MAKE_RL(0x13, RLE, 8, e);
MAKE_RL(0x14, RLH, 8, h);
MAKE_RL(0x15, RLL, 8, l);
MAKE_RL(0x16, RLHLv, 16, HLv);
MAKE_RL(0x17, RLACB, 8, a);

#define MAKE_RR(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "RR "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_RR(0x18, RRB, 8, b);
MAKE_RR(0x19, RRC, 8, c);
MAKE_RR(0x1A, RRD, 8, d);
MAKE_RR(0x1B, RRE, 8, e);
MAKE_RR(0x1C, RRH, 8, h);
MAKE_RR(0x1D, RRL, 8, l);
MAKE_RR(0x1E, RRHLv, 16, b);
MAKE_RR(0x1F, RRACB, 8, a);

#define MAKE_SLA(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "SLA "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_SLA(0x20, SLAB, 8, b);
MAKE_SLA(0x21, SLAC, 8, c);
MAKE_SLA(0x22, SLAD, 8, d);
MAKE_SLA(0x23, SLAE, 8, e);
MAKE_SLA(0x24, SLAH, 8, h);
MAKE_SLA(0x25, SLAL, 8, l);
MAKE_SLA(0x26, SLAHLv, 8, hlv);
MAKE_SLA(0x27, SLAA, 8, a);

#define MAKE_SRA(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "SRA "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_SRA(0x28, SRAB, 8, b);
MAKE_SRA(0x29, SRAC, 8, c);
MAKE_SRA(0x2A, SRAD, 8, d);
MAKE_SRA(0x2B, SRAE, 8, e);
MAKE_SRA(0x2C, SRAH, 8, h);
MAKE_SRA(0x2D, SRAL, 8, l);
MAKE_SRA(0x2E, SRAHLv, 16, hlv);
MAKE_SRA(0x2F, SRAA, 8, a);

#define MAKE_SWAP(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "SWAP "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_SWAP(0x30, SWAPB, 4, b);
MAKE_SWAP(0x31, SWAPC, 4, c);
MAKE_SWAP(0x32, SWAPD, 4, d);
MAKE_SWAP(0x33, SWAPE, 4, e);
MAKE_SWAP(0x34, SWAPH, 4, h);
MAKE_SWAP(0x35, SWAPL, 4, l);
MAKE_SWAP(0x35, SWAPHLv, 8, hlv);
MAKE_SWAP(0x37, SWAPA, 4, a);

#define MAKE_SRL(opcode, clname, cycles, reg) class clname final : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), "SRL "#reg) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_SRL(0x38, SRLB, 8, b);
MAKE_SRL(0x39, SRLC, 8, c);
MAKE_SRL(0x3A, SRLD, 8, d);
MAKE_SRL(0x3B, SRLE, 8, e);
MAKE_SRL(0x3C, SRLH, 8, h);
MAKE_SRL(0x3D, SRLL, 8, l);
MAKE_SRL(0x3E, SRLHLv, 8, hlv);
MAKE_SRL(0x3F, SRLA, 8, a);

#define MAKE_BIT(opcode, clname, cycles) class clname : public Instruction {\
public:\
	explicit clname(Emulator * emulator_) : \
		Instruction(emulator_, (opcode), 2, (cycles), #clname) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_BIT(0x40, BIT0B, 8);
MAKE_BIT(0x41, BIT0C, 8);
MAKE_BIT(0x42, BIT0D, 8);
MAKE_BIT(0x43, BIT0E, 8);
MAKE_BIT(0x44, BIT0H, 8);
MAKE_BIT(0x45, BIT0L, 8);
MAKE_BIT(0x46, BIT0HLv, 16);
MAKE_BIT(0x47, BIT0A, 8);

MAKE_BIT(0x48, BIT1B, 8);
MAKE_BIT(0x49, BIT1C, 8);
MAKE_BIT(0x4A, BIT1D, 8);
MAKE_BIT(0x4B, BIT1E, 8);
MAKE_BIT(0x4C, BIT1H, 8);
MAKE_BIT(0x4D, BIT1L, 8);
MAKE_BIT(0x4E, BIT1HLv, 16);
MAKE_BIT(0x4F, BIT1A, 8);

MAKE_BIT(0x50, BIT2B, 8);
MAKE_BIT(0x51, BIT2C, 8);
MAKE_BIT(0x52, BIT2D, 8);
MAKE_BIT(0x53, BIT2E, 8);
MAKE_BIT(0x54, BIT2H, 8);
MAKE_BIT(0x55, BIT2L, 8);
MAKE_BIT(0x56, BIT2HLv, 16);
MAKE_BIT(0x57, BIT2A, 8);

MAKE_BIT(0x58, BIT3B, 8);
MAKE_BIT(0x59, BIT3C, 8);
MAKE_BIT(0x5A, BIT3D, 8);
MAKE_BIT(0x5B, BIT3E, 8);
MAKE_BIT(0x5C, BIT3H, 8);
MAKE_BIT(0x5D, BIT3L, 8);
MAKE_BIT(0x5E, BIT3HLv, 16);
MAKE_BIT(0x5F, BIT3A, 8);

MAKE_BIT(0x60, BIT4B, 8);
MAKE_BIT(0x61, BIT4C, 8);
MAKE_BIT(0x62, BIT4D, 8);
MAKE_BIT(0x63, BIT4E, 8);
MAKE_BIT(0x64, BIT4H, 8);
MAKE_BIT(0x65, BIT4L, 8);
MAKE_BIT(0x66, BIT4HLv, 16);
MAKE_BIT(0x67, BIT4A, 8);

MAKE_BIT(0x68, BIT5B, 8);
MAKE_BIT(0x69, BIT5C, 8);
MAKE_BIT(0x6A, BIT5D, 8);
MAKE_BIT(0x6B, BIT5E, 8);
MAKE_BIT(0x6C, BIT5H, 8);
MAKE_BIT(0x6D, BIT5L, 8);
MAKE_BIT(0x6E, BIT5HLv, 16);
MAKE_BIT(0x6F, BIT5A, 8);

MAKE_BIT(0x70, BIT6B, 8);
MAKE_BIT(0x71, BIT6C, 8);
MAKE_BIT(0x72, BIT6D, 8);
MAKE_BIT(0x73, BIT6E, 8);
MAKE_BIT(0x74, BIT6H, 8);
MAKE_BIT(0x75, BIT6L, 8);
MAKE_BIT(0x76, BIT6HLv, 16);
MAKE_BIT(0x77, BIT6A, 8);

MAKE_BIT(0x78, BIT7B, 8);
MAKE_BIT(0x79, BIT7C, 8);
MAKE_BIT(0x7A, BIT7D, 8);
MAKE_BIT(0x7B, BIT7E, 8);
MAKE_BIT(0x7C, BIT7H, 8);
MAKE_BIT(0x7D, BIT7L, 8);
MAKE_BIT(0x7E, BIT7HLv, 16);
MAKE_BIT(0x7F, BIT7A, 8);

#define MAKE_RES(opcode, clname, bit, cycles) class clname##bit final : public Instruction {\
public:\
	explicit clname##bit(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), #clname""#bit) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_RES(0x80, RESB, 0, 8);
MAKE_RES(0x81, RESC, 0, 8);
MAKE_RES(0x82, RESD, 0, 8);
MAKE_RES(0x83, RESE, 0, 8);
MAKE_RES(0x84, RESH, 0, 8);
MAKE_RES(0x85, RESL, 0, 8);
MAKE_RES(0x86, RESHLv, 0, 16);
MAKE_RES(0x87, RESA, 0, 8);

MAKE_RES(0x88, RESB, 1, 8);
MAKE_RES(0x89, RESC, 1, 8);
MAKE_RES(0x8A, RESD, 1, 8);
MAKE_RES(0x8B, RESE, 1, 8);
MAKE_RES(0x8C, RESH, 1, 8);
MAKE_RES(0x8D, RESL, 1, 8);
MAKE_RES(0x8E, RESHLv, 1, 16);
MAKE_RES(0x8F, RESA, 1, 8);

MAKE_RES(0x90, RESB, 2, 8);
MAKE_RES(0x91, RESC, 2, 8);
MAKE_RES(0x92, RESD, 2, 8);
MAKE_RES(0x93, RESE, 2, 8);
MAKE_RES(0x94, RESH, 2, 8);
MAKE_RES(0x95, RESL, 2, 8);
MAKE_RES(0x96, RESHLv, 2, 16);
MAKE_RES(0x97, RESA, 2, 8);

MAKE_RES(0x98, RESB, 3, 8);
MAKE_RES(0x99, RESC, 3, 8);
MAKE_RES(0x9A, RESD, 3, 8);
MAKE_RES(0x9B, RESE, 3, 8);
MAKE_RES(0x9C, RESH, 3, 8);
MAKE_RES(0x9D, RESL, 3, 8);
MAKE_RES(0x9E, RESHLv, 3, 16);
MAKE_RES(0x9F, RESA, 3, 8);

MAKE_RES(0xA0, RESB, 4, 8);
MAKE_RES(0xA1, RESC, 4, 8);
MAKE_RES(0xA2, RESD, 4, 8);
MAKE_RES(0xA3, RESE, 4, 8);
MAKE_RES(0xA4, RESH, 4, 8);
MAKE_RES(0xA5, RESL, 4, 8);
MAKE_RES(0xA6, RESHLv, 4, 16);
MAKE_RES(0xA7, RESA, 4, 8);

MAKE_RES(0xA8, RESB, 5, 8);
MAKE_RES(0xA9, RESC, 5, 8);
MAKE_RES(0xAA, RESD, 5, 8);
MAKE_RES(0xAB, RESE, 5, 8);
MAKE_RES(0xAC, RESH, 5, 8);
MAKE_RES(0xAD, RESL, 5, 8);
MAKE_RES(0xAE, RESHLv, 5, 16);
MAKE_RES(0xAF, RESA, 5, 8);

MAKE_RES(0xB0, RESB, 6, 8);
MAKE_RES(0xB1, RESC, 6, 8);
MAKE_RES(0xB2, RESD, 6, 8);
MAKE_RES(0xB3, RESE, 6, 8);
MAKE_RES(0xB4, RESH, 6, 8);
MAKE_RES(0xB5, RESL, 6, 8);
MAKE_RES(0xB6, RESHLv, 6, 16);
MAKE_RES(0xB7, RESA, 6, 8);

MAKE_RES(0xB8, RESB, 7, 8);
MAKE_RES(0xB9, RESC, 7, 8);
MAKE_RES(0xBA, RESD, 7, 8);
MAKE_RES(0xBB, RESE, 7, 8);
MAKE_RES(0xBC, RESH, 7, 8);
MAKE_RES(0xBD, RESL, 7, 8);
MAKE_RES(0xBE, RESHLv, 7, 16);
MAKE_RES(0xBF, RESA, 7, 8);

#define MAKE_SET(opcode, clname, bit, cycles) class clname##bit final : public Instruction {\
public:\
	explicit clname##bit(Emulator * emulator_) :\
		Instruction(emulator_, (opcode), 2, (cycles), #clname""#bit) {\
	}\
\
	virtual void executeImpl() override;\
}

MAKE_SET(0xC0, SETB, 0, 8);
MAKE_SET(0xC1, SETC, 0, 8);
MAKE_SET(0xC2, SETD, 0, 8);
MAKE_SET(0xC3, SETE, 0, 8);
MAKE_SET(0xC4, SETH, 0, 8);
MAKE_SET(0xC5, SETL, 0, 8);
MAKE_SET(0xC6, SETHLv, 0, 16);
MAKE_SET(0xC7, SETA, 0, 8);

MAKE_SET(0xC8, SETB, 1, 8);
MAKE_SET(0xC9, SETC, 1, 8);
MAKE_SET(0xCA, SETD, 1, 8);
MAKE_SET(0xCB, SETE, 1, 8);
MAKE_SET(0xCC, SETH, 1, 8);
MAKE_SET(0xCD, SETL, 1, 8);
MAKE_SET(0xCE, SETHLv, 1, 16);
MAKE_SET(0xCF, SETA, 1, 8);

MAKE_SET(0xD0, SETB, 2, 8);
MAKE_SET(0xD1, SETC, 2, 8);
MAKE_SET(0xD2, SETD, 2, 8);
MAKE_SET(0xD3, SETE, 2, 8);
MAKE_SET(0xD4, SETH, 2, 8);
MAKE_SET(0xD5, SETL, 2, 8);
MAKE_SET(0xD6, SETHLv, 2, 16);
MAKE_SET(0xD7, SETA, 2, 8);

MAKE_SET(0xD8, SETB, 3, 8);
MAKE_SET(0xD9, SETC, 3, 8);
MAKE_SET(0xDA, SETD, 3, 8);
MAKE_SET(0xDB, SETE, 3, 8);
MAKE_SET(0xDC, SETH, 3, 8);
MAKE_SET(0xDD, SETL, 3, 8);
MAKE_SET(0xDE, SETHLv, 3, 16);
MAKE_SET(0xDF, SETA, 3, 8);

MAKE_SET(0xE0, SETB, 4, 8);
MAKE_SET(0xE1, SETC, 4, 8);
MAKE_SET(0xE2, SETD, 4, 8);
MAKE_SET(0xE3, SETE, 4, 8);
MAKE_SET(0xE4, SETH, 4, 8);
MAKE_SET(0xE5, SETL, 4, 8);
MAKE_SET(0xE6, SETHLv, 4, 16);
MAKE_SET(0xE7, SETA, 4, 8);

MAKE_SET(0xE8, SETB, 5, 8);
MAKE_SET(0xE9, SETC, 5, 8);
MAKE_SET(0xEA, SETD, 5, 8);
MAKE_SET(0xEB, SETE, 5, 8);
MAKE_SET(0xEC, SETH, 5, 8);
MAKE_SET(0xED, SETL, 5, 8);
MAKE_SET(0xEE, SETHLv, 5, 16);
MAKE_SET(0xEF, SETA, 5, 8);

MAKE_SET(0xF0, SETB, 6, 8);
MAKE_SET(0xF1, SETC, 6, 8);
MAKE_SET(0xF2, SETD, 6, 8);
MAKE_SET(0xF3, SETE, 6, 8);
MAKE_SET(0xF4, SETH, 6, 8);
MAKE_SET(0xF5, SETL, 6, 8);
MAKE_SET(0xF6, SETHLv, 6, 16);
MAKE_SET(0xF7, SETA, 6, 8);

MAKE_SET(0xF8, SETB, 7, 8);
MAKE_SET(0xF9, SETC, 7, 8);
MAKE_SET(0xFA, SETD, 7, 8);
MAKE_SET(0xFB, SETE, 7, 8);
MAKE_SET(0xFC, SETH, 7, 8);
MAKE_SET(0xFD, SETL, 7, 8);
MAKE_SET(0xFE, SETHLv, 7, 16);
MAKE_SET(0xFF, SETA, 7, 8);

}

#endif /* CODE_TRUNK_INCLUDE_INSTRUCTIONS_HPP_ */

