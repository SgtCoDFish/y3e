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

#ifndef CODE_TRUNK_INCLUDE_INSTRUCTIONPARSER_HPP_
#define CODE_TRUNK_INCLUDE_INSTRUCTIONPARSER_HPP_

#include <cstdint>

#include <memory>
#include <fstream>
#include <array>
#include <iostream>
#include <iomanip>
#include <vector>

#include "Instructions.hpp"

namespace y3e {
class Emulator;

/**
 * Allows a source of input (usually some sort of list of uint8_t bytes) to be parsed and the Z80
 * instructions the bytes represent to be shown.
 *
 * Should be subclassed to implement the actual parsing. Several implementations are provided by default
 * by Y3E.
 *
 * By default, instructions are cached where possible for performance reasons, which means that instructions
 * returned are not guaranteed to be valid after a second call to parse.
 */
class InstructionParser {
public:
	explicit InstructionParser(Emulator * emulator_);
	virtual ~InstructionParser() = default;

	/**
	 * Parse an instruction from the source given by a subclass.
	 * Returns a pointer to the instruction read, or nullptr if no instruction could be found.
	 *
	 * The parsed instruction is cached. It is not guaranteed to be valid after the next call to parse.
	 */
	Instruction *parse();

protected:
	std::unique_ptr<Instruction> makeInstruction(const uint8_t &opcode, bool parseArgs = true, bool failHard = true);
	std::unique_ptr<Instruction> makeInstructionCB(const uint8_t &opcode, bool failHard = true);

	Instruction *handleCB();

	std::array<std::unique_ptr<Instruction>, 256> instructionCache;
	std::array<std::unique_ptr<Instruction>, 256> cbInstructionCache;

	// holds instructions which cannot be cached until they're next parsed with new arguments
	std::array<std::unique_ptr<Instruction>, 256> uncachableInstructions;

	virtual uint8_t parseOpcode() = 0;

	virtual uint8_t parseInstructionArgument8() = 0;
	virtual int8_t parseInstructionArgument8Signed() = 0;

	virtual uint16_t parseInstructionArgument16() = 0;

	Emulator * const emulator;

private:
	void initialiseInstructionCache();
};

/**
 * Read from an input stream to determine what instruction is present at the given point in the stream
 * and then read further bytes as is neccessary depending on the length of the instruction.
 *
 * After each call to parse(), leaves stream at the first byte after the instruction that was read.
 *
 * This parser is likely to be quite slow given that disk reads must occur after every call to parse.
 * It might be advisable to pre-load the whole file and then use, for example, a VectorInstructionParser
 * or an ArrayInstructionParser.
 */
class FstreamInstructionParser final : public InstructionParser {
public:
	explicit FstreamInstructionParser(Emulator * emulator_, std::ifstream &stream_);
	virtual ~FstreamInstructionParser() = default;

protected:
	virtual uint8_t parseOpcode() override;

	virtual uint8_t parseInstructionArgument8() override;
	virtual int8_t parseInstructionArgument8Signed() override;

	virtual uint16_t parseInstructionArgument16() override;

private:
	std::ifstream &stream;
};

/**
 * Read from a vector of bytes using a const_iterator to determine the instruction, and read further
 * as required to create the complete instruction.
 *
 * After each call to parse() the iterator points to the byte after the last instruction that was read.
 */
class VectorInstructionParser final : public InstructionParser {
public:
	explicit VectorInstructionParser(Emulator * emulator_, std::vector<uint8_t>::const_iterator &it_);
	virtual ~VectorInstructionParser() = default;

protected:
	virtual uint8_t parseOpcode() override;

	virtual uint8_t parseInstructionArgument8() override;
	virtual int8_t parseInstructionArgument8Signed() override;

	virtual uint16_t parseInstructionArgument16() override;

private:
	std::vector<uint8_t>::const_iterator &it;
};

/**
 * Read from an array of bytes using a pointer to determine the instruction, and read further
 * as required to create the complete instruction.
 *
 * After each call to parse() the pointer points to the byte after the last instruction that was read.
 *
 * WARNING: No bounds checking is performed by this pointer.
 */
class ArrayInstructionParser final : public InstructionParser {
public:
	explicit ArrayInstructionParser(Emulator * emulator_, uint8_t *it_);
	virtual ~ArrayInstructionParser() = default;

	void setPos(uint8_t *it_);

protected:
	virtual uint8_t parseOpcode() override;

	virtual uint8_t parseInstructionArgument8() override;
	virtual int8_t parseInstructionArgument8Signed() override;

	virtual uint16_t parseInstructionArgument16() override;

private:
	uint8_t * it;
};

}

#endif /* CODE_TRUNK_INCLUDE_INSTRUCTIONPARSER_HPP_ */
