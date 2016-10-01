
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

#include <memory>

#include "InstructionParser.hpp"
#include "Emulator.hpp"
#include "Registers.hpp"

#define HANDLE_STANDARD(opcode, clname) case (opcode): {\
	ret = std::make_unique<clname>(emulator);\
	break;\
}

#define HANDLE_STANDARD_REG(opcode, clname, __ignore) HANDLE_STANDARD(opcode, clname)

#define HANDLE_ARG8U(opcode, clname) case (opcode): {\
    if(!parseArgs) {\
        ret = nullptr;\
        break;\
	}\
\
	const auto value = parseInstructionArgument8();\
	ret = std::make_unique<clname>(emulator, value);\
	break;\
}

#define HANDLE_ARG8S(opcode, clname) case (opcode): {\
    if(!parseArgs) {\
        ret = nullptr;\
        break;\
	}\
\
	const auto value = parseInstructionArgument8Signed();\
	ret = std::make_unique<clname>(emulator, value);\
	break;\
}

#define HANDLE_ARG16U(opcode, clname) case (opcode): {\
    if(!parseArgs) {\
        ret = nullptr;\
        break;\
	}\
\
	const auto value = parseInstructionArgument16();\
	ret = std::make_unique<clname>(emulator, value);\
	break;\
}

namespace y3e {

InstructionParser::InstructionParser(Emulator * emulator_) :
	emulator{ emulator_ } {
	initialiseInstructionCache();
}

Instruction * InstructionParser::parse() {
	std::unique_ptr<Instruction> ret;

	const uint8_t opcode = parseOpcode();

	if (opcode == 0xCB) {
		return handleCB();
	}

	if (instructionCache[opcode] == nullptr) {
		uncachableInstructions[opcode] = makeInstruction(opcode);
		return uncachableInstructions[opcode].get();
	}

	return instructionCache[opcode].get();
}

Instruction * InstructionParser::handleCB() {
	std::unique_ptr<Instruction> ret;
	const uint8_t cbOpcode = parseOpcode();

	// assuming caching enabled
	cbInstructionCache[cbOpcode] = makeInstructionCB(cbOpcode, true);

	return cbInstructionCache[cbOpcode].get();
}

std::unique_ptr<Instruction> InstructionParser::makeInstruction(const uint8_t &opcode, bool parseArgs, bool failHard) {
	std::unique_ptr<Instruction> ret;

	switch (opcode) {
		HANDLE_STANDARD(0x00, NOP)
			HANDLE_ARG16U(0x01, LDBCd16)
			HANDLE_STANDARD(0x02, LDBCvA)
			HANDLE_STANDARD(0x03, INCBC)
			HANDLE_STANDARD(0x04, INCB)
			HANDLE_STANDARD(0x05, DECB)
			HANDLE_ARG8U(0x06, LDBd8)
			HANDLE_STANDARD(0x07, RLCA)
			HANDLE_ARG16U(0x08, LDa16vSP)
			HANDLE_STANDARD(0x09, ADDHLBC)
			HANDLE_STANDARD(0x0A, LDABCv)
			HANDLE_STANDARD(0x0B, DECBC)
			HANDLE_STANDARD(0x0C, INCC)

			HANDLE_STANDARD(0x0D, DECC)
			HANDLE_ARG8U(0x0E, LDCd8)
			HANDLE_STANDARD(0x0F, RRCA)

			HANDLE_STANDARD(0x10, STOP)
			HANDLE_ARG16U(0x11, LDDEd16)
			HANDLE_STANDARD(0x12, LDDEvA)
			HANDLE_STANDARD(0x13, INCDE)
			HANDLE_STANDARD(0x14, INCD)
			HANDLE_STANDARD(0x15, DECD)
			HANDLE_ARG8U(0x16, LDDd8)
			HANDLE_STANDARD(0x17, RLA)
			HANDLE_ARG8S(0x18, JRr8)
			HANDLE_STANDARD(0x19, ADDHLDE)
			HANDLE_STANDARD(0x1A, LDADEv)
			HANDLE_STANDARD(0x1B, DECDE)
			HANDLE_STANDARD(0x1C, INCE)
			HANDLE_STANDARD(0x1D, DECE)
			HANDLE_ARG8U(0x1E, LDEd8)
			HANDLE_STANDARD(0x1F, RRA)

			HANDLE_ARG8S(0x20, JRNZr8)
			HANDLE_ARG16U(0x21, LDHLd16)
			HANDLE_STANDARD(0x22, LDIHLvA)
			HANDLE_STANDARD(0x23, INCHL)
			HANDLE_STANDARD(0x24, INCH)
			HANDLE_STANDARD(0x25, DECH)
			HANDLE_ARG8U(0x26, LDHd8)
			HANDLE_STANDARD(0x27, DAA)
			HANDLE_ARG8S(0x28, JRZr8)
			HANDLE_STANDARD(0x29, ADDHLHL)
			HANDLE_STANDARD(0x2A, LDIAHL)
			HANDLE_STANDARD(0x2B, DECHL)
			HANDLE_STANDARD(0x2C, INCL)
			HANDLE_STANDARD(0x2D, DECL)
			HANDLE_ARG8U(0x2E, LDLd8)
			HANDLE_STANDARD(0x2F, CPL)

			HANDLE_ARG8S(0x30, JRNCr8)
			HANDLE_ARG16U(0x31, LDSPd16)
			HANDLE_STANDARD(0x32, LDDHLA)
			HANDLE_STANDARD(0x33, INCSP)
			HANDLE_STANDARD(0x34, INCHLv)
			HANDLE_STANDARD(0x35, DECHLv)
			HANDLE_ARG8U(0x36, LDHLd8)
			HANDLE_STANDARD(0x37, SCF)
			HANDLE_ARG8S(0x38, JRCr8)
			HANDLE_STANDARD(0x39, ADDHLSP)
			HANDLE_STANDARD(0x3A, LDDAHLv)
			HANDLE_STANDARD(0x3B, DECSP)
			HANDLE_STANDARD(0x3C, INCA)
			HANDLE_STANDARD(0x3D, DECA)
			HANDLE_ARG8U(0x3E, LDAd8)
			HANDLE_STANDARD(0x3F, CCF)

			HANDLE_STANDARD(0x40, LDBB)
			HANDLE_STANDARD(0x41, LDBC)
			HANDLE_STANDARD(0x42, LDBD)
			HANDLE_STANDARD(0x43, LDBE)
			HANDLE_STANDARD(0x44, LDBH)
			HANDLE_STANDARD(0x45, LDBL)
			HANDLE_STANDARD(0x46, LDBHLv)
			HANDLE_STANDARD(0x47, LDBA)

			HANDLE_STANDARD(0x48, LDCB)
			HANDLE_STANDARD(0x49, LDCC)
			HANDLE_STANDARD(0x4A, LDCD)
			HANDLE_STANDARD(0x4B, LDCE)
			HANDLE_STANDARD(0x4C, LDCH)
			HANDLE_STANDARD(0x4D, LDCL)
			HANDLE_STANDARD(0x4E, LDCHLv)
			HANDLE_STANDARD(0x4F, LDCA)

			HANDLE_STANDARD(0x50, LDDB)
			HANDLE_STANDARD(0x51, LDDC)
			HANDLE_STANDARD(0x52, LDDD)
			HANDLE_STANDARD(0x53, LDDE)
			HANDLE_STANDARD(0x54, LDDH)
			HANDLE_STANDARD(0x55, LDDL)
			HANDLE_STANDARD(0x56, LDDHLv)
			HANDLE_STANDARD(0x57, LDDA)

			HANDLE_STANDARD(0x58, LDEB)
			HANDLE_STANDARD(0x59, LDEC)
			HANDLE_STANDARD(0x5A, LDED)
			HANDLE_STANDARD(0x5B, LDEE)
			HANDLE_STANDARD(0x5C, LDEH)
			HANDLE_STANDARD(0x5D, LDEL)
			HANDLE_STANDARD(0x5E, LDEHLv)
			HANDLE_STANDARD(0x5F, LDEA)

			HANDLE_STANDARD(0x60, LDHB)
			HANDLE_STANDARD(0x61, LDHC)
			HANDLE_STANDARD(0x62, LDHD)
			HANDLE_STANDARD(0x63, LDHE)
			HANDLE_STANDARD(0x64, LDHH)
			HANDLE_STANDARD(0x65, LDHL)
			HANDLE_STANDARD(0x66, LDHHLv)
			HANDLE_STANDARD(0x67, LDHA)

			HANDLE_STANDARD(0x68, LDLB)
			HANDLE_STANDARD(0x69, LDLC)
			HANDLE_STANDARD(0x6A, LDLD)
			HANDLE_STANDARD(0x6B, LDLE)
			HANDLE_STANDARD(0x6C, LDLH)
			HANDLE_STANDARD(0x6D, LDLL)
			HANDLE_STANDARD(0x6E, LDLHLv)
			HANDLE_STANDARD(0x6F, LDLA)

			HANDLE_STANDARD(0x70, LDHLvB)
			HANDLE_STANDARD(0x71, LDHLvC)
			HANDLE_STANDARD(0x72, LDHLvD)
			HANDLE_STANDARD(0x73, LDHLvE)
			HANDLE_STANDARD(0x74, LDHLvH)
			HANDLE_STANDARD(0x75, LDHLvL)
			HANDLE_STANDARD(0x76, HALT)
			HANDLE_STANDARD(0x77, LDHLvA)

			HANDLE_STANDARD(0x78, LDAB)
			HANDLE_STANDARD(0x79, LDAC)
			HANDLE_STANDARD(0x7A, LDAD)
			HANDLE_STANDARD(0x7B, LDAE)
			HANDLE_STANDARD(0x7C, LDAH)
			HANDLE_STANDARD(0x7D, LDAL)
			HANDLE_STANDARD(0x7E, LDAHLv)
			HANDLE_STANDARD(0x7F, LDAA)

			HANDLE_STANDARD(0x80, ADDAB)
			HANDLE_STANDARD(0x81, ADDAC)
			HANDLE_STANDARD(0x82, ADDAD)
			HANDLE_STANDARD(0x83, ADDAE)
			HANDLE_STANDARD(0x84, ADDAH)
			HANDLE_STANDARD(0x85, ADDAL)
			HANDLE_STANDARD(0x86, ADDAHLv)
			HANDLE_STANDARD(0x87, ADDAA)
			HANDLE_STANDARD(0x88, ADCAB)
			HANDLE_STANDARD(0x89, ADCAC)
			HANDLE_STANDARD(0x8A, ADCAD)
			HANDLE_STANDARD(0x8B, ADCAE)
			HANDLE_STANDARD(0x8C, ADCAH)
			HANDLE_STANDARD(0x8D, ADCAL)
			HANDLE_STANDARD(0x8E, ADCAHLv)
			HANDLE_STANDARD(0x8F, ADCAA)

			HANDLE_STANDARD(0x90, SUBB)
			HANDLE_STANDARD(0x91, SUBC)
			HANDLE_STANDARD(0x92, SUBD)
			HANDLE_STANDARD(0x93, SUBE)
			HANDLE_STANDARD(0x94, SUBH)
			HANDLE_STANDARD(0x95, SUBL)
			HANDLE_STANDARD(0x96, SUBHLv)
			HANDLE_STANDARD(0x97, SUBA)

			HANDLE_STANDARD(0x98, SBCAB)
			HANDLE_STANDARD(0x99, SBCAC)
			HANDLE_STANDARD(0x9A, SBCAD)
			HANDLE_STANDARD(0x9B, SBCAE)
			HANDLE_STANDARD(0x9C, SBCAH)
			HANDLE_STANDARD(0x9D, SBCAL)
			HANDLE_STANDARD(0x9E, SBCAHLv)
			HANDLE_STANDARD(0x9F, SBCAA)

			HANDLE_STANDARD(0xA0, ANDB)
			HANDLE_STANDARD(0xA1, ANDC)
			HANDLE_STANDARD(0xA2, ANDD)
			HANDLE_STANDARD(0xA3, ANDE)
			HANDLE_STANDARD(0xA4, ANDH)
			HANDLE_STANDARD(0xA5, ANDL)
			HANDLE_STANDARD(0xA6, ANDHLv)
			HANDLE_STANDARD(0xA7, ANDA)

			HANDLE_STANDARD(0xA8, XORB)
			HANDLE_STANDARD(0xA9, XORC)
			HANDLE_STANDARD(0xAA, XORD)
			HANDLE_STANDARD(0xAB, XORE)
			HANDLE_STANDARD(0xAC, XORH)
			HANDLE_STANDARD(0xAD, XORL)
			HANDLE_STANDARD(0xAE, XORHLv)
			HANDLE_STANDARD(0xAF, XORA)

			HANDLE_STANDARD(0xB0, ORB)
			HANDLE_STANDARD(0xB1, ORC)
			HANDLE_STANDARD(0xB2, ORD)
			HANDLE_STANDARD(0xB3, ORE)
			HANDLE_STANDARD(0xB4, ORH)
			HANDLE_STANDARD(0xB5, ORL)
			HANDLE_STANDARD(0xB6, ORHLv)
			HANDLE_STANDARD(0xB7, ORA)

			HANDLE_STANDARD(0xB8, CPB)
			HANDLE_STANDARD(0xB9, CPC)
			HANDLE_STANDARD(0xBA, CPD)
			HANDLE_STANDARD(0xBB, CPE)
			HANDLE_STANDARD(0xBC, CPH)
			// Weird name to not clash with CPL instruction
			HANDLE_STANDARD(0xBD, CPwithL)
			HANDLE_STANDARD(0xBE, CPHLv)
			HANDLE_STANDARD(0xBF, CPA)

			HANDLE_STANDARD(0xC0, RETNZ)
			HANDLE_STANDARD(0xC1, POPBC)
			HANDLE_ARG16U(0xC2, JPNZa16)
			HANDLE_ARG16U(0xC3, JP16)
			HANDLE_ARG16U(0xC4, CALLNZa16)
			HANDLE_STANDARD(0xC5, PUSHBC)
			HANDLE_ARG8U(0xC6, ADDAd8)
			HANDLE_STANDARD(0xC7, RST00)
			HANDLE_STANDARD(0xC8, RETZ)
			HANDLE_STANDARD(0xC9, RET)
			HANDLE_ARG16U(0xCA, JPZa16)

			HANDLE_ARG16U(0xCC, CALLZa16)
			HANDLE_ARG16U(0xCD, CALLa16)
			HANDLE_ARG8U(0xCE, ADCAd8)
			HANDLE_STANDARD(0xCF, RST08)

			HANDLE_STANDARD(0xD0, RETNC)
			HANDLE_STANDARD(0xD1, POPDE)
			HANDLE_ARG16U(0xD2, JPNCa16)

			HANDLE_ARG16U(0xD4, CALLNCa16)
			HANDLE_STANDARD(0xD5, PUSHDE)
			HANDLE_ARG8U(0xD6, SUBd8)
			HANDLE_STANDARD(0xD7, RST10)
			HANDLE_STANDARD(0xD8, RETC)
			HANDLE_STANDARD(0xD9, RETI)
			HANDLE_ARG16U(0xDA, JPCa16)

			HANDLE_ARG16U(0xDC, CALLCa16)

			HANDLE_ARG8U(0xDE, SBCAd8)
			HANDLE_STANDARD(0xDF, RST18)

			HANDLE_ARG8U(0xE0, LDHa8A)
			HANDLE_STANDARD(0xE1, POPHL)
			HANDLE_STANDARD(0xE2, LDCVA)

			HANDLE_STANDARD(0xE5, PUSHHL)
			HANDLE_ARG8U(0xE6, ANDd8)
			HANDLE_STANDARD(0xE7, RST20)
			HANDLE_ARG8S(0xE8, ADDSPr8)
			HANDLE_STANDARD(0xE9, JPHL)
			HANDLE_ARG16U(0xEA, LDa16A)

			HANDLE_ARG8U(0xEE, XORd8)
			HANDLE_STANDARD(0xEF, RST28)

			HANDLE_ARG8U(0xF0, LDHAa8)
			HANDLE_STANDARD(0xF1, POPAF)
			HANDLE_STANDARD(0xF2, LDACv)
			HANDLE_STANDARD(0xF3, DI)

			HANDLE_STANDARD(0xF5, PUSHAF)
			HANDLE_ARG8U(0xF6, ORd8)
			HANDLE_STANDARD(0xF7, RST30)
			HANDLE_ARG8S(0xF8, LDHLSPr8)
			HANDLE_STANDARD(0xF9, LDSPHL)

			HANDLE_ARG16U(0xFA, LDAa16v)

			HANDLE_STANDARD(0xFB, EI)

			HANDLE_ARG8U(0xFE, CPd8)
			HANDLE_STANDARD(0xFF, RST38)

	default:
		{
			if (failHard) {
				std::cout << "Unimplemented instruction: 0x" << std::hex << (static_cast<int>(opcode) & 0xff) << std::endl
					<< "Reg dump:\n" << emulator->reg << std::endl;
				std::abort();
			}

			ret = nullptr;
		}
	}

	return ret;
}

std::unique_ptr<Instruction> InstructionParser::makeInstructionCB(const uint8_t &cbOpcode, bool failHard) {
	std::unique_ptr<Instruction> ret;

	switch (cbOpcode) {
		HANDLE_STANDARD(0x00, RLCB)
			HANDLE_STANDARD(0x01, RLCC)
			HANDLE_STANDARD(0x02, RLCD)
			HANDLE_STANDARD(0x03, RLCE)
			HANDLE_STANDARD(0x04, RLCH)
			HANDLE_STANDARD(0x05, RLCL)
			HANDLE_STANDARD(0x06, RLCHLv)
			HANDLE_STANDARD(0x07, RLCACB)

			HANDLE_STANDARD(0x08, RRCB)
			HANDLE_STANDARD(0x09, RRCC)
			HANDLE_STANDARD(0x0A, RRCD)
			HANDLE_STANDARD(0x0B, RRCE)
			HANDLE_STANDARD(0x0C, RRCH)
			HANDLE_STANDARD(0x0D, RRCL)
			HANDLE_STANDARD(0x0E, RRCHLv)
			HANDLE_STANDARD(0x0F, RRCACB)

			HANDLE_STANDARD(0x10, RLB)
			HANDLE_STANDARD(0x11, RLC)
			HANDLE_STANDARD(0x12, RLD)
			HANDLE_STANDARD(0x13, RLE)
			HANDLE_STANDARD(0x14, RLH)
			HANDLE_STANDARD(0x15, RLL)
			HANDLE_STANDARD(0x16, RLHLv)
			HANDLE_STANDARD(0x17, RLACB)

			HANDLE_STANDARD(0x18, RRB)
			HANDLE_STANDARD(0x19, RRC)
			HANDLE_STANDARD(0x1A, RRD)
			HANDLE_STANDARD(0x1B, RRE)
			HANDLE_STANDARD(0x1C, RRH)
			HANDLE_STANDARD(0x1D, RRL)
			HANDLE_STANDARD(0x1E, RRHLv)
			HANDLE_STANDARD(0x1F, RRACB)

			HANDLE_STANDARD(0x20, SLAB)
			HANDLE_STANDARD(0x21, SLAC)
			HANDLE_STANDARD(0x22, SLAD)
			HANDLE_STANDARD(0x23, SLAE)
			HANDLE_STANDARD(0x24, SLAH)
			HANDLE_STANDARD(0x25, SLAL)
			HANDLE_STANDARD(0x26, SLAHLv)
			HANDLE_STANDARD(0x27, SLAA)

			HANDLE_STANDARD(0x28, SRAB)
			HANDLE_STANDARD(0x29, SRAC)
			HANDLE_STANDARD(0x2A, SRAD)
			HANDLE_STANDARD(0x2B, SRAE)
			HANDLE_STANDARD(0x2C, SRAH)
			HANDLE_STANDARD(0x2D, SRAL)
			HANDLE_STANDARD(0x2E, SRAHLv)
			HANDLE_STANDARD(0x2F, SRAA)

			HANDLE_STANDARD(0x30, SWAPB)
			HANDLE_STANDARD(0x31, SWAPC)
			HANDLE_STANDARD(0x32, SWAPD)
			HANDLE_STANDARD(0x33, SWAPE)
			HANDLE_STANDARD(0x34, SWAPH)
			HANDLE_STANDARD(0x35, SWAPL)
			HANDLE_STANDARD(0x36, SWAPHLv)
			HANDLE_STANDARD(0x37, SWAPA)

			HANDLE_STANDARD(0x38, SRLB)
			HANDLE_STANDARD(0x39, SRLC)
			HANDLE_STANDARD(0x3A, SRLD)
			HANDLE_STANDARD(0x3B, SRLE)
			HANDLE_STANDARD(0x3C, SRLH)
			HANDLE_STANDARD(0x3D, SRLL)
			HANDLE_STANDARD(0x3E, SRLHLv)
			HANDLE_STANDARD(0x3F, SRLA)

			HANDLE_STANDARD_REG(0x40, BIT0B, 8)
			HANDLE_STANDARD_REG(0x41, BIT0C, 8)
			HANDLE_STANDARD_REG(0x42, BIT0D, 8)
			HANDLE_STANDARD_REG(0x43, BIT0E, 8)
			HANDLE_STANDARD_REG(0x44, BIT0H, 8)
			HANDLE_STANDARD_REG(0x45, BIT0L, 8)
			HANDLE_STANDARD_REG(0x46, BIT0HLv, 16)
			HANDLE_STANDARD_REG(0x47, BIT0A, 8)

			HANDLE_STANDARD_REG(0x48, BIT1B, 8)
			HANDLE_STANDARD_REG(0x49, BIT1C, 8)
			HANDLE_STANDARD_REG(0x4A, BIT1D, 8)
			HANDLE_STANDARD_REG(0x4B, BIT1E, 8)
			HANDLE_STANDARD_REG(0x4C, BIT1H, 8)
			HANDLE_STANDARD_REG(0x4D, BIT1L, 8)
			HANDLE_STANDARD_REG(0x4E, BIT1HLv, 16)
			HANDLE_STANDARD_REG(0x4F, BIT1A, 8)

			HANDLE_STANDARD_REG(0x50, BIT2B, 8)
			HANDLE_STANDARD_REG(0x51, BIT2C, 8)
			HANDLE_STANDARD_REG(0x52, BIT2D, 8)
			HANDLE_STANDARD_REG(0x53, BIT2E, 8)
			HANDLE_STANDARD_REG(0x54, BIT2H, 8)
			HANDLE_STANDARD_REG(0x55, BIT2L, 8)
			HANDLE_STANDARD_REG(0x56, BIT2HLv, 16)
			HANDLE_STANDARD_REG(0x57, BIT2A, 8)

			HANDLE_STANDARD_REG(0x58, BIT3B, 8)
			HANDLE_STANDARD_REG(0x59, BIT3C, 8)
			HANDLE_STANDARD_REG(0x5A, BIT3D, 8)
			HANDLE_STANDARD_REG(0x5B, BIT3E, 8)
			HANDLE_STANDARD_REG(0x5C, BIT3H, 8)
			HANDLE_STANDARD_REG(0x5D, BIT3L, 8)
			HANDLE_STANDARD_REG(0x5E, BIT3HLv, 16)
			HANDLE_STANDARD_REG(0x5F, BIT3A, 8)

			HANDLE_STANDARD_REG(0x60, BIT4B, 8)
			HANDLE_STANDARD_REG(0x61, BIT4C, 8)
			HANDLE_STANDARD_REG(0x62, BIT4D, 8)
			HANDLE_STANDARD_REG(0x63, BIT4E, 8)
			HANDLE_STANDARD_REG(0x64, BIT4H, 8)
			HANDLE_STANDARD_REG(0x65, BIT4L, 8)
			HANDLE_STANDARD_REG(0x66, BIT4HLv, 16)
			HANDLE_STANDARD_REG(0x67, BIT4A, 8)
			HANDLE_STANDARD_REG(0x68, BIT5B, 8)
			HANDLE_STANDARD_REG(0x69, BIT5C, 8)
			HANDLE_STANDARD_REG(0x6A, BIT5D, 8)
			HANDLE_STANDARD_REG(0x6B, BIT5E, 8)
			HANDLE_STANDARD_REG(0x6C, BIT5H, 8)
			HANDLE_STANDARD_REG(0x6D, BIT5L, 8)
			HANDLE_STANDARD_REG(0x6E, BIT5HLv, 16)
			HANDLE_STANDARD_REG(0x6F, BIT5A, 8)

			HANDLE_STANDARD_REG(0x70, BIT6B, 8)
			HANDLE_STANDARD_REG(0x71, BIT6C, 8)
			HANDLE_STANDARD_REG(0x72, BIT6D, 8)
			HANDLE_STANDARD_REG(0x73, BIT6E, 8)
			HANDLE_STANDARD_REG(0x74, BIT6H, 8)
			HANDLE_STANDARD_REG(0x75, BIT6L, 8)
			HANDLE_STANDARD_REG(0x76, BIT6HLv, 16)
			HANDLE_STANDARD_REG(0x77, BIT6A, 8)
			HANDLE_STANDARD_REG(0x78, BIT7B, 8)
			HANDLE_STANDARD_REG(0x79, BIT7C, 8)
			HANDLE_STANDARD_REG(0x7A, BIT7D, 8)
			HANDLE_STANDARD_REG(0x7B, BIT7E, 8)
			HANDLE_STANDARD_REG(0x7C, BIT7H, 8)
			HANDLE_STANDARD_REG(0x7D, BIT7L, 8)
			HANDLE_STANDARD_REG(0x7E, BIT7HLv, 16)
			HANDLE_STANDARD_REG(0x7F, BIT7A, 8)

			HANDLE_STANDARD_REG(0x80, RESB0, b)
			HANDLE_STANDARD_REG(0x81, RESC0, c)
			HANDLE_STANDARD_REG(0x82, RESD0, d)
			HANDLE_STANDARD_REG(0x83, RESE0, e)
			HANDLE_STANDARD_REG(0x84, RESH0, h)
			HANDLE_STANDARD_REG(0x85, RESL0, l)
			HANDLE_STANDARD(0x86, RESHLv0)
			HANDLE_STANDARD_REG(0x87, RESA0, a)

			HANDLE_STANDARD_REG(0x88, RESB1, b)
			HANDLE_STANDARD_REG(0x89, RESC1, c)
			HANDLE_STANDARD_REG(0x8A, RESD1, d)
			HANDLE_STANDARD_REG(0x8B, RESE1, e)
			HANDLE_STANDARD_REG(0x8C, RESH1, h)
			HANDLE_STANDARD_REG(0x8D, RESL1, l)
			HANDLE_STANDARD(0x8E, RESHLv1)
			HANDLE_STANDARD_REG(0x8F, RESA1, a)

			HANDLE_STANDARD_REG(0x90, RESB2, b)
			HANDLE_STANDARD_REG(0x91, RESC2, c)
			HANDLE_STANDARD_REG(0x92, RESD2, d)
			HANDLE_STANDARD_REG(0x93, RESE2, e)
			HANDLE_STANDARD_REG(0x94, RESH2, h)
			HANDLE_STANDARD_REG(0x95, RESL2, l)
			HANDLE_STANDARD(0x96, RESHLv2)
			HANDLE_STANDARD_REG(0x97, RESA2, a)

			HANDLE_STANDARD_REG(0x98, RESB3, b)
			HANDLE_STANDARD_REG(0x99, RESC3, c)
			HANDLE_STANDARD_REG(0x9A, RESD3, d)
			HANDLE_STANDARD_REG(0x9B, RESE3, e)
			HANDLE_STANDARD_REG(0x9C, RESH3, h)
			HANDLE_STANDARD_REG(0x9D, RESL3, l)
			HANDLE_STANDARD(0x9E, RESHLv3)
			HANDLE_STANDARD_REG(0x9F, RESA3, a)

			HANDLE_STANDARD_REG(0xA0, RESB4, b)
			HANDLE_STANDARD_REG(0xA1, RESC4, c)
			HANDLE_STANDARD_REG(0xA2, RESD4, d)
			HANDLE_STANDARD_REG(0xA3, RESE4, e)
			HANDLE_STANDARD_REG(0xA4, RESH4, h)
			HANDLE_STANDARD_REG(0xA5, RESL4, l)
			HANDLE_STANDARD(0xA6, RESHLv4)
			HANDLE_STANDARD_REG(0xA7, RESA4, a)

			HANDLE_STANDARD_REG(0xA8, RESB5, b)
			HANDLE_STANDARD_REG(0xA9, RESC5, c)
			HANDLE_STANDARD_REG(0xAA, RESD5, d)
			HANDLE_STANDARD_REG(0xAB, RESE5, e)
			HANDLE_STANDARD_REG(0xAC, RESH5, h)
			HANDLE_STANDARD_REG(0xAD, RESL5, l)
			HANDLE_STANDARD(0xAE, RESHLv5)
			HANDLE_STANDARD_REG(0xAF, RESA5, a)

			HANDLE_STANDARD_REG(0xB0, RESB6, b)
			HANDLE_STANDARD_REG(0xB1, RESC6, c)
			HANDLE_STANDARD_REG(0xB2, RESD6, d)
			HANDLE_STANDARD_REG(0xB3, RESE6, e)
			HANDLE_STANDARD_REG(0xB4, RESH6, h)
			HANDLE_STANDARD_REG(0xB5, RESL6, l)
			HANDLE_STANDARD(0xB6, RESHLv6)
			HANDLE_STANDARD_REG(0xB7, RESA6, a)

			HANDLE_STANDARD_REG(0xB8, RESB7, b)
			HANDLE_STANDARD_REG(0xB9, RESC7, c)
			HANDLE_STANDARD_REG(0xBA, RESD7, d)
			HANDLE_STANDARD_REG(0xBB, RESE7, e)
			HANDLE_STANDARD_REG(0xBC, RESH7, h)
			HANDLE_STANDARD_REG(0xBD, RESL7, l)
			HANDLE_STANDARD(0xBE, RESHLv7)
			HANDLE_STANDARD_REG(0xBF, RESA7, a)

			HANDLE_STANDARD_REG(0xC0, SETB0, b)
			HANDLE_STANDARD_REG(0xC1, SETC0, c)
			HANDLE_STANDARD_REG(0xC2, SETD0, d)
			HANDLE_STANDARD_REG(0xC3, SETE0, e)
			HANDLE_STANDARD_REG(0xC4, SETH0, h)
			HANDLE_STANDARD_REG(0xC5, SETL0, l)
			HANDLE_STANDARD(0xC6, SETHLv0)
			HANDLE_STANDARD_REG(0xC7, SETA0, a)

			HANDLE_STANDARD_REG(0xC8, SETB1, b)
			HANDLE_STANDARD_REG(0xC9, SETC1, c)
			HANDLE_STANDARD_REG(0xCA, SETD1, d)
			HANDLE_STANDARD_REG(0xCB, SETE1, e)
			HANDLE_STANDARD_REG(0xCC, SETH1, h)
			HANDLE_STANDARD_REG(0xCD, SETL1, l)
			HANDLE_STANDARD(0xCE, SETHLv1)
			HANDLE_STANDARD_REG(0xCF, SETA1, a)

			HANDLE_STANDARD_REG(0xD0, SETB2, b)
			HANDLE_STANDARD_REG(0xD1, SETC2, c)
			HANDLE_STANDARD_REG(0xD2, SETD2, d)
			HANDLE_STANDARD_REG(0xD3, SETE2, e)
			HANDLE_STANDARD_REG(0xD4, SETH2, h)
			HANDLE_STANDARD_REG(0xD5, SETL2, l)
			HANDLE_STANDARD(0xD6, SETHLv2)
			HANDLE_STANDARD_REG(0xD7, SETA2, a)

			HANDLE_STANDARD_REG(0xD8, SETB3, b)
			HANDLE_STANDARD_REG(0xD9, SETC3, c)
			HANDLE_STANDARD_REG(0xDA, SETD3, d)
			HANDLE_STANDARD_REG(0xDB, SETE3, e)
			HANDLE_STANDARD_REG(0xDC, SETH3, h)
			HANDLE_STANDARD_REG(0xDD, SETL3, l)
			HANDLE_STANDARD(0xDE, SETHLv3)
			HANDLE_STANDARD_REG(0xDF, SETA3, a)

			HANDLE_STANDARD_REG(0xE0, SETB4, b)
			HANDLE_STANDARD_REG(0xE1, SETC4, c)
			HANDLE_STANDARD_REG(0xE2, SETD4, d)
			HANDLE_STANDARD_REG(0xE3, SETE4, e)
			HANDLE_STANDARD_REG(0xE4, SETH4, h)
			HANDLE_STANDARD_REG(0xE5, SETL4, l)
			HANDLE_STANDARD(0xE6, SETHLv4)
			HANDLE_STANDARD_REG(0xE7, SETA4, a)

			HANDLE_STANDARD_REG(0xE8, SETB5, b)
			HANDLE_STANDARD_REG(0xE9, SETC5, c)
			HANDLE_STANDARD_REG(0xEA, SETD5, d)
			HANDLE_STANDARD_REG(0xEB, SETE5, e)
			HANDLE_STANDARD_REG(0xEC, SETH5, h)
			HANDLE_STANDARD_REG(0xED, SETL5, l)
			HANDLE_STANDARD(0xEE, SETHLv5)
			HANDLE_STANDARD_REG(0xEF, SETA5, a)

			HANDLE_STANDARD_REG(0xF0, SETB6, b)
			HANDLE_STANDARD_REG(0xF1, SETC6, c)
			HANDLE_STANDARD_REG(0xF2, SETD6, d)
			HANDLE_STANDARD_REG(0xF3, SETE6, e)
			HANDLE_STANDARD_REG(0xF4, SETH6, h)
			HANDLE_STANDARD_REG(0xF5, SETL6, l)
			HANDLE_STANDARD(0xF6, SETHLv6)
			HANDLE_STANDARD_REG(0xF7, SETA6, a)

			HANDLE_STANDARD_REG(0xF8, SETB7, b)
			HANDLE_STANDARD_REG(0xF9, SETC7, c)
			HANDLE_STANDARD_REG(0xFA, SETD7, d)
			HANDLE_STANDARD_REG(0xFB, SETE7, e)
			HANDLE_STANDARD_REG(0xFC, SETH7, h)
			HANDLE_STANDARD_REG(0xFD, SETL7, l)
			HANDLE_STANDARD(0xFE, SETHLv7)
			HANDLE_STANDARD_REG(0xFF, SETA7, a)

	default:
		{
			if (failHard) {
				std::cout << "Unimplemented CB instruction: 0x" << std::hex << (static_cast<int>(cbOpcode) & 0xff) << std::endl
					<< "Reg dump:\n" << emulator->reg << std::endl;
				std::abort();
			}

			ret = nullptr;
		}
	}

	return ret;
}

void InstructionParser::initialiseInstructionCache() {
	for (uint16_t i = 0x00; i < 0x100; ++i) {
		instructionCache[i] = makeInstruction(i, false, false);
		cbInstructionCache[i] = makeInstructionCB(i, false);
	}

//	std::cout << "Generated instruction cache\n";
}

FstreamInstructionParser::FstreamInstructionParser(Emulator * emulator_, std::ifstream &stream_) :
	InstructionParser(emulator_),
	stream(stream_) {

}

uint8_t FstreamInstructionParser::parseOpcode() {
	return uint8_t(stream.get());
}

uint8_t FstreamInstructionParser::parseInstructionArgument8() {
	return uint8_t(stream.get());
}

int8_t FstreamInstructionParser::parseInstructionArgument8Signed() {
	return int8_t(stream.get());
}

uint16_t FstreamInstructionParser::parseInstructionArgument16() {
	const uint8_t byte1 = stream.get();
	const uint8_t byte2 = stream.get();

	const uint16_t ret = (byte2 << 8) + byte1;

	return ret;
}

VectorInstructionParser::VectorInstructionParser(Emulator * emulator_, std::vector<uint8_t>::const_iterator &it_) :
	InstructionParser(emulator_),
	it(it_) {

}

uint8_t VectorInstructionParser::parseOpcode() {
	return *it++; // return value and then increment
}

uint8_t VectorInstructionParser::parseInstructionArgument8() {
	return *it++;
}

int8_t VectorInstructionParser::parseInstructionArgument8Signed() {
	return int8_t(*it++);
}

uint16_t VectorInstructionParser::parseInstructionArgument16() {
	const uint8_t byte1 = *it++;
	const uint8_t byte2 = *it++;

	const uint16_t ret = (byte2 << 8) + byte1;

	return ret;
}

ArrayInstructionParser::ArrayInstructionParser(Emulator * emulator_, uint8_t *it_) :
	InstructionParser(emulator_),
	it(it_) {

}

uint8_t ArrayInstructionParser::parseOpcode() {
	return *it++; // return value and then increment
}

uint8_t ArrayInstructionParser::parseInstructionArgument8() {
	return *it++;
}

int8_t ArrayInstructionParser::parseInstructionArgument8Signed() {
	return int8_t(*it++);
}

uint16_t ArrayInstructionParser::parseInstructionArgument16() {
	const uint8_t byte1 = *it++;
	const uint8_t byte2 = *it++;

	const uint16_t ret = (byte2 << 8) + byte1;

	return ret;
}

void ArrayInstructionParser::setPos(uint8_t *it_) {
	it = it_;
}

}
