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

#ifndef CODE_TRUNK_INCLUDE_STACKHANDLER_HPP_
#define CODE_TRUNK_INCLUDE_STACKHANDLER_HPP_

#include <cstdint>

namespace y3e {
class Emulator;

/**
 * Emulates the gameboy stack, which starts at the top of 16-bit addressable space
 * and grows downwards. This class makes it easier to debug the stack by keeping track
 * of pushed values.
 */
class StackHandler {
public:
	explicit StackHandler(Emulator * emulator_);
	~StackHandler() = default;

	// Stack depth, how many items are on the stack
	uint32_t stackDepth = 0;

	/**
	 * Push a value onto the stack. Will call the relevant memory manager functions.
	 */
	void push(uint16_t value);

	/**
	 * Pop a value from the stack, using the relevant memory manager functions.
	 */
	uint16_t pop();

private:
	Emulator * emulator;
};

}

#endif /* CODE_TRUNK_INCLUDE_STACKHANDLER_HPP_ */
