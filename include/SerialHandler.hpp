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

#ifndef CODE_TRUNK_INCLUDE_SERIALHANDLER_HPP_
#define CODE_TRUNK_INCLUDE_SERIALHANDLER_HPP_

#include <cstdint>

#include <stack>

namespace y3e {
class Emulator;

/**
 * SerialHandler implements an extensible method for handling the link-cable feature
 * of the Game Boy. This base class is a dummy implementation which will do nothing
 * except report to any software that the transfer has failed; it doesn't take into
 * account clock timing or any other consideration.
 *
 * An implementer may choose to subclass the SerialHandler to implement actual data-link
 * emulation (for example, over the internet).
 */
class SerialHandler {
public:
	explicit SerialHandler(Emulator * emulator_);
	virtual ~SerialHandler() = default;

	/**
	 * Should be called by the memory manager to indicate that serial-output relevant
	 * memory has been modified.
	 */
	virtual void handleMemoryWrite(const uint16_t &address, const uint8_t &value);

	/**
	 * Should be called every frame, to signal how much progress the transfer should have made.
	 *
	 * Clocks should be a multiple of 4.
	 */
	void update(const int64_t &clocks);

private:
	Emulator * emulator;

	int64_t clockCounter = 0;

	std::stack<uint8_t> writeStack;
};

}

#endif /* CODE_TRUNK_INCLUDE_SERIALHANDLER_HPP_ */
