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
#include "SerialHandler.hpp"

namespace y3e {

SerialHandler::SerialHandler(Emulator * emulator_) :
		        emulator { emulator_ } {

}

void SerialHandler::handleMemoryWrite(const uint16_t &address, const uint8_t &value) {
	if (address < 0xFF01 || address > 0xFF02) {
		return;
	}

	if (address == 0xFF01) {
		// ignore a write to the transfer buffer for now.
		// TODO: Implement handling
		return;
	} else if (address == 0xFF02) {
		if (value & 0x80) { // if bit 7 is set
			// start transfer

			// set bit 7 to 0 to indicate no transfer is possible
			emulator->memory.write(0xFF02, value & 0x7F);

			// write FF to the SB register to indicate failure
			emulator->memory.write(0xFF01, 0xFF);
//			emulator->interruptHandler.triggerInterrupt(InterruptType::SERIAL_IO_COMPLETE);
		}
	}
}

void SerialHandler::update(const int64_t &clocks) {
	clockCounter += clocks;

	if (clockCounter >= 8) {
		clockCounter -= 8;
	}
}

}
