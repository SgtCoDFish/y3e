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

#include <iostream>

#include "Emulator.hpp"
#include "SDLAudioHandler.hpp"
#include "Y3EPlatformOptions.hpp"

namespace y3e {

SDLAudioHandler::SDLAudioHandler(Emulator * emulator_) :
		        AudioHandler(emulator_) {
	audioDebug();

//	apu.treble_eq(-20.0);
//	stereoBuffer.bass_freq(461);
//	apu.volume(0.5);

	apu.output(stereoBuffer.center(), stereoBuffer.left(), stereoBuffer.right());
	stereoBuffer.clock_rate(4194304);

	if (stereoBuffer.set_sample_rate(SDLAudioHandler::SAMPLE_RATE) != nullptr) {
		std::cout << "Couldn't set sample rate.\n";
		error_ = true;
		return;
	}

	if (queue.start(SDLAudioHandler::SAMPLE_RATE, 1) != nullptr) {
		std::cout << "Couldn't start audio queue\n";
		error_ = true;
		return;
	}
}

void SDLAudioHandler::update(const int64_t &clocks) {
	frameClocks += (clocks) / 4;

	while (frameClocks >= SOUND_CLOCKS) {
		frameClocks -= SOUND_CLOCKS;
		endFrame();
	}
//	std::cout << "fc: " << std::dec << frameClocks << std::hex << std::endl;
}

void SDLAudioHandler::endFrame() {
#if defined(_WIN32) || defined(Y3E_RPI)
	const int ticksToTime = 2;
#else
	const int ticksToTime = 4;
#endif

	const bool isStereo = apu.end_frame(SOUND_CLOCKS * ticksToTime);
	stereoBuffer.end_frame(SOUND_CLOCKS * ticksToTime, isStereo);

	play();
//	if (stereoBuffer.samples_avail() >= OUT_BUF_SIZE) {
//		hasAudio_ = true;
//	}
}

void SDLAudioHandler::play() {
//	std::lock_guard<std::mutex> guard(getAudioMutex());

	while (stereoBuffer.samples_avail() >= OUT_BUF_SIZE) {
		const auto samplesRead = stereoBuffer.read_samples(buffer.data(), OUT_BUF_SIZE);
		queue.write(buffer.data(), samplesRead);
	}

	hasAudio_ = false;
}

bool SDLAudioHandler::handleAudioRegisterRead(const uint16_t &address, uint8_t &val) {
	val = apu.read_register(frameClocks, address);

	return true;
}

void SDLAudioHandler::audioDebug() {
	std::cout << "Audio debug info:\n";

	const char * const driverName = SDL_GetCurrentAudioDriver();

	if (driverName == nullptr) {
		std::cout << "\tNo audio driver loaded." << "\n";
	} else {
		std::cout << "\tCurrent audio driver: " << driverName << "\n";
	}

	std::cout << "\tAudio buffer size: " << BUFFER_SIZE << std::endl;
}

void SDLAudioHandler::internalAudioRegisterHandler(const uint16_t &address, const uint8_t &value) {
	apu.write_register(frameClocks, address, value);
}

void SDLAudioHandler::setVolume(double newVol) {
	apu.volume(newVol);
}

}

