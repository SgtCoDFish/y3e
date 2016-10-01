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

#ifndef CODE_TRUNK_SDL2_SDLAUDIOHANDLER_HPP_
#define CODE_TRUNK_SDL2_SDLAUDIOHANDLER_HPP_

#include "AudioHandler.hpp"

#include <SDL2/SDL.h>
#include "gb_apu/Gb_Apu.h"
#include "Sound_Queue.h"
#include "gb_apu/Multi_Buffer.h"

namespace y3e {

/**
 * Uses SDL2 (and potentially SDL2_mixer) to output audio faithful to
 * the sounds on the orignal cartridge.
 */
class SDLAudioHandler final : public AudioHandler {
public:
	explicit SDLAudioHandler(Emulator * emulator_);
	virtual ~SDLAudioHandler() = default;

	virtual bool handleAudioRegisterRead(const uint16_t &address, uint8_t &value) override final;

	virtual void update(const int64_t &clocks) override final;
	virtual void endFrame() override final;

	virtual void play() override final;

	bool hasSDLAudioError() const {
		return error_;
	}

protected:
	virtual void setVolume(double newVol) override final;
	virtual void internalAudioRegisterHandler(const uint16_t &address, const uint8_t &value) override final;

private:
	static const constexpr int64_t SOUND_CLOCKS = 20000;
	static const constexpr size_t SAMPLE_RATE = 41000;
	static const constexpr size_t BUFFER_SIZE = (SAMPLE_RATE / 60) * 4;
	static const constexpr int OUT_BUF_SIZE = BUFFER_SIZE / sizeof(blip_sample_t);

	gb_time_t frameClocks = 0;

	Gb_Apu apu;
	Stereo_Buffer stereoBuffer;
	Sound_Queue queue;

	std::array<blip_sample_t, BUFFER_SIZE> buffer;

	bool error_ = false;

	void audioDebug();
};

}

#endif /* CODE_TRUNK_SDL2_SDLAUDIOHANDLER_HPP_ */
