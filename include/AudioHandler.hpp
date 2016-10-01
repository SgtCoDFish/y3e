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

#ifndef CODE_TRUNK_INCLUDE_AUDIOHANDLER_HPP_
#define CODE_TRUNK_INCLUDE_AUDIOHANDLER_HPP_

#include <cstdint>

#include <array>
#include <mutex>

namespace y3e {
class Emulator;

/**
 * Options relating to the 1st sound channel, an envelope with
 * sweep function. Set by the various memory registers.
 */
struct channel1_options {
	// set by NR10
	float sweepTime = 0.0f;
	uint8_t sweepShift = 0u;
	int8_t sweepFunction = 1; // 1 or -1, depending on bit3 of NR10

	// set by NR11
	float soundLength = 0u;
	float wavePatternDuty = 0.125f;

	// set by NR12
	uint8_t envelopeSweepNumber = 0u;
	int8_t envelopeFunction = 1; // 1 or -1 depending on bit3 of NR12
	uint8_t envelopeInitialVolume = 0u;

	// set by NR13 and NR14
	void updateFrequency();
	uint16_t frequencyX = 0u;
	float frequency = 0.0f;

	// set by NR14
	bool counterSelection = false;
	bool initial = false;

};

/**
 * Options set for the 2nd tone channel.
 * Set by various NR registers.
 */
struct channel2_options {
	// set by NR21
	float soundLength = 0u;
	float wavePatternDuty = 0.125f;

	// set by NR22
	uint8_t envelopeSweepNumber = 0u;
	int8_t envelopeFunction = 1;
	uint8_t envelopeInitialVolume = 0u;

	// set by NR23 and NR24
	void updateFrequency();
	uint16_t frequencyX = 0u;
	float frequency = 0.0f;

	// set by NR24
	bool counterSelection = false;
	bool initial = false;
};

/**
 * The wave output sound channel (#3).
 * Set by various NR registers.
 */
struct channel3_options {
	// set by NR30
	bool on = false;

	// set by NR31
	float soundLength = 0.0f;

	// set by NR32
	float outputLevel = 0.0f;

	// set by NR33 and NR34
	void updateFrequency();
	uint16_t frequencyX = 0u;
	float frequency = 0.0f;

	// set by NR34
	bool counterSelection = false;
	bool initial = false;

	// set by memory area 0xFF30-0xFF3F
	std::array<uint8_t, 16> wavePatternRAM;
};

/**
 * Options for channel 4, the white noise channel.
 * Set by various NR registers
 */
struct channel4_options {
	// set by NR41
	float soundLength = 0.0f;

	// set by NR42
	uint8_t envelopeSweepNumber = 0u;
	int8_t envelopeFunction = 1;
	uint8_t envelopeInitialVolume = 0u;

	// set by NR43
	float dividingRatio = 0.5f; // set to 0.5f when value is 0
	uint8_t counterStep = 15u; // 0 when set = 15 bits
	uint8_t shiftClockFrequency = 0u;

	float frequency = 0.0f;

	// set by NR44
	bool counterSelection = false;
	bool initial = false;
};

/**
 * Base class to manage audio. Should be extended on a per-backend bassito implement
 * platform-specific audio playing code.
 */
class AudioHandler {
public:
	explicit AudioHandler(Emulator * emulator_);
	virtual ~AudioHandler() = default;

	/**
	 * Called by the Emulator class after every instruction to update the internal counter
	 * of the audio subsystem. clockCycles should be a multiple of 4.
	 */
	virtual void update(const int64_t &clockCycles);

	/**
	 * Called when an audio frame ends (distinct from a graphical frame). Some backends may
	 * call this method on their own; others may call it automatically.
	 */
	virtual void endFrame();

	/**
	 * Allows the audio subsystem to cache its own data for audio register input. May or may not
	 * be overriden, depending on whether or not the backend cares about managing its own data.
	 *
	 * Should return false if the value should be read from the memory in MemoryManager, or true
	 * if the value placed into the value argument is to be used instead.
	 */
	virtual bool handleAudioRegisterRead(const uint16_t &address, uint8_t &value);

	/**
	 * Allows the emulator to pass audio register data into the audio system.
	 */
	void handleAudioRegisterWrite(const uint16_t &address, const uint8_t &value);

	/**
	 * Called by endFrame under some backends, to actually play the data created by the audio
	 * subsystem. May, on other backends, need to be called manually.
	 */
	virtual void play();

	/**
	 * Return true if there is audio to be played.
	 */
	bool hasAudio() const {
		return hasAudio_;
	}

	/**
	 * Allows audio to be run in a separate thread using a common mutex.
	 */
	std::mutex &getAudioMutex() {
		return audioMutex_;
	}

	/**
	 * Increases the volume by a predefined step, up to a maximum volume of 100%.
	 */
	void volumeUp();

	/**
	 * Decreases the volume by a predefined step, down to a minimum of 0%.
	 */
	void volumeDown();

	/**
	 * Return the volume in the range 0.0 - 1.0 where 1.0 is 100% volume.
	 */
	double getVolume() const;

	// Documentation at http://gbdev.gg8.se/wiki/articles/Sound_Controller#Sound_Overview
	// Sound channel 1 - Tone & Sweep
	void handleNR10Write(const uint8_t &value); // 0xFF10
	void handleNR11Write(const uint8_t &value); // 0xFF11
	void handleNR12Write(const uint8_t &value); // 0xFF12
	void handleNR13Write(const uint8_t &value); // 0xFF13
	void handleNR14Write(const uint8_t &value); // 0xFF14

	// Sound Channel 2 - Tone
	void handleNR21Write(const uint8_t &value); // 0xFF16
	void handleNR22Write(const uint8_t &value); // 0xFF17
	void handleNR23Write(const uint8_t &value); // 0xFF18
	void handleNR24Write(const uint8_t &value); // 0xFF19

	// Sound Channel 3 - Wave Output
	void handleNR30Write(const uint8_t &value); // 0xFF1A
	void handleNR31Write(const uint8_t &value); // 0xFF1B
	void handleNR32Write(const uint8_t &value); // 0xFF1C
	void handleNR33Write(const uint8_t &value); // 0xFF1D
	void handleNR34Write(const uint8_t &value); // 0xFF1E

	// 0xFF30 - 0xFF3F
	void handleWavePatternRAMWrite(const uint16_t &address, const uint8_t &value);

	// Sound Channel 4 - Noise
	void handleNR41Write(const uint8_t &value); // 0xFF20
	void handleNR42Write(const uint8_t &value); // 0xFF21
	void handleNR43Write(const uint8_t &value); // 0xFF22
	void handleNR44Write(const uint8_t &value); // 0xFF23

	// Sound Control Register
	void handleNR50Write(const uint8_t &value); // 0xFF24
	void handleNR51Write(const uint8_t &value); // 0xFF25
	void handleNR52Write(const uint8_t &value); // 0xFF26

protected:
	Emulator * emulator;

	double volume_ = 1.0;

	bool soundOn_ = false;

	bool hasAudio_ = false;
	std::mutex audioMutex_;

	virtual void internalAudioRegisterHandler(const uint16_t &address, const uint8_t &value);

	/**
	 * Implementing classes should override to allow for volume settings. newVol is in the range
	 * 0.0 <= newVol <= 1.0 where 1.0 is 100% volume
	 */
	virtual void setVolume(double newVol);

	channel1_options channel1;
	channel2_options channel2;
	channel3_options channel3;
	channel4_options channel4;
};

}

#endif /* CODE_TRUNK_INCLUDE_AUDIOHANDLER_HPP_ */
