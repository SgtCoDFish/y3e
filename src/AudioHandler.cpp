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

#include <cmath>

#include <iostream>

#include "AudioHandler.hpp"

namespace y3e {

AudioHandler::AudioHandler(Emulator * emulator_) :
		        emulator { emulator_ } {
}

void AudioHandler::update(const int64_t &clocks) {
	// do nothing, should be overriden as needed
}

void AudioHandler::endFrame() {
	// do nothing, should be overriden as needed
}

bool AudioHandler::handleAudioRegisterRead(const uint16_t &address, uint8_t &value) {
	return false;
}

void AudioHandler::volumeUp() {
	volume_ += 0.1;

	if(volume_ > 1.0) {
		volume_ = 1.0;
	}

	setVolume(volume_);
}

void AudioHandler::volumeDown() {
	volume_ -= 0.1;

	if(volume_ < 0.0) {
		volume_ = 0.0;
	}

	setVolume(volume_);
}

double AudioHandler::getVolume() const {
	return volume_;
}

void AudioHandler::setVolume(double newVol) {

}

void AudioHandler::handleAudioRegisterWrite(const uint16_t &address, const uint8_t &value) {
	if (address == 0xFF10) {
		handleNR10Write(value);
	} else if (address == 0xFF11) {
		handleNR11Write(value);
	} else if (address == 0xFF12) {
		handleNR12Write(value);
	} else if (address == 0xFF13) {
		handleNR13Write(value);
	} else if (address == 0xFF14) {
		handleNR14Write(value);
	} else if (address == 0xFF16) {
		handleNR21Write(value);
	} else if (address == 0xFF17) {
		handleNR22Write(value);
	} else if (address == 0xFF18) {
		handleNR23Write(value);
	} else if (address == 0xFF19) {
		handleNR24Write(value);
	} else if (address == 0xFF1A) {
		handleNR30Write(value);
	} else if (address == 0xFF1B) {
		handleNR31Write(value);
	} else if (address == 0xFF1C) {
		handleNR32Write(value);
	} else if (address == 0xFF1D) {
		handleNR33Write(value);
	} else if (address == 0xFF1E) {
		handleNR34Write(value);
	} else if (address >= 0xFF30 && address <= 0xFF3F) {
		handleWavePatternRAMWrite(address, value);
	} else if (address == 0xFF20) {
		handleNR41Write(value);
	} else if (address == 0xFF21) {
		handleNR42Write(value);
	} else if (address == 0xFF22) {
		handleNR43Write(value);
	} else if (address == 0xFF23) {
		handleNR44Write(value);
	} else if (address == 0xFF24) {
		handleNR50Write(value);
	} else if (address == 0xFF25) {
		handleNR51Write(value);
	} else if (address == 0xFF26) {
		handleNR52Write(value);
	}

	internalAudioRegisterHandler(address, value);
}

void AudioHandler::play() {

}

void AudioHandler::handleNR10Write(const uint8_t &value) {
	const uint8_t sweepShiftNumber = value & 0x07; // bits 0-2
	const uint8_t sweepIncrease = value & 0x08; // bit 3
	const uint8_t sweepTimeBits = ((value & 0x70) >> 4); // bits 4-6

	channel1.sweepShift = sweepShiftNumber;

	// 1 sets the function to subtraction
	channel1.sweepFunction = (sweepIncrease == 0u ? 1 : -1);

	channel1.sweepTime = (float) sweepTimeBits / 128.0f;
}

void AudioHandler::handleNR11Write(const uint8_t &value) {
	const uint8_t soundLengthData = value & 0x3F; // bits 0-5
	const uint8_t wavePatternDuty = ((value & 0xC0) >> 6); // bits 6-7

	/*
	 * Sound Length = (64 - t1) * (1/256)s
	 * t1 = soundLengthData, 0 <= t1 < 64
	 * Used only if bit 6 of NR14 is set.
	 */
	channel1.soundLength = (64 - soundLengthData) * (1.0f / 256.0f);

	switch (wavePatternDuty) {
	case 0b00:
		channel1.wavePatternDuty = 0.125f;
		break;

	case 0b01:
		channel1.wavePatternDuty = 0.25f;
		break;

	case 0b10:
		channel1.wavePatternDuty = 0.5f;
		break;

	case 0b11:
		channel1.wavePatternDuty = 0.75f;
		break;

#ifndef NDEBUG
	default:
		std::cout << "Invalid channel1 wavePatternDuty (this should be impossible)\n";
		break;
#endif
	}
}

void AudioHandler::handleNR12Write(const uint8_t &value) {
	const uint8_t envelopeSweepNumber = value & 0x07; // bits 0-2
	const uint8_t envelopeDirection = value & 0x08; // bit 3
	const uint8_t initialEnvelopeVolume = ((value & 0xF0) >> 4); // bits 4-7

	channel1.envelopeSweepNumber = envelopeSweepNumber;
	channel1.envelopeFunction = (envelopeDirection == 0u ? 1 : -1);
	channel1.envelopeInitialVolume = initialEnvelopeVolume;
}

void AudioHandler::handleNR13Write(const uint8_t &value) {
	// value is lower 8 bits of channel 1's frequency

	// careful not to clobber bits set by NR14!
	const uint16_t channel1FrequencyUpper = 0xFF00 & channel1.frequencyX;
	channel1.frequencyX = channel1FrequencyUpper + value;
	channel1.updateFrequency();
}

void AudioHandler::handleNR14Write(const uint8_t &value) {
	const uint8_t frequencyHigher3 = value & 0x07; // bits 0-2
	const uint8_t counterSelection = value & 0x40; // bit 6
	const uint8_t setInitial = value & 0x80; // bit 7

	const uint16_t channel1FrequencyLower = channel1.frequencyX & 0x00FF;
	channel1.frequencyX = channel1FrequencyLower + (frequencyHigher3 << 8);
	channel1.updateFrequency();

	channel1.counterSelection = (counterSelection != 0);
	channel1.initial = (setInitial != 0);
}

void AudioHandler::handleNR21Write(const uint8_t &value) {
	const uint8_t soundLengthData = value & 0x3F; // bits 0-5
	const uint8_t wavePatternDuty = ((value & 0xC0) >> 6); // bits 6-7

	/*
	 * Sound Length = (64 - t1) * (1/256)s
	 * t1 = soundLengthData, 0 <= t1 < 64
	 * Used only if bit 6 of NR14 is set.
	 */
	channel2.soundLength = (64 - soundLengthData) * (1.0f / 256.0f);

	switch (wavePatternDuty) {
	case 0b00:
		channel2.wavePatternDuty = 0.125f;
		break;

	case 0b01:
		channel2.wavePatternDuty = 0.25f;
		break;

	case 0b10:
		channel2.wavePatternDuty = 0.5f;
		break;

	case 0b11:
		channel2.wavePatternDuty = 0.75f;
		break;

#ifndef NDEBUG
	default:
		std::cout << "Invalid channel2 wavePatternDuty (this should be impossible)\n";
		break;
#endif
	}
}

void AudioHandler::handleNR22Write(const uint8_t &value) {
	const uint8_t envelopeSweepNumber = value & 0x07; // bits 0-2
	const uint8_t envelopeDirection = value & 0x08; // bit 3
	const uint8_t initialEnvelopeVolume = ((value & 0xF0) >> 4); // bits 4-7

	channel2.envelopeSweepNumber = envelopeSweepNumber;
	channel2.envelopeFunction = (envelopeDirection == 0u ? 1 : -1);
	channel2.envelopeInitialVolume = initialEnvelopeVolume;
}

void AudioHandler::handleNR23Write(const uint8_t &value) {
	// value is lower 8 bits of channel 2's frequency
	const uint16_t channel2FrequencyUpper = 0xFF00 & channel2.frequencyX;
	channel2.frequencyX = channel2FrequencyUpper + value;
	channel2.updateFrequency();
}

void AudioHandler::handleNR24Write(const uint8_t &value) {
	const uint8_t frequencyHigher3 = value & 0x07; // bits 0-2
	const uint8_t counterSelection = value & 0x40; // bit 6
	const uint8_t setInitial = value & 0x80; // bit 7

	const uint16_t channel2FrequencyLower = channel2.frequencyX & 0x00FF;
	channel2.frequencyX = channel2FrequencyLower + (frequencyHigher3 << 8);
	channel2.updateFrequency();

	channel2.counterSelection = (counterSelection != 0);
	channel2.initial = (setInitial != 0);
}

void AudioHandler::handleNR30Write(const uint8_t &value) {
	const uint8_t onOffToggle = value & 0x80; // bit 7

	channel3.on = (onOffToggle != 1);
}

void AudioHandler::handleNR31Write(const uint8_t &value) {
	// value is sound length
	channel3.soundLength = (256.0f - value) * (1.0f / 256.0f);
}

void AudioHandler::handleNR32Write(const uint8_t &value) {
	const uint8_t outputLevel = ((value & 0x60) >> 5); // bits 5-6

	switch (outputLevel) {
	case 0b00:
		channel3.outputLevel = 0.0f;
		break;

	case 0b01:
		channel3.outputLevel = 1.0f;
		break;

	case 0b10:
		channel3.outputLevel = 0.5f;
		break;

	case 0b11:
		channel3.outputLevel = 0.25f;
		break;

#ifndef NDEBUG
	default:
		std::cout << "Impossible value for channel 3 output level\n";
		break;
#endif
	}
}

void AudioHandler::handleNR33Write(const uint8_t &value) {
	// value is lower 8 bits of frequency of channel 3

	const uint16_t channel3FrequencyUpper = 0xFF00 & channel3.frequencyX;
	channel3.frequencyX = channel3FrequencyUpper + value;
	channel3.updateFrequency();
}

void AudioHandler::handleNR34Write(const uint8_t &value) {
	const uint8_t frequencyHigher3 = value & 0x07; // bits 0-2
	const uint8_t counterSelection = value & 0x40; // bit 6
	const uint8_t setInitial = value & 0x80; // bit 7

	const uint16_t channel3FrequencyLower = channel3.frequencyX & 0x00FF;
	channel3.frequencyX = channel3FrequencyLower + (frequencyHigher3 << 8);
	channel3.updateFrequency();

	channel3.counterSelection = (counterSelection != 0);
	channel3.initial = (setInitial != 0);
}

void AudioHandler::handleWavePatternRAMWrite(const uint16_t &address, const uint8_t &value) {
#ifndef NDEBUG
	if (address < 0xFF30 || address > 0xFF3F) {
		std::cout << "Invalid address passed to handleWavePatternRAMWrite (release would ignore): " << address
		        << std::endl;
	}
#endif

	const uint8_t arrayOffset = address - 0xFF30; // wave pattern RAM is 16 byte array

#ifndef NDEBUG
	if (arrayOffset >= channel3.wavePatternRAM.size()) {
		std::cout << "Invalid wave pattern RAM address offset: " << arrayOffset << std::endl;
	}
#endif

	channel3.wavePatternRAM[arrayOffset] = value;
}

void AudioHandler::handleNR41Write(const uint8_t &value) {
	const uint8_t soundLengthData = value & 0x3F; // bits 0-5

	channel4.soundLength = (64.0f - soundLengthData) * (1.0f / 256.0f);
}

void AudioHandler::handleNR42Write(const uint8_t &value) {
	const uint8_t envelopeSweepNumber = value & 0x07; // bits 0-2
	const uint8_t envelopeDirection = value & 0x08; // bit 3
	const uint8_t initialEnvelopeVolume = ((value & 0xF0) >> 4); // bits 4-7

	channel4.envelopeSweepNumber = envelopeSweepNumber;
	channel4.envelopeFunction = (envelopeDirection == 0u ? 1 : -1);
	channel4.envelopeInitialVolume = initialEnvelopeVolume;
}

void AudioHandler::handleNR43Write(const uint8_t &value) {
	const uint8_t dividingRatio = value & 0x07; // bits 0-2
	const uint8_t counterStep = value & 0x08; // bit 3
	const uint8_t shiftFrequency = ((value & 0xF0) >> 4); // bits 4-7

	channel4.dividingRatio = (dividingRatio == 0u ? 0.5f : static_cast<float>(dividingRatio));
	channel4.counterStep = (counterStep != 0 ? 7 : 15);
	channel4.shiftClockFrequency = shiftFrequency;

	channel4.frequency = (524288.0f / channel4.dividingRatio / (std::pow(2, channel4.shiftClockFrequency + 1)));
}

void AudioHandler::handleNR44Write(const uint8_t &value) {
	const uint8_t counterSelection = value & 0x40; // bit 6
	const uint8_t initialSelection = value & 0x80; // bit 7

	channel4.counterSelection = (counterSelection != 0);
	channel4.initial = (initialSelection != 0);
}

void AudioHandler::handleNR50Write(const uint8_t &value) {
	// unexpected that this will be used
//	const uint8_t s1OutputLevel = value & 0x07; // bits 0-2
//	const uint8_t vInToS1 = value & 0x08; // bit 3
//
//	const uint8_t s2OutputLevel = value & 0x70; // bits 4-6
//	const uint8_t vInToS02 = value & 0x80; // bit 7

	// TODO: Impl
}

void AudioHandler::handleNR51Write(const uint8_t &value) {
	// TODO: Impl
}

void AudioHandler::handleNR52Write(const uint8_t &value) {
	// bits 0-3 are read only
	const uint8_t stopAll = value & 0x80; // bit 7

	soundOn_ = (stopAll != 0);
}

void AudioHandler::internalAudioRegisterHandler(const uint16_t &address, const uint8_t &value) {
	// do nothing, intended to be overriden if needed
}

void channel1_options::updateFrequency() {
	// freq = 131072 / (2048-x) Hz

	frequency = 131072.0f / (2048.0f - frequencyX);
}

void channel2_options::updateFrequency() {
	// freq = 131072 / (2048-x) Hz

	frequency = 131072.0f / (2048.0f - frequencyX);
}

void channel3_options::updateFrequency() {
	// freq = 65536.0f / (2048.0f - x)

	frequency = 65536.0f / (2048.0f - frequencyX);
}

}
