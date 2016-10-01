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

#ifndef CODE_TRUNK_INCLUDE_STATEMANAGER_HPP_
#define CODE_TRUNK_INCLUDE_STATEMANAGER_HPP_

#include <cstdint>

#include <chrono>
#include <string>
#include <vector>
#include <fstream>

#include <rapidjson/document.h>

namespace y3e {
class Emulator;

/**
 * Handles saving and loading of game states. This is distinct to in-game saving and loading of batteries
 * which is handled in its own distanct way. State handling is an external feature, unsupported on the original
 * Game Boy and as such must take care to not affect game state if at all possible.
 *
 * State file format:
 * - STATE_HEADER printed verbatim in ASCII
 * - 8 digits, representing the length of the uncompressed output
 * - the compressed output
 */
class StateManager final {
public:
	/**
	 * Turns a vector of bytes into a string of ASCII hex values, padded with 0s if needed.
	 */
	static std::string stringifyByteVector(const std::vector<uint8_t> &vec);

	/**
	 * Parses a string of 2-wide ASCII bytes and returns a vector containing those bytes.
	 */
	static std::vector<uint8_t> vectorFromByteString(const std::string &str);

	/**
	 * The state file version that new states will be saved in. Does not affect loading of states.
	 */
	static const constexpr uint64_t STATE_FILE_VERSION = 0u;
	static const std::string STATE_HEADER;

	explicit StateManager(Emulator * const emulator_);

	/**
	 * Saves a state to disk in the default save location. Will take some processing resources and must
	 * save a large amount of data. Care must be taken to ensure that all relevant emulation data is preserved.
	 */
	void saveState(const std::string &filename, uint64_t fileVersion = STATE_FILE_VERSION);

	/**
	 * Loads state from a file and resets the emulator to that state. If the file does not have save state data for
	 * the currently loaded game the behaviour is undefined.
	 *
	 * Returns true if state was successfully loaded.
	 */
	bool loadState(const std::string &filename);

private:
	Emulator * const emulator;

	std::chrono::high_resolution_clock::time_point lastActionTime;

	bool canTakeAction();

	std::string saveStateVersion0();

	bool loadStateVersion0(const rapidjson::Document &document);
};

}

#endif /* CODE_TRUNK_INCLUDE_STATEMANAGER_HPP_ */
