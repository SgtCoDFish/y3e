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

#ifndef CODE_TRUNK_INCLUDE_BATTERYHANDLER_HPP_
#define CODE_TRUNK_INCLUDE_BATTERYHANDLER_HPP_

#include <cstdint>

#include <chrono>
#include <fstream>
#include <string>

namespace y3e {
class Emulator;

/**
 *
 */
class BatteryHandler {
public:
	static const constexpr uint64_t BATTERY_FILE_VERSION = 2u;
	static constexpr const double SAVE_DELAY_SECONDS = 10.0;

	explicit BatteryHandler(Emulator * emulator_, bool verbose_ = false);
	~BatteryHandler() = default;

	/**
	 * Saves the battery file to disk if at least SAVE_DELAY_SECONDS have passed since the last
	 * save external RAM has changed in that interval.
	 */
	void update(bool forceWrite = false);

	/**
	 * Writes the current contents of external RAM to the battery file to the filename
	 * generated by makeBatteryName().
	 */
	void writeBattery(uint64_t fileVersion = BATTERY_FILE_VERSION);

	/**
	 * Writes the current contents of external RAM to the battery file to the filename
	 * given.
	 */
	void writeBattery(const std::string &filename, uint64_t fileVersion = BATTERY_FILE_VERSION);

	/*
	 * Writes a battery to file along with outputting a message describing that this was done 
	 * because the emulator is shutting down.
	 */
	void writeFinalBattery(uint64_t fileVersion = BATTERY_FILE_VERSION);

	/**
	 * Loads a battery from a file into the emulated external RAM of the emulator. The filename
	 * is the same as that generated by makeBatteryName.
	 */
	void loadBattery();

	/**
	 * Loads a battery from a file into the emulated external RAM of the emulator. The filename
	 * is provided.
	 */
	void loadBattery(const std::string &filename);

	/**
	 * Writes a default battery; an empty file containing nothing but the battery file header.
	 */
	void createDefaultBattery();

	/**
	 * Returns the filename that the game's battery should be loaded from / saved to.
	 */
	std::string makeBatteryName();

	/**
	 * Return true if a battery has been loaded. This may be important if state is being loaded fresh
	 * as the state will overwrite the battery.
	 */
	bool batteryLoaded() const {
		return batteryLoaded_;
	}

	/**
	 * Notifies the BatteryHandler that state has been affected outside of the emulator and that 
	 * batteries should no longer write batteries.
	 */
	void notifyStateLoad();

private:
	std::string makeVersionNumber(uint64_t number);

	void doSaveBackup(uint64_t oldVersionNumber);

	void writeBatteryVersion1(const std::string &filename);
	void writeBatteryVersion2(const std::string &filename);

	void loadBatteryVersion1(std::ifstream &f);
	void loadBatteryVersion2(std::ifstream &f);

	/**
	 * Returns a default constructed version array guaranteed to hold a version number as saved in a battery.
	 */
	std::array<char, 9> getVersionArray() {
		return std::array<char, 9>();
	}

	Emulator * emulator;

	bool batteryLoaded_ = false;

	std::chrono::time_point<std::chrono::high_resolution_clock> lastSave;

	bool stateLoaded = false;
	bool verbose = false;
};

}

#endif /* CODE_TRUNK_INCLUDE_BATTERYHANDLER_HPP_ */
