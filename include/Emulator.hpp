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

#ifndef CODE_TRUNK_INCLUDE_EMULATOR_HPP_
#define CODE_TRUNK_INCLUDE_EMULATOR_HPP_

#include <cstdint>

#include <memory>
#include <string>
#include <array>
#include <chrono>
#include <vector>

#include "ROM.hpp"
#include "ROMParser.hpp"
#include "Registers.hpp"
#include "Instructions.hpp"
#include "MemoryManager.hpp"
#include "StackHandler.hpp"
#include "InputManager.hpp"
#include "GPU.hpp"
#include "Renderer.hpp"
#include "PaletteManager.hpp"
#include "InterruptHandler.hpp"
#include "InstructionParser.hpp"
#include "StateManager.hpp"
#include "AudioHandler.hpp"
#include "SerialHandler.hpp"
#include "PlatformTools.hpp"
#include "BatteryHandler.hpp"

namespace y3e {

/**
 * Abstract class which holds all of the details of a whole emulation session.
 *
 * Should be subclassed to implement an emulator specific to a
 * platform, as required. Specifically the InputManager, Renderer and
 * AudioHandler need to be implemented on a per platform basis.
 */
class Emulator {
public:
	static const constexpr uint64_t BATTERY_FILE_VERSION = 0u;

	explicit Emulator() = default;
	virtual ~Emulator() = default;

	/**
	 * Implementing classes should return a descriptive name to detail the backend.
	 * May be displayed to the user.
	 */
	virtual const char * getPlatformName() const = 0;

	Registers reg;
	MemoryManager memory { this };
	GPU gpu { this };
	InterruptHandler interruptHandler { this };
	PaletteManager paletteManager { this };
	StackHandler stack { this };
	StateManager stateManager { this };
	BatteryHandler batteryHandler { this };

	/**
	 * Implementing classes should set this to a renderer valid for their platform.
	 *
	 * Failure to do this will result in a failure for the emulator to start.
	 */
	std::unique_ptr<Renderer> renderer;

	/**
	 * Implementing classes should set this to an input manager valid for their platform.
	 *
	 * Failure to do this will result in a failure for the emulator to start.
	 */
	std::unique_ptr<InputManager> input;

	/**
	 * Implementing classes should set this to a platform specific tools provider
	 * valid for their platform.
	 *
	 * Failure to do this will result in a failure for the emulator to start.
	 */
	std::unique_ptr<PlatformTools> platformTools;

	/**
	 * Implementing classes should set this to an audio manager valid for their platform,
	 * or leave the default audio handler which does nothing.
	 *
	 * If no audio handler is set the default will be used automatically.
	 */
	std::unique_ptr<AudioHandler> audio;

	/**
	 * Implementing classes should set this to a serial handler valid for their platform,
	 * or the default SerialHandler which does nothing.
	 *
	 * If no serial handler is set the default will be used automatically.
	 */
	std::unique_ptr<SerialHandler> serialHandler;

	/**
	 * Starts the emulator, handing all future processing on the calling thread to the emulator
	 * class until this function returns. It is *strongly* reccommended to call run() instead of
	 * doRun() as run carries out checks to make sure that the emulator has been sufficiently
	 * initialised for running.
	 *
	 * Returns true if everything was initialised correctly, or false if the emulator cannot start
	 * because of some error.
	 */
	bool run();

	/**
	 * Starts the emulator, handing all future processing on the calling thread to the emulator
	 * class until this function returns.
	 *
	 * It is *strongly* advised that you do NOT use this method; instead, call run() which performs
	 * several checks. If any of those checks fail, the emulator is almost certain not to work.
	 */
	void doRun();

	/**
	 * Reads one instruction from the program counter using the given instruction parser
	 * and executes it in the context of this emulator instance.
	 *
	 * Returns the number of cycles taken by the instruction
	 */
	uint8_t doCPU(ArrayInstructionParser &ip);

	/**
	 * Enables halt mode; typically this is not called manually but is set during CPU execution
	 * when a HALT instruction is encountered.
	 */
	void doHalt();

	/**
	 * Allows manual control of the halt flag. Typically handled internally.
	 */
	void setHalt(bool halt_) {
		this->haltMode_ = halt_;
	}

	/**
	 * Enables the stop flag; typically not called manually but set during CPU execution
	 */
	void doStop();

	/**
	 * Implementing classes should add any functionality here which needs to be run at the start of
	 * every frame, which here means before the CPU is run every instruction.
	 */
	virtual void frameBegin() = 0;

	/**
	 * Implementing classes should add any functionality here which needs to be run at the end of
	 * every frame, which here means after the CPU is run every instruction.
	 */
	virtual void frameEnd() = 0;

	/**
	 * The main loop run during debug mode. Normally not called externally.
	 */
	void handleDebug();

	/**
	 * Move an already-loaded ROM file into the emulator for use. A ROM must be loaded before
	 * run() is called.
	 */
	void loadROM(std::unique_ptr<ROM> &&rom);
	void loadROM(const std::string &fileName);
	void unloadROM();

	void addBreakPoint(uint16_t breakPoint);

	/**
	 * Sets the frame limiter to a multiple of 60fps.
	 *
	 * Normally the limiter keeps the frame rate at 60fps, which is the native frame rate of the
	 * Game Boy. Setting the limiter to 2 would limit the frame rate at 120fps, etc.
	 */
	void setFrameLimitRate(uint8_t limiter);

	void enableDebug() {
		debugMode_ = true;
		enableDebugPrinting();
	}

	void enableDebugPrinting() {
		debugPrint_ = true;
	}

	bool debugEnabled() const {
		return debugMode_;
	}

	ROM * getROM() const {
		return rom.get();
	}

	/**
	 * Adds clocks to the emulator from an external source. This should be used vary sparingly
	 * as it adds uncertainty from outside the emulator, which will negatively impact emulation
	 * accuracy. Clocks should be a multiple of 4.
	 * @param clocks
	 */
	void addClocks(int64_t clocks) {
		clockCount_ += clocks;
	}

	int64_t getClocks() const {
		return clockCount_;
	}

	/**
	 * Should be called by other emulator components to indicate that a vblank has occurred; if
	 * frame limiting is enabled this will happen at 60fps if the computer is fast enough.
	 */
	virtual void signalVBlank();

	/**
	 * Requests that the emulator quit as soon as possible.
	 */
	void signalQuit();

	bool shouldQuit() const {
		return shouldQuit_;
	}

	/**
	 * Signals to the emulator that the user pressed some key combination which indicates they wish to
	 * quit. Depending on setup, will exit immediately or show some kind of confirmation dialog.
	 */
	void signalQuitRequested();

	/**
	 * Prints instruction details for every single instruction parsed.
	 *
	 * Warning: very slow.
	 */
	void setDebugPrintingForAllInstructions(bool on) {
		debugPrintAllInstructions_ = on;
	}

	/**
	 * Toogles printing instruction details for every single instruction parsed.
	 *
	 * Warning: very slow.
	 */
	void toggleDebugPrintingForAllInstructions() {
		debugPrintAllInstructions_ = ~debugPrintAllInstructions_;
	}

	/**
	 * Print a message to stdout only if debug mode is enabled.
	 */
	void debugPrint(const std::string &message);

	/**
	 * Allows the emulator to insert artificial delays on a per-frame basis if emulation would
	 * have been faster than the Game Boy ran (i.e. if the frame rate is greater than the Game Boy's
	 * native 60fps).
	 */
	void enableFPSLimiting() {
		fpsLimit_ = true;
	}

	/**
	 * Disables frame rate limiting, which could lead to games which run far too quickly.
	 */
	void disableFPSLimiting() {
		fpsLimit_ = false;
	}

	bool fpsLimitingEnabled() const {
		return fpsLimit_;
	}

	std::string makeStateName(int64_t stateNumber = -1);

	/**
	 * Sets an error state on the emulator, indicating to the outside world that something went wrong
	 * during emulation. This is usually an indicator that progress cannot continue.
	 */
	void setErrorState(const std::string &errorMessage);

	/**
	 * Used to reset the error state in the event that whatever caused the error is fixed.
	 */
	void resetErrorState();
	bool hasError() const;
	std::string getErrorMessage() const;

	/**
	 * Returns the average FPS measured while the emulator has been running.
	 * Useful for after the emulator has shut down for a guideline on performance.
	 *
	 * Returns a negative number if the emulator didn't run long enough to get a
	 * meaningful average.
	 */
	double getAverageFPS() const;

protected:
	friend class StateManager;

	void handleBatteryInitial();

	std::unique_ptr<ROM> rom;

	bool shouldQuit_ = false;
	bool quitRequested_ = false;

	bool hasBreakPoint_ = false;
	uint16_t breakPoint_ = 0u;

	bool debugMode_ = false;
	bool debugPrint_ = false;
	bool debugPrintAllInstructions_ = false;

	int64_t clockCount_ = 0;

	bool haltMode_ = false;
	bool stopMode_ = false;

	bool fpsLimit_ = true;

	bool hasError_ = false;
	std::string errorMessage_ { "No error" };

	// roughly the game boy's native fps
	double fpsTargetGB = 1.0 / 60.00;
	static constexpr const unsigned int Y3E_COUNT_MAX = 300;

	// durations of a frame are added to the array and timeCounter is incremented
	// every frame
	std::array<float, Y3E_COUNT_MAX> timeArray;
	uint32_t timeCounter = 0;

	// the accumulation counter is the number of averages taken so far
	std::array<double, Y3E_COUNT_MAX> averageArray;
	uint32_t accumulationCounter = 0;

	std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
};

}

#endif /* CODE_TRUNK_INCLUDE_EMULATOR_HPP_ */
