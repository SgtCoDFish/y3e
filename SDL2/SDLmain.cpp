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

#include <cstdlib>
#include <cstdint>
#include <cstring>

#include <iostream>
#include <iomanip>
#include <memory>
#include <sstream>

#include <SDL2/SDL.h>

#include <APG/SXXDL.hpp>

#include "ROMParser.hpp"
#include "SDLEmulator.hpp"
#include "Instructions.hpp"
#include "InstructionParser.hpp"
#include "tinyfiledialogs.h"
#include "Y3EPlatformOptions.hpp"

int main(int argc, char *argv[]) {
	std::cout << "Y3E version " << y3e::Version::versionString << std::endl;

	int sdlFlags = SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER;

	if (SDL_Init(sdlFlags) != 0) {
		std::cout << "SDL_INIT Failed: " << SDL_GetError() << std::endl;
		return EXIT_FAILURE;
	}

	bool audioEnabled = true;

	bool hasDesiredAudio = false;
	const char * const desiredAudio = "alsa";
	const int numAudioDrivers = SDL_GetNumAudioDrivers();
	for (int i = 0; i < numAudioDrivers; i++) {
		const char * const audioDriver = SDL_GetAudioDriver(i);

		// std::cout << "Audio driver available: " << audioDriver << std::endl;

		if (std::strcmp(audioDriver, desiredAudio) == 0) {
			hasDesiredAudio = true;
		}
	}

	if (hasDesiredAudio) {
		if (SDL_AudioInit(desiredAudio) != 0) {
			std::cout << "Platform has desired audio target \"" << desiredAudio << "\" but couldn't start it: "
			        << SDL_GetError();

			if (SDL_AudioInit(nullptr) != 0) {
				std::cout << "Also couldn't start default audio target (error: " << SDL_GetError()
				        << ") , disabling audio.\n";
				audioEnabled = false;
			}
		} else {
			std::cout << "Manually chose desired audio target \"" << desiredAudio << "\".\n";
		}
	} else if (SDL_AudioInit(nullptr) != 0) {
		std::cout << "Couldn't start default audio target: \"" << SDL_GetError() << "\", disabling audio.\n";
		audioEnabled = false;
	}

	y3e::SDLEmulator emulator(audioEnabled);

	std::string fileName;

	if (argc >= 2) {
		for (int i = 1; i < argc; i++) {
			if (std::strcmp(argv[i], "--break-at") == 0) {
				std::stringstream ss;

				if (i + 1 == argc) {
					std::cerr << "--break-at requires a hex memory address as an argument.\n";
					return EXIT_FAILURE;
				}

				ss << std::hex << argv[i + 1];

				uint16_t breakPoint;

				ss >> breakPoint;

				std::cout << "Adding breakpoint at 0x" << std::hex << breakPoint << std::endl;

				emulator.addBreakPoint(breakPoint);
			} else if (std::strcmp(argv[i], "--debug-all") == 0) {
				emulator.setDebugPrintingForAllInstructions(true);
			} else if (std::strcmp(argv[i], "--no-fps-limit") == 0) {
				emulator.disableFPSLimiting();
			} else if (std::strcmp(argv[i], "--debug-print") == 0) {
				emulator.enableDebugPrinting();
			} else if (std::strcmp(argv[i], "--no-audio") == 0 || std::strcmp(argv[i], "--disable-audio") == 0) {
				emulator.disableSDLAudio();
			} else if (std::strcmp(argv[i], "--no-tile-caching") == 0) {
				emulator.gpu.disableTileCaching();
			} else {
				fileName = argv[i];
			}
		}
	}

	if (fileName == "") {
		static const char * const filters[] = { "*.gb", "*.gbc" };
		const char * openFileName = tinyfd_openFileDialog("Select a .gb file", "", 2, filters, "Game Boy ROMs", false);

		if (openFileName == nullptr) {
			std::cout << "No ROM file selected, exiting.\n";
			return 1;
		}

		fileName = openFileName;
	}

	emulator.loadROM(fileName);

	if (emulator.hasError()) {
		std::cout << "Error detected: " << emulator.getErrorMessage() << std::endl;
		return 1;
	}

	std::cout << *emulator.getROM() << std::endl;

	emulator.run();

	const double averageFPS = emulator.getAverageFPS();

	if (averageFPS > 0.0) {
		std::cout << "Overall average FPS: " << averageFPS << std::endl;
	}

	std::cout << "Thanks for playing!\n";

	SDL_Quit();
	return EXIT_SUCCESS;
}
