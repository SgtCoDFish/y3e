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

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <ShlObj.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#endif

#include <cstdlib>

#include <iostream>
#include <memory>

#include "SDLPlatformTools.hpp"

namespace y3e {

void SDLPlatformTools::delay(const uint32_t &delayInMS) const {
	SDL_Delay(delayInMS);
}

std::string SDLPlatformTools::getSaveLocation() {
	if (hasCachedSaveLocation_) {
		return cachedSaveLocation_;
	}

#ifdef _WIN32
	PWSTR outptr;

	auto result = SHGetKnownFolderPath(FOLDERID_SavedGames, 0, nullptr, &outptr);

	if (result != S_OK) {
		std::cout << "Couldn't find \"Saved Games\" folder. Defaulting to saving in the same directory as y3e.exe; this might need admin privileges!\n";
		cachedSaveLocation_ = "";
		hasCachedSaveLocation_ = true;
		return cachedSaveLocation_;
	}

	// calculate buffer length
	const auto bufferLength = WideCharToMultiByte(CP_UTF8, 0, outptr, -1, nullptr, 0, nullptr, nullptr);
	auto buffer = std::make_unique<char[]>(bufferLength);

	const auto convResult = WideCharToMultiByte(CP_UTF8, 0, outptr, -1, buffer.get(), bufferLength, nullptr, nullptr);
	if (convResult == 0) {
		std::cout << "Windows unicode error. Defaulting to saving in the same directory as y3e.exe; this could need admin status!\n";
		cachedSaveLocation_ = "";
		hasCachedSaveLocation_ = true;
		return cachedSaveLocation_;
	}

	CoTaskMemFree(outptr);

	cachedSaveLocation_ = std::string(buffer.get()) + "\\y3e\\";

	const DWORD fileResult = GetFileAttributes(cachedSaveLocation_.c_str());

	if (CreateDirectory(cachedSaveLocation_.c_str(), nullptr) == 0) {
		const DWORD err = GetLastError();

		// we expect it already exists most of the time
		if (err != ERROR_ALREADY_EXISTS) {
			std::cout << "Couldn't create Y3E save directory. Defaulting to saving in the same directory as y3e.exe; this could need admin status!\n";
			cachedSaveLocation_ = "";
			hasCachedSaveLocation_ = true;
			return cachedSaveLocation_;
		}
	}

	hasCachedSaveLocation_ = true;
#else
	const char * homeDir = ::getenv("HOME");

	if (homeDir == nullptr) {
		// try to locate from passwd
		passwd * pw = ::getpwuid(::getuid());

		if (pw == nullptr) {
			std::cout << "Error while creating root directory for save data. Writing to same dir as y3e executable.\n"
			        << "Error: " << ::strerror(errno) << std::endl;
			cachedSaveLocation_ = "";
			return cachedSaveLocation_;
		}

		homeDir = pw->pw_dir;
	}

	hasCachedSaveLocation_ = true;
	cachedSaveLocation_ = std::string(homeDir) + "/.y3e/";

	const int res = mkdir(cachedSaveLocation_.c_str(), S_IREAD | S_IWRITE | S_IEXEC);

	if (res == -1) {
		if (errno != EEXIST) {
			std::cout << "Error while creating root directory for save data. Writing to same dir as y3e executable.\n"
			        << "Error: " << ::strerror(errno) << std::endl;
			cachedSaveLocation_ = "";
			return cachedSaveLocation_;
		}
	} else {
		// created directory
		std::cout << "Created directory " << cachedSaveLocation_ << "\n";
	}
#endif

	std::cout << "Save location is: " << cachedSaveLocation_ << std::endl;
	return cachedSaveLocation_;
}

}
