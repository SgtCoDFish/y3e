// Gb_Snd_Emu 0.1.4. http://www.slack.net/~ant/libs/

#include "Wave_Writer.h"

#include <cassert>
#include <cstdlib>
#include <cstdint>
#include <cstdio>

/* Copyright (C) 2003-2005 by Shay Green. Permission is hereby granted, free
 of charge, to any person obtaining a copy of this software and associated
 documentation files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 to permit persons to whom the Software is furnished to do so, subject to the
 following conditions: The above copyright notice and this permission notice
 shall be included in all copies or substantial portions of the Software. THE
 SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */

const uint32_t header_size = 0x2C;

static void exit_with_error(const char* str) {
	fprintf( stderr, "Error: %s\n", str);
	exit( EXIT_FAILURE);
}

Wave_Writer::Wave_Writer(long sample_rate, const char* filename) {
	sample_count_ = 0;
	rate = sample_rate;
	buf_pos = header_size;
	stereo(0);

	buf = new unsigned char[buf_size];
	if (!buf) {
		exit_with_error("Out of memory");
	}

	file = std::fopen(filename, "wb");
	if (!file) {
		exit_with_error("Couldn't open WAVE file for writing");
	}
}

void Wave_Writer::flush() {
	if (buf_pos && !fwrite(buf, buf_pos, 1, file))
		exit_with_error("Couldn't write WAVE data");
	buf_pos = 0;
}

void Wave_Writer::write(const sample_t* in, long remain, int skip) {
	sample_count_ += remain;
	while (remain) {
		if (buf_pos >= buf_size)
			flush();

		long n = (unsigned long) (buf_size - buf_pos) / sizeof(sample_t);
		if (n > remain)
			n = remain;
		remain -= n;

		// convert to lsb first format
		unsigned char* p = &buf[buf_pos];
		while (n--) {
			int s = *in;
			in += skip;
			*p++ = (unsigned char) s;
			*p++ = (unsigned char) (s >> 8);
		}

		buf_pos = p - buf;
		assert(buf_pos <= buf_size);
	}
}

void Wave_Writer::write(const float* in, long remain, int skip) {
	sample_count_ += remain;
	while (remain) {
		if (buf_pos >= buf_size)
			flush();

		long n = (unsigned long) (buf_size - buf_pos) / sizeof(sample_t);
		if (n > remain)
			n = remain;
		remain -= n;

		// convert to lsb first format
		unsigned char* p = &buf[buf_pos];
		while (n--) {
			long s = *in * 0x7fff;
			in += skip;
			if ((short) s != s)
				s = 0x7fff - (s >> 24); // clamp to 16 bits
			*p++ = (unsigned char) s;
			*p++ = (unsigned char) (s >> 8);
		}

		buf_pos = p - buf;
		assert(buf_pos <= buf_size);
	}
}

Wave_Writer::~Wave_Writer() {
	flush();

	// ignore for now because there are issues with narrowing under C++11

	// generate header
	long ds = sample_count_ * sizeof(sample_t);
	long rs = header_size - 8 + ds;
	int frame_size = chan_count * sizeof(sample_t);
	long bps = rate * frame_size;
	uint8_t header[header_size] = { 'R', 'I', 'F',
	        'F', // into
	        static_cast<uint8_t>(rs),
	        static_cast<uint8_t>(rs >> 8),           // length of rest of file
	        static_cast<uint8_t>(rs >> 16),
	        static_cast<uint8_t>(rs >> 24), // more lengths
	        'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 0x10, 0, 0,
	        0,         // size of fmt chunk
	        1,
	        0,                // uncompressed format
	        static_cast<uint8_t>(chan_count),
	        0,       // channel count
	        static_cast<uint8_t>(rate),
	        static_cast<uint8_t>(rate >> 8),     // sample rate
	        static_cast<uint8_t>(rate >> 16), static_cast<uint8_t>(rate >> 24), static_cast<uint8_t>(bps),
	        static_cast<uint8_t>(bps >> 8),         // bytes per second
	        static_cast<uint8_t>(bps >> 16), static_cast<uint8_t>(bps >> 24), static_cast<uint8_t>(frame_size),
	        0,       // bytes per sample frame
	        16,
	        0,               // bits per sample
	        'd', 'a', 't', 'a', static_cast<uint8_t>(ds), static_cast<uint8_t>(ds >> 8), static_cast<uint8_t>(ds >> 16),
	        static_cast<uint8_t>(ds >> 24)               // size of sample data
	        // ...              // sample data
	        };

	// write header
	std::fseek(file, 0, SEEK_SET);
	std::fwrite(header, sizeof header, 1, file);

	std::fclose(file);
	delete[] buf;
}

