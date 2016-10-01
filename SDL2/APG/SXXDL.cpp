/*
 * Copyright (c) 2014, 2015 See AUTHORS file.
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

#include <SDL2/SDL.h>
//#include <SDL2/SDL_ttf.h>

#include "APG/SXXDL.hpp"

SXXDL::window_ptr SXXDL::make_window_ptr(SDL_Window *window) {
	return window_ptr(window, SDL_DestroyWindow);
}

SXXDL::surface_ptr SXXDL::make_surface_ptr(SDL_Surface *surface) {
	return surface_ptr(surface, SDL_FreeSurface);
}

SXXDL::sdl_texture_ptr SXXDL::make_sdl_texture_ptr(SDL_Texture *texture) {
	return sdl_texture_ptr(texture, SDL_DestroyTexture);
}

SXXDL::renderer_ptr SXXDL::make_renderer_ptr(SDL_Renderer *renderer) {
	return renderer_ptr(renderer, SDL_DestroyRenderer);
}

SXXDL::pixel_format_ptr SXXDL::make_pixel_format_ptr(SDL_PixelFormat *format) {
	return pixel_format_ptr(format, SDL_FreeFormat);
}

