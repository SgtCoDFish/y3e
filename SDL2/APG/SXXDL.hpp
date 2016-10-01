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

#ifndef INCLUDE_APG_SXXDL_INCLUDE_HPP_
#define INCLUDE_APG_SXXDL_INCLUDE_HPP_

/*
 * This file contains various helper type definitions to make SDL2 safer and easier to use
 * in a C++ environment.
 *
 * It is adapted from the APG library by the same author, located at:
 * https://github.com/SgtCoDFish/APG
 */

#include <memory>

#include <SDL2/SDL.h>

namespace SXXDL {

using window_ptr = std::unique_ptr<SDL_Window, void(*)(SDL_Window *)>;
using surface_ptr = std::unique_ptr<SDL_Surface, void(*)(SDL_Surface *)>;
using sdl_texture_ptr = std::unique_ptr<SDL_Texture, void(*)(SDL_Texture *)>;
using renderer_ptr = std::unique_ptr<SDL_Renderer, void(*)(SDL_Renderer *)>;
using pixel_format_ptr = std::unique_ptr<SDL_PixelFormat, void(*)(SDL_PixelFormat *)>;

window_ptr make_window_ptr(SDL_Window *window);
surface_ptr make_surface_ptr(SDL_Surface *surface);
sdl_texture_ptr make_sdl_texture_ptr(SDL_Texture *texture);
renderer_ptr make_renderer_ptr(SDL_Renderer *renderer);
pixel_format_ptr make_pixel_format_ptr(SDL_PixelFormat *format);

}

#endif /* SXXDL_HPP_ */
