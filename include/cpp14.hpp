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

#ifdef Y3E_MAKE_UNIQUE
#ifndef CODE_TRUNK_INCLUDE_CPP14_HPP_
#define CODE_TRUNK_INCLUDE_CPP14_HPP_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

/*
 * This file contains an implementation of C++14's std::make_unique as
 * presented in the C++14 standard, the relevant draft proposal of which can
 * be found at:
 *
 * https://isocpp.org/files/papers/N3656.txt
 *
 * It should only be pulled in if std::make_unique isn't provided by the
 * compiler being used. This is generally not the case but less-than-modern
 * versions of g++ sometimes have this problem.
 */

namespace std {
	template<class T> struct _Unique_if {
		typedef unique_ptr<T> _Single_object;
	};

	template<class T> struct _Unique_if<T[]> {
		typedef unique_ptr<T[]> _Unknown_bound;
	};

	template<class T, size_t N> struct _Unique_if<T[N]> {
		typedef void _Known_bound;
	};

	template<class T, class ... Args>
	typename _Unique_if<T>::_Single_object make_unique(Args&&... args) {
		return unique_ptr<T>(new T(std::forward<Args>(args)...));
	}

	template<class T>
	typename _Unique_if<T>::_Unknown_bound make_unique(size_t n) {
		typedef typename remove_extent<T>::type U;
		return unique_ptr<T>(new U[n]());
	}

	template<class T, class ... Args>
	typename _Unique_if<T>::_Known_bound make_unique(Args&&...) = delete;
}

#endif /* CODE_TRUNK_INCLUDE_CPP14_HPP_ */
#endif
