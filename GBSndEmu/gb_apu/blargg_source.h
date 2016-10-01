// By default, #included at beginning of library source files

// Copyright (C) 2005 Shay Green.

#ifndef BLARGG_SOURCE_H
#define BLARGG_SOURCE_H

// If debugging is enabled, abort program if expr is false. Meant for checking
// internal state and consistency. A failed assertion indicates a bug in the module.
// void assert( bool expr );
#include <cassert>

// If expr returns non-NULL error string, return it from current function, otherwise continue.
#define BLARGG_RETURN_ERR( expr ) do {                          \
		blargg_err_t blargg_return_err_ = (expr);               \
		if ( blargg_return_err_ ) return blargg_return_err_;    \
	} while ( 0 )

// If ptr is NULL, return out of memory error string.
#define BLARGG_CHECK_ALLOC( ptr )   do { if ( !(ptr) ) return "Out of memory"; } while ( 0 )

#endif

