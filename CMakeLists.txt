cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_LEGACY_CYGWIN32 0)

project(Y3E)

set(Y3E_VERSION_MAJOR 1)
set(Y3E_VERSION_MINOR 0)
set(Y3E_VERSION_PATCH 0)

if (NOT DEFINED Y3E_VERSION_EXTRA)
    set(Y3E_VERSION_EXTRA "")
endif ()

set(Y3E_VERSION "${Y3E_VERSION_MAJOR}.${Y3E_VERSION_MINOR}.${Y3E_VERSION_PATCH}")

if ( Y3E_VERSION_EXTRA )
	set(Y3E_VERSION "${Y3E_VERSION}-${Y3E_VERSION_EXTRA}")
endif ()

if ( CMAKE_BUILD_TYPE MATCHES Release )
    message("Building release library.")
    set (Y3E_DEBUG FALSE)
    set (gb_apu_DEBUG FALSE)
else ()
    message("Building debug library.")
    set (Y3E_DEBUG TRUE)
    set (gb_apu_DEBUG TRUE)
endif ()

set(Y3E_PLATFORM_NAME "SDL2")
set(Y3E_PLATFORM_INCLUDE_DIR "SDL2")
set(Y3E_PLATFORM_SRC_DIR "SDL2") 

message("Conifguring Y3E version ${Y3E_VERSION} on platform ${Y3E_PLATFORM_NAME}.")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option(Y3E_MAKE_UNIQUE "Should we provide our own implementation of make_unique?" OFF)

find_package(SDL2 REQUIRED)

find_package(GLM REQUIRED)

if ( Y3E_LOCAL_RAPIDJSON )
    set ( RAPIDJSON_INCLUDE_DIRS "include" )
else ()
    set(rapidjson_FIND_QUIETLY REQUIRED)
    find_package(rapidjson)
    
    if ( NOT RAPIDJSON_FOUND )
        set ( RAPIDJSON_INCLUDE_DIRS "include" )
    endif () 
endif ()

if ( Y3E_MAKE_UNIQUE )
	add_definitions(-DY3E_MAKE_UNIQUE)
endif ()

if( MSVC )
	# MSVC
	set(dialect "")
	set(warnings "/wd4804")
	set(flags "/EHsc /MP")
	set(Y3E_DEBUG_FLAGS "/Od")
	set(Y3E_RELEASE_FLAGS "/Ox")
	
	# VS 2015 has a linker issue, this should fix it
	if( MSVC_VERSION GREATER 1800)
		message ("Using legacy STDIO fix for MSVC 15.")
		set(OS_LIBS legacy_stdio_definitions)
		set(EXTRA_SOURCES ${PROJECT_SOURCE_DIR}/MSVC/vs2015_io_fix.cpp)
	endif()
	
	if ( Y3E_MAKE_UNIQUE )
		set(flags "${flags} /FI\"include/cpp14.hpp\"")
	endif ()
else ()
	set(dialect "-std=c++1y")
    set(warnings "-Wall -Wno-strict-aliasing -Wno-unused-parameter")
	
	if( "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" )
	    set(warnings "${warnings} -Wno-unused-private-field")
	endif ()
	
	set(Y3E_DEBUG_FLAGS "-O0 -g3")
	set(Y3E_RELEASE_FLAGS "-O3 -g0")
	
	if ( DEFINED WIN32 )
		# MinGW-w64
		set(flags "-mwindows -mconsole")
		set(OS_LIBS Version Imm32 winmm mingw32 gdi32)
	else ()
		# GCC/Clang
		set(flags "-fPIC")
		set(OS_LIBS "")
	endif ()
	
	if ( Y3E_NATIVE )
	    set(flags "${flags} -march=native") 
	endif ()
	
	if ( Y3E_MAKE_UNIQUE )
		set(flags "${flags} -include \"cpp14.hpp\"")
	endif ()
endif()

file(GLOB_RECURSE Y3E_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/tinyfiledialogs.c)
file(GLOB_RECURSE Y3E_HEADERS ${PROJECT_SOURCE_DIR}/include/*.hpp)
file(GLOB_RECURSE Y3E_PLATFORM_SOURCES ${PROJECT_SOURCE_DIR}/${Y3E_PLATFORM_SRC_DIR}/*.cpp)

if ( NOT DEFINED Y3E_FORCE_QUIT )
    set (Y3E_FORCE_QUIT OFF)
endif () 

configure_file (
	"${PROJECT_SOURCE_DIR}/gen/Y3EPlatformOptions.hpp.in"
	"${PROJECT_BINARY_DIR}/Y3EPlatformOptions.hpp"
)

set(CMAKE_CXX_FLAGS_BASE "${CMAKE_CXX_FLAGS_BASE} ${dialect} ${warnings} ${flags}")

set(EXECUTABLE_NAME "y3e")

if ( Y3E_DEBUG )
    set(CMAKE_CXX_FLAGS_BASE "${CMAKE_CXX_FLAGS_BASE} ${Y3E_DEBUG_FLAGS}")
    set(EXECUTABLE_NAME "${EXECUTABLE_NAME}-d")
else ()
    set(CMAKE_CXX_FLAGS_BASE "${CMAKE_CXX_FLAGS_BASE} ${Y3E_RELEASE_FLAGS}")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_BASE}")

include_directories("include" ${SDL2_INCLUDE_DIR}
							  ${Y3E_PLATFORM_INCLUDE_DIR}
							  ${RAPIDJSON_INCLUDE_DIRS}
							  ${PROJECT_SOURCE_DIR}/GBSndEmu
							  ${PROJECT_BINARY_DIR})
set(LIBS ${SDL2_LIBRARY} ${OS_LIBS} gb_apu)

message("CXX Flags: ${CMAKE_CXX_FLAGS}")

if ( MINGW )
	include_directories(${SDL2_INCLUDE_DIR}/..)
endif ()

if ( UNIX )
    set(LIBS ${LIBS} dl pthread)
endif ()

include(${PROJECT_SOURCE_DIR}/GBSndEmu/CMakeLists.txt)

add_executable(${EXECUTABLE_NAME} ${Y3E_SOURCES} ${Y3E_PLATFORM_SOURCES} ${EXTRA_SOURCES} ${Y3E_HEADERS})
target_link_libraries(${EXECUTABLE_NAME} ${LIBS})
