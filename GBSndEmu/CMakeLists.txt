file(GLOB_RECURSE GB_APU_SOURCES ${PROJECT_SOURCE_DIR}/GBSndEmu/*.cpp)
file(GLOB_RECURSE GB_APU_HEADERS ${PROJECT_SOURCE_DIR}/GBSndEmu/*.h)

#message(${GB_APU_SOURCES})
#message(${GB_APU_HEADERS})

add_library(gb_apu STATIC ${GB_APU_SOURCES} ${GB_APU_HEADERS})

if ( gb_apu_DEBUG )
    add_executable(aputest_sdl ${PROJECT_SOURCE_DIR}/GBSndDemos/demo_sdl.cpp)
    target_link_libraries(aputest_sdl ${LIBS})

    add_executable(aputest_wav ${PROJECT_SOURCE_DIR}/GBSndDemos/demo.cpp)
    target_link_libraries(aputest_wav ${LIBS})
endif ()