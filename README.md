# Y3E - A Game Boy Emulator in Modern C++
Y3E is an unofficial Nintendo Game Boy emulation library, packaged together with an SDL2 backend which together create an emulator.
It can read ROM files (.gb) from original cartridges and execute them, with complete support for all instructions and with mostly-correct video and audio handling.

## Usage
Y3E supports both keyboard input and game controller input (tested with the XBox 360 USB controller).

| Game Boy | Keyboard | Game Pad |   
|:--------:|:--------:|:--------:|   
|    A     |    Z     |    A     |   
|    B     |    X     |    B     |   
|  START   |  ENTER   |  START   |   
|  SELECT  | BACKSPACE|  SELECT  |   
|  D-Pad   |  Arrows  |  D-Pad   |   

Note that analog input controls will not work for the Game Pad; only the directional pad is supported.

The following extra keys are also supported:
 - "="   
 Increase Volume
 - "-"   
 Decrease Volume
 - Control + <1-9>   
 Save game state file 1-9 for the loaded game (*)
 - Alt + <1-9>   
 Load game state file 1-9 for the loaded game (*)
 - F1 - F4   
 Set the frame rate limited to n times 60 fps; e.g. F2 sets the limit to 120fps, double speed.
 - F12   
 Enable debug mode.
 - Escape   
 Quit Y3E
 - Control + P   
 Force a save of the battery file. This might overwrite your data!
 
 (*) Regarding save states: see "Y3E Usage Notes"

## Supported Games
The following games have been extensively tested and should work well:
 - Tetris
 - Tennis
 - Pokémon Red/Blue
 - Pokémon Trading Card Game
 - The Legend of Zelda: Link's Awakening
 - Harvest Moon
 
The following games are known to work with some bugs:
 - Pokémon Yellow (title screen must be skipped or the game will crash, Pikachu's "voice" does not work)
 - Super Mario Land (some graphical glitches)

## Y3E Usage Notes
### Multiplayer
Do not use any "2 player" modes in games, as serial handling is not fully implemented and attempts to access 2-player mode can cause the emulator to lock up.

### Save States
Save stats have nothing to do with in game saving; saving will work as normal as long as Y3E can find a place to save your game files.

If you choose to use save states, regular saving will be disabled after a save backup is made.

Save features will work as normal in-game but **NOTHING WILL BE SAVED**. If you decide that you do 
actually want to save as normal, press Control+P to force your batteries to be saved again.

This behaviour is because the emulator would otherwise have to overwrite your saved game files to support state saving.

# Building Y3E
On most systems Y3E can be built using the following commands (assuming that dependencies have been met).

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=<Debug/Release> [other CMake options here] ..
make -j<num_cores>
```

Platform specific information is given below. The easiest system to build for is Ubuntu Linux (tested on 14.04, 15.10 and 16.04). If a platform is not listed below, it isn't officially supported but may still work.

## CMake Options
Y3E has several other possible CMake options which can be passed in when project files are generated. They are:

- `Y3E_MAKE_UNIQUE`   
  Forces Y3E to use its own implementation of `std::make_unique`, assuming that the compiler used only supports C++11. If C++14 is supported and this flag is used, conflicts are likely to occur.
- `Y3E_NATIVE`   
  Attempts to enable processor-specific optimisations in the compiler which may make the generated executable non-portable but improve performance on the machine on which Y3E is compiled. Suggested if Y3E is to be run on the same machine on which it is compiled.
- `Y3E_LOCAL_RAPIDJSON`   
  Forces Y3E to use its included, local version of rapidjson. If set to "off", and a system installed version of rapidjson is found, the system version will be used instead.
- `Y3E_FORCE_QUIT`   
  Changes the functionality of requesting an exit in the emulator (e.g. pressing escape on a keyboard using the SDL2 backend). If set to "ON", requesting an exit will immediately force the emulator to quit without prompting for confirmation. This is recommended on the Raspberry Pi where it may not be possible to display a confirmation window, leaving no possibility of exiting a full screen instance of Y3E. 
- `Y3E_RPI`   
  Build with Raspberry Pi specfic enhancements for audio

## Dependencies
### Compiler
Y3E is written in C++11 with the inclusion of a single C++14 feature (namely `std::make_unique`). There is a wrapper for `std::make_unique` which allows Y3E to be compiled on any compiler which fully supports C++11.

If the wrapper is required add `-DY3E_MAKE_UNIQUE=ON` as an argument when using CMake to generate project files.

Y3E has been tested successfully with MSVC 2015, g++ 4.8.4+ and clang++ 3.6+.

### Libraries
Dependencies will vary based on which backend/subsystem you want. The SDL2 backend is officially supported and that is the subsystem described below.

Overall the following dependencies are required (assuming you want to build the SDL platform):
- SDL2
- libglm

rapidjson is also an optional dependency. If you don't have it installed, Y3E will use its own provided version. If it's detected however,
the system version is used unless you set the `Y3E_LOCAL_RAPIDJSON` flag when running cmake. 

#### Compiling SDL2
*Note:* These instructions **do not** apply for the Raspberry Pi. See the Raspberry Pi section below for details.

If you need to compile SDL2 manually, download the source from the [official site](https://www.libsdl.org/download-2.0.php).

unzip or untar the file, navigate into the directory then:
```
mkdir build && cd build
../configure
make -j4
sudo make install
```

## Ubuntu/Debian (GNU/Linux)
On ALL distributions run:
```
sudo apt-get install libsdl2-dev
sudo apt-get install libglm-dev
```

After that run the generic build instructions above. If you're on a system with an older version of g++ without support for `std::make_unique` remember to invoke `-DY3E_MAKE_UNIQUE=ON` when running CMake.

## Visual Studio 2015
Other versions of Visual C++ aren't supported officially. SDL2 and Visual Studio 2015 have (an issue)[http://stackoverflow.com/questions/30412951/unresolved-external-symbol-imp-fprintf-and-imp-iob-func-sdl2], a fix for which is compiled in by default when you generate project files with CMake on Windows.

Command line:

`cmake -G "Visual Studio 14 2015 Win64" ..`

You'll probably also need to add -DSDL2_DIR=<path> to help cmake find SDL2. It's **strongly** reccommended that you use the development libraries from the [official SDL2 site](https://www.libsdl.org/download-2.0.php) and not try to compile SDL2 yourself on Windows.

After generation, open Y3E.sln and compile as you normally would on Windows.

## Raspberry Pi 2
### Compiling
The Raspberry Pi 2 is the only model officially supported, although other models may possibly work.

**IMPORTANT:** Make sure that `libudev-dev` is installed before compiling SDL2. If not, input will not work correctly.

Instructions below assume running an up-to-date version of Raspbian Jessie. The SDL2 library provided in the Raspbian repositories was useless when last checked and you'll need to compile manually. [This guide](https://solarianprogrammer.com/2015/01/22/raspberry-pi-raspbian-getting-started-sdl-2/) details the setup.

Other than that, assuming you've installed all the required dependencies including libglm-dev (and optionally rapidjson) you should be able to compile using the generic advice above. It's strongly suggested you compile in *Release* mode for the extra performance boost and use the `-DY3E_NATIVE=ON -DY3E_RPI=ON` flags with CMake.

It's strongly advised that you run CMake with `-DY3E_FORCE_QUIT=ON` on the Pi which will force the program to exit immediately if the escape key is pressed. Otherwise it could be very difficult to close a full-screen Y3E if no system GUI is installed.

### Running
Actually running Y3E after compilation may give the error message `failed to open vchiq instance`. To fix this, run `sudo chmod a+rw /dev/vchiq`.

If you want to get a controller working to play with, follow the [XBox 360 controller setup guide](https://www.raspberrypi.org/forums/viewtopic.php?f=78&t=67379&p=492539#p492539).

[This guide](https://github.com/emscripten-ports/SDL2/blob/master/docs/README-raspberrypi.md) contains details on possible usergroup errors you might have while running on the pi.

### Troubleshooting
If controller/keyboard input doesn't work correctly you probably forgot to install `libudev-dev` before compiling SDL2. Install it and recompile.

# Other Notes
## Installing rapidjson
If you choose to install your own version of rapidjson, it's simple thanks to CMake:
```
git clone https://github.com/miloyip/rapidjson
cd rapidjson
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
