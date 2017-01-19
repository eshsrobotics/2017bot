# 2017bot
Code for ESHS FRC 2017 Steamworks competition.

## Build instructions
### C++

We use the CMake tool to generate our C++ Makefiles.  If you have new header
or include files to add, modify CMakeLists.txt to add them.  It's
well-commented, and the process should be straightforward.

To build, run these commands:

 ccmake .  # Should only be necessary once.
 cmake .   # Generates or regenerates the Makefile.  (It can also generate IDE build files.)
 make      # Does the actual build.

The binary will be placed in bin/camera-client (bin/camera-client.exe on
Windows), and can be run without arguments.

## Dev Tools
### Build & Deploy
* [Using Eclipse](http://wpilib.screenstepslive.com/s/4485/m/13809/l/242586-building-and-downloading-a-robot-project-to-the-roborio)(Official)
* [GradleRio](https://github.com/Open-RIO/GradleRIO) can be used to build and deploy code to the roboRIO without using eclipse
