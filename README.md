# 2017bot
Code for ESHS FRC 2017 Steamworks competition.

## Build instructions
### C++

The C++ client has the following build dependencies (note that the names may
be slightly different for your distro):

* cmake
* make
* opencv-dev
* libboost-devel (or libboost-program-options-dev if you want to be specific)

On Linux, the installation of these packages, or their distro-specific namesakes,
is simple.  A Windows build is possible with Cygwin--we've done it before--but
it's a bit dicey.

1. Download the Cygwin installer from http://cygwin.com (or Chocolatey).
2. Run the installer, called setup-x86_64.exe for the vanilla installer or cygwinsetup.exe if installed through Chocolatey.
3. Cygwin's installer will automatically install in C:\Cygwin64 and select its base packages the first time it is run.  Cygwin also packages cmake, libboost-devel, and libboost_program_options1.6.0, so select those.
4. Cygwin does _not_ package OpenCV.  For that, you will run the Cygwin installer a second time, keeping the same installation directory from before, and point Cygwin to the CygwinPorts repository (instructions for doing this are at http://cygwinports.com/.)

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
