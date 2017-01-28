PROJECT(camera_client CXX)

# 2.8+ needed for find_package.
cmake_minimum_required(VERSION 2.8)

# Add compiler flags to turn on (most) warnings and to use whichever aspects of
# the C++14 standard have so far been implemented.
if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-Wall --std=c++1y)
  endif(CMAKE_COMPILER_IS_GNUCXX)
endif(UNIX)

# A note on find_package():
#
# This is one of CMake's killer features.  More often than not these days, for
# a given library libFoo, a Linux distro's C or C++ development library
# package (often called "libFoo-devel" or "libFoo-dev") will include a file
# called "FindFoo.cmake", "FooConfig.cmake", or the like.  You can confirm
# this yourself using your distro's package search capabilities.
#
#   Debian, Ubuntu, and derivatives:
#     # You may need to sudo apt-get install apt-file and sudo apt=-file update first.
#     apt-file search --regexp ".*Foo.*cmake"
#   Mageialinux:
#     urpmf -v ".*Foo.*cmake"
#   Cygwin:
#     cygcheck --package-query ".*Foo.*cmake" (don't remember if this uses regexes or not)
#   ArchLinux:
#     pkgfile -s ".*Foo.*cmake" (does this use regular expressions?)
#
# Install whatever package contains that file, and then this find_package()
# directive should be able to find the compiling and pkgconfig instructions
# therein and link to your distro's copy of the OpenCV library...or any other
# one.

# We use CONFIG mode here because we don't have a local FindOpenCV.cmake
# (i.e., MODULE mode) and we're relying on the distro. For more information,
# see https://cmake.org/cmake/help/v3.0/command/find_package.html.
#
# Remember that the first argument to find_package() is case-sensitive on
# Unix.

# Cygwin doesn't seem to automatically set CXX to the GNU C++ compiler,
# and since g++ is our official compiler anyway, I don't have a problem
# with devoting a little bit of space in CMakeLists.txt to giving 
# Cygwin a Clue(TM).
set (CXX "g++")

find_package(OpenCV 2.4 CONFIG REQUIRED)
message ("Locally-detected OpenCV version: ${OpenCV_VERSION}")

# We use Boost for the INI reader built into boost::program_options.
find_package(Boost COMPONENTS program_options REQUIRED)

# Where can we find our headers?
include_directories("./include")

# And where can we find our binaries?
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "./lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "./bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "./lib")

# C++ source files for the main executable (implementation files only, not
# header files.)
set(robot_program_src src/main.cpp
                      src/Config.cpp
                      src/RemoteTransmitter.cpp
                      src/PapasVision.cpp)

# Binary targets.
add_executable(camera-client ${robot_program_src})
target_link_libraries(camera-client ${OpenCV_LIBS}
                                    ${Boost_PROGRAM_OPTIONS_LIBRARY})