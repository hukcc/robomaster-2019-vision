# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robomaster/桌面/AddKalman

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robomaster/桌面/AddKalman/build

# Include any dependencies generated for this target.
include src/slover/CMakeFiles/slover.dir/depend.make

# Include the progress variables for this target.
include src/slover/CMakeFiles/slover.dir/progress.make

# Include the compile flags for this target's objects.
include src/slover/CMakeFiles/slover.dir/flags.make

src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o: src/slover/CMakeFiles/slover.dir/flags.make
src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o: ../src/slover/angle_slover.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robomaster/桌面/AddKalman/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slover.dir/angle_slover.cpp.o -c /home/robomaster/桌面/AddKalman/src/slover/angle_slover.cpp

src/slover/CMakeFiles/slover.dir/angle_slover.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slover.dir/angle_slover.cpp.i"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robomaster/桌面/AddKalman/src/slover/angle_slover.cpp > CMakeFiles/slover.dir/angle_slover.cpp.i

src/slover/CMakeFiles/slover.dir/angle_slover.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slover.dir/angle_slover.cpp.s"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robomaster/桌面/AddKalman/src/slover/angle_slover.cpp -o CMakeFiles/slover.dir/angle_slover.cpp.s

src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.requires:

.PHONY : src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.requires

src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.provides: src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.requires
	$(MAKE) -f src/slover/CMakeFiles/slover.dir/build.make src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.provides.build
.PHONY : src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.provides

src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.provides.build: src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o


src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o: src/slover/CMakeFiles/slover.dir/flags.make
src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o: ../src/slover/armor_recorder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robomaster/桌面/AddKalman/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slover.dir/armor_recorder.cpp.o -c /home/robomaster/桌面/AddKalman/src/slover/armor_recorder.cpp

src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slover.dir/armor_recorder.cpp.i"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robomaster/桌面/AddKalman/src/slover/armor_recorder.cpp > CMakeFiles/slover.dir/armor_recorder.cpp.i

src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slover.dir/armor_recorder.cpp.s"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robomaster/桌面/AddKalman/src/slover/armor_recorder.cpp -o CMakeFiles/slover.dir/armor_recorder.cpp.s

src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.requires:

.PHONY : src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.requires

src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.provides: src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.requires
	$(MAKE) -f src/slover/CMakeFiles/slover.dir/build.make src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.provides.build
.PHONY : src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.provides

src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.provides.build: src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o


src/slover/CMakeFiles/slover.dir/kalman.cpp.o: src/slover/CMakeFiles/slover.dir/flags.make
src/slover/CMakeFiles/slover.dir/kalman.cpp.o: ../src/slover/kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robomaster/桌面/AddKalman/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/slover/CMakeFiles/slover.dir/kalman.cpp.o"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slover.dir/kalman.cpp.o -c /home/robomaster/桌面/AddKalman/src/slover/kalman.cpp

src/slover/CMakeFiles/slover.dir/kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slover.dir/kalman.cpp.i"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robomaster/桌面/AddKalman/src/slover/kalman.cpp > CMakeFiles/slover.dir/kalman.cpp.i

src/slover/CMakeFiles/slover.dir/kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slover.dir/kalman.cpp.s"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robomaster/桌面/AddKalman/src/slover/kalman.cpp -o CMakeFiles/slover.dir/kalman.cpp.s

src/slover/CMakeFiles/slover.dir/kalman.cpp.o.requires:

.PHONY : src/slover/CMakeFiles/slover.dir/kalman.cpp.o.requires

src/slover/CMakeFiles/slover.dir/kalman.cpp.o.provides: src/slover/CMakeFiles/slover.dir/kalman.cpp.o.requires
	$(MAKE) -f src/slover/CMakeFiles/slover.dir/build.make src/slover/CMakeFiles/slover.dir/kalman.cpp.o.provides.build
.PHONY : src/slover/CMakeFiles/slover.dir/kalman.cpp.o.provides

src/slover/CMakeFiles/slover.dir/kalman.cpp.o.provides.build: src/slover/CMakeFiles/slover.dir/kalman.cpp.o


# Object files for target slover
slover_OBJECTS = \
"CMakeFiles/slover.dir/angle_slover.cpp.o" \
"CMakeFiles/slover.dir/armor_recorder.cpp.o" \
"CMakeFiles/slover.dir/kalman.cpp.o"

# External object files for target slover
slover_EXTERNAL_OBJECTS =

lib/libslover.a: src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o
lib/libslover.a: src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o
lib/libslover.a: src/slover/CMakeFiles/slover.dir/kalman.cpp.o
lib/libslover.a: src/slover/CMakeFiles/slover.dir/build.make
lib/libslover.a: src/slover/CMakeFiles/slover.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robomaster/桌面/AddKalman/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library ../../lib/libslover.a"
	cd /home/robomaster/桌面/AddKalman/build/src/slover && $(CMAKE_COMMAND) -P CMakeFiles/slover.dir/cmake_clean_target.cmake
	cd /home/robomaster/桌面/AddKalman/build/src/slover && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slover.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/slover/CMakeFiles/slover.dir/build: lib/libslover.a

.PHONY : src/slover/CMakeFiles/slover.dir/build

src/slover/CMakeFiles/slover.dir/requires: src/slover/CMakeFiles/slover.dir/angle_slover.cpp.o.requires
src/slover/CMakeFiles/slover.dir/requires: src/slover/CMakeFiles/slover.dir/armor_recorder.cpp.o.requires
src/slover/CMakeFiles/slover.dir/requires: src/slover/CMakeFiles/slover.dir/kalman.cpp.o.requires

.PHONY : src/slover/CMakeFiles/slover.dir/requires

src/slover/CMakeFiles/slover.dir/clean:
	cd /home/robomaster/桌面/AddKalman/build/src/slover && $(CMAKE_COMMAND) -P CMakeFiles/slover.dir/cmake_clean.cmake
.PHONY : src/slover/CMakeFiles/slover.dir/clean

src/slover/CMakeFiles/slover.dir/depend:
	cd /home/robomaster/桌面/AddKalman/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robomaster/桌面/AddKalman /home/robomaster/桌面/AddKalman/src/slover /home/robomaster/桌面/AddKalman/build /home/robomaster/桌面/AddKalman/build/src/slover /home/robomaster/桌面/AddKalman/build/src/slover/CMakeFiles/slover.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/slover/CMakeFiles/slover.dir/depend

