# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/mike/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/193.6911.21/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/mike/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/193.6911.21/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mike/catkin_ws/src/cpe631_final_with_nav

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug

# Include any dependencies generated for this target.
include gtest/gtest/CMakeFiles/gtest.dir/depend.make

# Include the progress variables for this target.
include gtest/gtest/CMakeFiles/gtest.dir/progress.make

# Include the compile flags for this target's objects.
include gtest/gtest/CMakeFiles/gtest.dir/flags.make

gtest/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: gtest/gtest/CMakeFiles/gtest.dir/flags.make
gtest/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: /usr/src/gtest/src/gtest-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gtest/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o"
	cd /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtest.dir/src/gtest-all.cc.o -c /usr/src/gtest/src/gtest-all.cc

gtest/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest.dir/src/gtest-all.cc.i"
	cd /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/src/gtest/src/gtest-all.cc > CMakeFiles/gtest.dir/src/gtest-all.cc.i

gtest/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest.dir/src/gtest-all.cc.s"
	cd /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/src/gtest/src/gtest-all.cc -o CMakeFiles/gtest.dir/src/gtest-all.cc.s

# Object files for target gtest
gtest_OBJECTS = \
"CMakeFiles/gtest.dir/src/gtest-all.cc.o"

# External object files for target gtest
gtest_EXTERNAL_OBJECTS =

gtest/gtest/libgtest.so: gtest/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o
gtest/gtest/libgtest.so: gtest/gtest/CMakeFiles/gtest.dir/build.make
gtest/gtest/libgtest.so: gtest/gtest/CMakeFiles/gtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libgtest.so"
	cd /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gtest/gtest/CMakeFiles/gtest.dir/build: gtest/gtest/libgtest.so

.PHONY : gtest/gtest/CMakeFiles/gtest.dir/build

gtest/gtest/CMakeFiles/gtest.dir/clean:
	cd /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest && $(CMAKE_COMMAND) -P CMakeFiles/gtest.dir/cmake_clean.cmake
.PHONY : gtest/gtest/CMakeFiles/gtest.dir/clean

gtest/gtest/CMakeFiles/gtest.dir/depend:
	cd /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mike/catkin_ws/src/cpe631_final_with_nav /usr/src/gtest /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest /home/mike/catkin_ws/src/cpe631_final_with_nav/cmake-build-debug/gtest/gtest/CMakeFiles/gtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtest/gtest/CMakeFiles/gtest.dir/depend

