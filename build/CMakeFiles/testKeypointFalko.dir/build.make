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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build

# Include any dependencies generated for this target.
include CMakeFiles/testKeypointFalko.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testKeypointFalko.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testKeypointFalko.dir/flags.make

CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o: CMakeFiles/testKeypointFalko.dir/flags.make
CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o: ../test/testKeypointFalko.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o -c /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testKeypointFalko.cpp

CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testKeypointFalko.cpp > CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.i

CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testKeypointFalko.cpp -o CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.s

CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.requires:

.PHONY : CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.requires

CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.provides: CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.requires
	$(MAKE) -f CMakeFiles/testKeypointFalko.dir/build.make CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.provides.build
.PHONY : CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.provides

CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.provides.build: CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o


CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o: CMakeFiles/testKeypointFalko.dir/flags.make
CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o: ../test/testData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o -c /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testData.cpp

CMakeFiles/testKeypointFalko.dir/test/testData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testKeypointFalko.dir/test/testData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testData.cpp > CMakeFiles/testKeypointFalko.dir/test/testData.cpp.i

CMakeFiles/testKeypointFalko.dir/test/testData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testKeypointFalko.dir/test/testData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testData.cpp -o CMakeFiles/testKeypointFalko.dir/test/testData.cpp.s

CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.requires:

.PHONY : CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.requires

CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.provides: CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.requires
	$(MAKE) -f CMakeFiles/testKeypointFalko.dir/build.make CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.provides.build
.PHONY : CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.provides

CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.provides.build: CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o


# Object files for target testKeypointFalko
testKeypointFalko_OBJECTS = \
"CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o" \
"CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o"

# External object files for target testKeypointFalko
testKeypointFalko_EXTERNAL_OBJECTS =

../bin/testKeypointFalko: CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o
../bin/testKeypointFalko: CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o
../bin/testKeypointFalko: CMakeFiles/testKeypointFalko.dir/build.make
../bin/testKeypointFalko: ../lib/libfalkolib.a
../bin/testKeypointFalko: CMakeFiles/testKeypointFalko.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/testKeypointFalko"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testKeypointFalko.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testKeypointFalko.dir/build: ../bin/testKeypointFalko

.PHONY : CMakeFiles/testKeypointFalko.dir/build

CMakeFiles/testKeypointFalko.dir/requires: CMakeFiles/testKeypointFalko.dir/test/testKeypointFalko.cpp.o.requires
CMakeFiles/testKeypointFalko.dir/requires: CMakeFiles/testKeypointFalko.dir/test/testData.cpp.o.requires

.PHONY : CMakeFiles/testKeypointFalko.dir/requires

CMakeFiles/testKeypointFalko.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testKeypointFalko.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testKeypointFalko.dir/clean

CMakeFiles/testKeypointFalko.dir/depend:
	cd /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles/testKeypointFalko.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testKeypointFalko.dir/depend
