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
include CMakeFiles/testFalkoAHT.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testFalkoAHT.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testFalkoAHT.dir/flags.make

CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o: CMakeFiles/testFalkoAHT.dir/flags.make
CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o: ../test/testFalkoAHT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o -c /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testFalkoAHT.cpp

CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testFalkoAHT.cpp > CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.i

CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testFalkoAHT.cpp -o CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.s

CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.requires:

.PHONY : CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.requires

CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.provides: CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.requires
	$(MAKE) -f CMakeFiles/testFalkoAHT.dir/build.make CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.provides.build
.PHONY : CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.provides

CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.provides.build: CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o


CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o: CMakeFiles/testFalkoAHT.dir/flags.make
CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o: ../test/testData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o -c /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testData.cpp

CMakeFiles/testFalkoAHT.dir/test/testData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testFalkoAHT.dir/test/testData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testData.cpp > CMakeFiles/testFalkoAHT.dir/test/testData.cpp.i

CMakeFiles/testFalkoAHT.dir/test/testData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testFalkoAHT.dir/test/testData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/test/testData.cpp -o CMakeFiles/testFalkoAHT.dir/test/testData.cpp.s

CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.requires:

.PHONY : CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.requires

CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.provides: CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.requires
	$(MAKE) -f CMakeFiles/testFalkoAHT.dir/build.make CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.provides.build
.PHONY : CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.provides

CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.provides.build: CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o


# Object files for target testFalkoAHT
testFalkoAHT_OBJECTS = \
"CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o" \
"CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o"

# External object files for target testFalkoAHT
testFalkoAHT_EXTERNAL_OBJECTS =

../bin/testFalkoAHT: CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o
../bin/testFalkoAHT: CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o
../bin/testFalkoAHT: CMakeFiles/testFalkoAHT.dir/build.make
../bin/testFalkoAHT: ../lib/libfalkolib.a
../bin/testFalkoAHT: CMakeFiles/testFalkoAHT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/testFalkoAHT"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testFalkoAHT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testFalkoAHT.dir/build: ../bin/testFalkoAHT

.PHONY : CMakeFiles/testFalkoAHT.dir/build

CMakeFiles/testFalkoAHT.dir/requires: CMakeFiles/testFalkoAHT.dir/test/testFalkoAHT.cpp.o.requires
CMakeFiles/testFalkoAHT.dir/requires: CMakeFiles/testFalkoAHT.dir/test/testData.cpp.o.requires

.PHONY : CMakeFiles/testFalkoAHT.dir/requires

CMakeFiles/testFalkoAHT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testFalkoAHT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testFalkoAHT.dir/clean

CMakeFiles/testFalkoAHT.dir/depend:
	cd /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build /home/nate/workspaces/ov_loc_ws/src/ov_loc/openslam_falkolib/build/CMakeFiles/testFalkoAHT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testFalkoAHT.dir/depend
