# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /opt/cmake-3.9.1/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.9.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yxy/Desktop/quad_sim/controllers/quadruped_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build

# Include any dependencies generated for this target.
include CMakeFiles/quadruped_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quadruped_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quadruped_controller.dir/flags.make

CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o: CMakeFiles/quadruped_controller.dir/flags.make
CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o: ../quadruped_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o -c /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/quadruped_controller.cpp

CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.i"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/quadruped_controller.cpp > CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.i

CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.s"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/quadruped_controller.cpp -o CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.s

CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.requires:

.PHONY : CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.requires

CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.provides: CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadruped_controller.dir/build.make CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.provides.build
.PHONY : CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.provides

CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.provides.build: CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o


CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o: CMakeFiles/quadruped_controller.dir/flags.make
CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o: ../webots_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o -c /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/webots_interface.cpp

CMakeFiles/quadruped_controller.dir/webots_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadruped_controller.dir/webots_interface.cpp.i"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/webots_interface.cpp > CMakeFiles/quadruped_controller.dir/webots_interface.cpp.i

CMakeFiles/quadruped_controller.dir/webots_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadruped_controller.dir/webots_interface.cpp.s"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/webots_interface.cpp -o CMakeFiles/quadruped_controller.dir/webots_interface.cpp.s

CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.requires:

.PHONY : CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.requires

CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.provides: CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadruped_controller.dir/build.make CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.provides.build
.PHONY : CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.provides

CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.provides.build: CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o


# Object files for target quadruped_controller
quadruped_controller_OBJECTS = \
"CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o" \
"CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o"

# External object files for target quadruped_controller
quadruped_controller_EXTERNAL_OBJECTS =

quadruped_controller: CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o
quadruped_controller: CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o
quadruped_controller: CMakeFiles/quadruped_controller.dir/build.make
quadruped_controller: CMakeFiles/quadruped_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable quadruped_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadruped_controller.dir/link.txt --verbose=$(VERBOSE)
	/opt/cmake-3.9.1/bin/cmake -E copy /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build/quadruped_controller /home/yxy/Desktop/quad_sim/controllers/quadruped_controller

# Rule to build all files generated by this target.
CMakeFiles/quadruped_controller.dir/build: quadruped_controller

.PHONY : CMakeFiles/quadruped_controller.dir/build

CMakeFiles/quadruped_controller.dir/requires: CMakeFiles/quadruped_controller.dir/quadruped_controller.cpp.o.requires
CMakeFiles/quadruped_controller.dir/requires: CMakeFiles/quadruped_controller.dir/webots_interface.cpp.o.requires

.PHONY : CMakeFiles/quadruped_controller.dir/requires

CMakeFiles/quadruped_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quadruped_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quadruped_controller.dir/clean

CMakeFiles/quadruped_controller.dir/depend:
	cd /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yxy/Desktop/quad_sim/controllers/quadruped_controller /home/yxy/Desktop/quad_sim/controllers/quadruped_controller /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build /home/yxy/Desktop/quad_sim/controllers/quadruped_controller/build/CMakeFiles/quadruped_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quadruped_controller.dir/depend

