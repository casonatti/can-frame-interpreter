# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jeison/instor/repositories/can-frame-interpreter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jeison/instor/repositories/can-frame-interpreter/build

# Include any dependencies generated for this target.
include CMakeFiles/motor_commands.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/motor_commands.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/motor_commands.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor_commands.dir/flags.make

CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o: CMakeFiles/motor_commands.dir/flags.make
CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o: ../src/motor_commands.cpp
CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o: CMakeFiles/motor_commands.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jeison/instor/repositories/can-frame-interpreter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o -MF CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o.d -o CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o -c /home/jeison/instor/repositories/can-frame-interpreter/src/motor_commands.cpp

CMakeFiles/motor_commands.dir/src/motor_commands.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_commands.dir/src/motor_commands.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jeison/instor/repositories/can-frame-interpreter/src/motor_commands.cpp > CMakeFiles/motor_commands.dir/src/motor_commands.cpp.i

CMakeFiles/motor_commands.dir/src/motor_commands.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_commands.dir/src/motor_commands.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jeison/instor/repositories/can-frame-interpreter/src/motor_commands.cpp -o CMakeFiles/motor_commands.dir/src/motor_commands.cpp.s

# Object files for target motor_commands
motor_commands_OBJECTS = \
"CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o"

# External object files for target motor_commands
motor_commands_EXTERNAL_OBJECTS =

libmotor_commands.a: CMakeFiles/motor_commands.dir/src/motor_commands.cpp.o
libmotor_commands.a: CMakeFiles/motor_commands.dir/build.make
libmotor_commands.a: CMakeFiles/motor_commands.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jeison/instor/repositories/can-frame-interpreter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmotor_commands.a"
	$(CMAKE_COMMAND) -P CMakeFiles/motor_commands.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_commands.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motor_commands.dir/build: libmotor_commands.a
.PHONY : CMakeFiles/motor_commands.dir/build

CMakeFiles/motor_commands.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motor_commands.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motor_commands.dir/clean

CMakeFiles/motor_commands.dir/depend:
	cd /home/jeison/instor/repositories/can-frame-interpreter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jeison/instor/repositories/can-frame-interpreter /home/jeison/instor/repositories/can-frame-interpreter /home/jeison/instor/repositories/can-frame-interpreter/build /home/jeison/instor/repositories/can-frame-interpreter/build /home/jeison/instor/repositories/can-frame-interpreter/build/CMakeFiles/motor_commands.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motor_commands.dir/depend

