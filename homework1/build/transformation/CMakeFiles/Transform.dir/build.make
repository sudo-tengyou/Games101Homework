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
CMAKE_SOURCE_DIR = /home/cs18/Desktop/homework/homework1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cs18/Desktop/homework/homework1/build

# Include any dependencies generated for this target.
include transformation/CMakeFiles/Transform.dir/depend.make

# Include the progress variables for this target.
include transformation/CMakeFiles/Transform.dir/progress.make

# Include the compile flags for this target's objects.
include transformation/CMakeFiles/Transform.dir/flags.make

transformation/CMakeFiles/Transform.dir/transform.cpp.o: transformation/CMakeFiles/Transform.dir/flags.make
transformation/CMakeFiles/Transform.dir/transform.cpp.o: ../transformation/transform.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/homework/homework1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object transformation/CMakeFiles/Transform.dir/transform.cpp.o"
	cd /home/cs18/Desktop/homework/homework1/build/transformation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Transform.dir/transform.cpp.o -c /home/cs18/Desktop/homework/homework1/transformation/transform.cpp

transformation/CMakeFiles/Transform.dir/transform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Transform.dir/transform.cpp.i"
	cd /home/cs18/Desktop/homework/homework1/build/transformation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cs18/Desktop/homework/homework1/transformation/transform.cpp > CMakeFiles/Transform.dir/transform.cpp.i

transformation/CMakeFiles/Transform.dir/transform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Transform.dir/transform.cpp.s"
	cd /home/cs18/Desktop/homework/homework1/build/transformation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cs18/Desktop/homework/homework1/transformation/transform.cpp -o CMakeFiles/Transform.dir/transform.cpp.s

transformation/CMakeFiles/Transform.dir/transform.cpp.o.requires:

.PHONY : transformation/CMakeFiles/Transform.dir/transform.cpp.o.requires

transformation/CMakeFiles/Transform.dir/transform.cpp.o.provides: transformation/CMakeFiles/Transform.dir/transform.cpp.o.requires
	$(MAKE) -f transformation/CMakeFiles/Transform.dir/build.make transformation/CMakeFiles/Transform.dir/transform.cpp.o.provides.build
.PHONY : transformation/CMakeFiles/Transform.dir/transform.cpp.o.provides

transformation/CMakeFiles/Transform.dir/transform.cpp.o.provides.build: transformation/CMakeFiles/Transform.dir/transform.cpp.o


# Object files for target Transform
Transform_OBJECTS = \
"CMakeFiles/Transform.dir/transform.cpp.o"

# External object files for target Transform
Transform_EXTERNAL_OBJECTS =

transformation/libTransform.a: transformation/CMakeFiles/Transform.dir/transform.cpp.o
transformation/libTransform.a: transformation/CMakeFiles/Transform.dir/build.make
transformation/libTransform.a: transformation/CMakeFiles/Transform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cs18/Desktop/homework/homework1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libTransform.a"
	cd /home/cs18/Desktop/homework/homework1/build/transformation && $(CMAKE_COMMAND) -P CMakeFiles/Transform.dir/cmake_clean_target.cmake
	cd /home/cs18/Desktop/homework/homework1/build/transformation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Transform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
transformation/CMakeFiles/Transform.dir/build: transformation/libTransform.a

.PHONY : transformation/CMakeFiles/Transform.dir/build

transformation/CMakeFiles/Transform.dir/requires: transformation/CMakeFiles/Transform.dir/transform.cpp.o.requires

.PHONY : transformation/CMakeFiles/Transform.dir/requires

transformation/CMakeFiles/Transform.dir/clean:
	cd /home/cs18/Desktop/homework/homework1/build/transformation && $(CMAKE_COMMAND) -P CMakeFiles/Transform.dir/cmake_clean.cmake
.PHONY : transformation/CMakeFiles/Transform.dir/clean

transformation/CMakeFiles/Transform.dir/depend:
	cd /home/cs18/Desktop/homework/homework1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cs18/Desktop/homework/homework1 /home/cs18/Desktop/homework/homework1/transformation /home/cs18/Desktop/homework/homework1/build /home/cs18/Desktop/homework/homework1/build/transformation /home/cs18/Desktop/homework/homework1/build/transformation/CMakeFiles/Transform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : transformation/CMakeFiles/Transform.dir/depend

