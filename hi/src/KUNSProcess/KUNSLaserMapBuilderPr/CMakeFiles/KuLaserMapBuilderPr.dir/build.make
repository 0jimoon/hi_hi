# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/minkuk/KUNS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/minkuk/KUNS

# Include any dependencies generated for this target.
include src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/depend.make

# Include the progress variables for this target.
include src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/progress.make

# Include the compile flags for this target's objects.
include src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/flags.make

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/flags.make
src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o: src/KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/minkuk/KUNS/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o -c /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.cpp

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.i"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.cpp > CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.i

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.s"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.cpp -o CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.s

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.requires:
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.requires

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.provides: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.requires
	$(MAKE) -f src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/build.make src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.provides.build
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.provides

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.provides.build: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/flags.make
src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o: src/KUNSProcess/KUNSLaserMapBuilderPr/KuMapBuilderParameter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/minkuk/KUNS/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o -c /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/KuMapBuilderParameter.cpp

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.i"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/KuMapBuilderParameter.cpp > CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.i

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.s"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/KuMapBuilderParameter.cpp -o CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.s

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.requires:
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.requires

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.provides: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.requires
	$(MAKE) -f src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/build.make src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.provides.build
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.provides

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.provides.build: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o

# Object files for target KuLaserMapBuilderPr
KuLaserMapBuilderPr_OBJECTS = \
"CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o" \
"CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o"

# External object files for target KuLaserMapBuilderPr
KuLaserMapBuilderPr_EXTERNAL_OBJECTS =

lib/libKuLaserMapBuilderPr.a: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o
lib/libKuLaserMapBuilderPr.a: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o
lib/libKuLaserMapBuilderPr.a: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/build.make
lib/libKuLaserMapBuilderPr.a: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../../lib/libKuLaserMapBuilderPr.a"
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && $(CMAKE_COMMAND) -P CMakeFiles/KuLaserMapBuilderPr.dir/cmake_clean_target.cmake
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/KuLaserMapBuilderPr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/build: lib/libKuLaserMapBuilderPr.a
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/build

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/requires: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuLaserMapBuilderPr.o.requires
src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/requires: src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/KuMapBuilderParameter.o.requires
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/requires

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/clean:
	cd /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr && $(CMAKE_COMMAND) -P CMakeFiles/KuLaserMapBuilderPr.dir/cmake_clean.cmake
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/clean

src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/depend:
	cd /home/minkuk/KUNS && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/minkuk/KUNS /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr /home/minkuk/KUNS /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr /home/minkuk/KUNS/src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/KUNSProcess/KUNSLaserMapBuilderPr/CMakeFiles/KuLaserMapBuilderPr.dir/depend
