# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.27.7/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.27.7/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/juliusarolovitch/search

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/juliusarolovitch/search/build

# Include any dependencies generated for this target.
include CMakeFiles/run_2d_mapf_pp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/run_2d_mapf_pp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/run_2d_mapf_pp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_2d_mapf_pp.dir/flags.make

CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o: CMakeFiles/run_2d_mapf_pp.dir/flags.make
CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o: /Users/juliusarolovitch/search/domains/2d_mapf/run_2d_pp.cpp
CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o: CMakeFiles/run_2d_mapf_pp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/juliusarolovitch/search/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o -MF CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o.d -o CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o -c /Users/juliusarolovitch/search/domains/2d_mapf/run_2d_pp.cpp

CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/juliusarolovitch/search/domains/2d_mapf/run_2d_pp.cpp > CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.i

CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/juliusarolovitch/search/domains/2d_mapf/run_2d_pp.cpp -o CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.s

# Object files for target run_2d_mapf_pp
run_2d_mapf_pp_OBJECTS = \
"CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o"

# External object files for target run_2d_mapf_pp
run_2d_mapf_pp_EXTERNAL_OBJECTS =

run_2d_mapf_pp: CMakeFiles/run_2d_mapf_pp.dir/domains/2d_mapf/run_2d_pp.cpp.o
run_2d_mapf_pp: CMakeFiles/run_2d_mapf_pp.dir/build.make
run_2d_mapf_pp: libsearch.dylib
run_2d_mapf_pp: /opt/homebrew/lib/libboost_system-mt.dylib
run_2d_mapf_pp: /opt/homebrew/lib/libboost_filesystem-mt.dylib
run_2d_mapf_pp: /opt/homebrew/lib/libboost_program_options-mt.dylib
run_2d_mapf_pp: /opt/homebrew/lib/libboost_atomic-mt.dylib
run_2d_mapf_pp: CMakeFiles/run_2d_mapf_pp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/juliusarolovitch/search/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable run_2d_mapf_pp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_2d_mapf_pp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_2d_mapf_pp.dir/build: run_2d_mapf_pp
.PHONY : CMakeFiles/run_2d_mapf_pp.dir/build

CMakeFiles/run_2d_mapf_pp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_2d_mapf_pp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_2d_mapf_pp.dir/clean

CMakeFiles/run_2d_mapf_pp.dir/depend:
	cd /Users/juliusarolovitch/search/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/juliusarolovitch/search /Users/juliusarolovitch/search /Users/juliusarolovitch/search/build /Users/juliusarolovitch/search/build /Users/juliusarolovitch/search/build/CMakeFiles/run_2d_mapf_pp.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/run_2d_mapf_pp.dir/depend

