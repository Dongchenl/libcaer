# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/dongchen/libcaer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dongchen/libcaer

# Utility rule file for doc.

# Include the progress variables for this target.
include docs/CMakeFiles/doc.dir/progress.make

docs/CMakeFiles/doc: docs/latex/refman.tex
docs/CMakeFiles/doc: docs/html/index.html


docs/latex/refman.tex: docs/libcaer_api.doxy
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/dongchen/libcaer/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating latex/refman.tex, html/index.html"
	cd /home/dongchen/libcaer/include && /usr/bin/doxygen /home/dongchen/libcaer/docs//libcaer_api.doxy

docs/html/index.html: docs/latex/refman.tex
	@$(CMAKE_COMMAND) -E touch_nocreate docs/html/index.html

doc: docs/CMakeFiles/doc
doc: docs/latex/refman.tex
doc: docs/html/index.html
doc: docs/CMakeFiles/doc.dir/build.make

.PHONY : doc

# Rule to build all files generated by this target.
docs/CMakeFiles/doc.dir/build: doc

.PHONY : docs/CMakeFiles/doc.dir/build

docs/CMakeFiles/doc.dir/clean:
	cd /home/dongchen/libcaer/docs && $(CMAKE_COMMAND) -P CMakeFiles/doc.dir/cmake_clean.cmake
.PHONY : docs/CMakeFiles/doc.dir/clean

docs/CMakeFiles/doc.dir/depend:
	cd /home/dongchen/libcaer && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dongchen/libcaer /home/dongchen/libcaer/docs /home/dongchen/libcaer /home/dongchen/libcaer/docs /home/dongchen/libcaer/docs/CMakeFiles/doc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : docs/CMakeFiles/doc.dir/depend
