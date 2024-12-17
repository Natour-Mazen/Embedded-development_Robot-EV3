# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/src/svtr_shared

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/src/svtr_shared

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/src/svtr_shared/CMakeFiles /home/src/svtr_shared/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/src/svtr_shared/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named TP_Robot

# Build rule for target.
TP_Robot: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 TP_Robot
.PHONY : TP_Robot

# fast build rule for target.
TP_Robot/fast:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/build
.PHONY : TP_Robot/fast

communication.o: communication.c.o

.PHONY : communication.o

# target to build an object file
communication.c.o:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/communication.c.o
.PHONY : communication.c.o

communication.i: communication.c.i

.PHONY : communication.i

# target to preprocess a source file
communication.c.i:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/communication.c.i
.PHONY : communication.c.i

communication.s: communication.c.s

.PHONY : communication.s

# target to generate assembly for a file
communication.c.s:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/communication.c.s
.PHONY : communication.c.s

main.o: main.c.o

.PHONY : main.o

# target to build an object file
main.c.o:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/main.c.o
.PHONY : main.c.o

main.i: main.c.i

.PHONY : main.i

# target to preprocess a source file
main.c.i:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/main.c.i
.PHONY : main.c.i

main.s: main.c.s

.PHONY : main.s

# target to generate assembly for a file
main.c.s:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/main.c.s
.PHONY : main.c.s

mdd.o: mdd.c.o

.PHONY : mdd.o

# target to build an object file
mdd.c.o:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/mdd.c.o
.PHONY : mdd.c.o

mdd.i: mdd.c.i

.PHONY : mdd.i

# target to preprocess a source file
mdd.c.i:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/mdd.c.i
.PHONY : mdd.c.i

mdd.s: mdd.c.s

.PHONY : mdd.s

# target to generate assembly for a file
mdd.c.s:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/mdd.c.s
.PHONY : mdd.c.s

myev3.o: myev3.c.o

.PHONY : myev3.o

# target to build an object file
myev3.c.o:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/myev3.c.o
.PHONY : myev3.c.o

myev3.i: myev3.c.i

.PHONY : myev3.i

# target to preprocess a source file
myev3.c.i:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/myev3.c.i
.PHONY : myev3.c.i

myev3.s: myev3.c.s

.PHONY : myev3.s

# target to generate assembly for a file
myev3.c.s:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/myev3.c.s
.PHONY : myev3.c.s

time_util.o: time_util.c.o

.PHONY : time_util.o

# target to build an object file
time_util.c.o:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/time_util.c.o
.PHONY : time_util.c.o

time_util.i: time_util.c.i

.PHONY : time_util.i

# target to preprocess a source file
time_util.c.i:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/time_util.c.i
.PHONY : time_util.c.i

time_util.s: time_util.c.s

.PHONY : time_util.s

# target to generate assembly for a file
time_util.c.s:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/time_util.c.s
.PHONY : time_util.c.s

workers.o: workers.c.o

.PHONY : workers.o

# target to build an object file
workers.c.o:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/workers.c.o
.PHONY : workers.c.o

workers.i: workers.c.i

.PHONY : workers.i

# target to preprocess a source file
workers.c.i:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/workers.c.i
.PHONY : workers.c.i

workers.s: workers.c.s

.PHONY : workers.s

# target to generate assembly for a file
workers.c.s:
	$(MAKE) -f CMakeFiles/TP_Robot.dir/build.make CMakeFiles/TP_Robot.dir/workers.c.s
.PHONY : workers.c.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... TP_Robot"
	@echo "... communication.o"
	@echo "... communication.i"
	@echo "... communication.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... mdd.o"
	@echo "... mdd.i"
	@echo "... mdd.s"
	@echo "... myev3.o"
	@echo "... myev3.i"
	@echo "... myev3.s"
	@echo "... time_util.o"
	@echo "... time_util.i"
	@echo "... time_util.s"
	@echo "... workers.o"
	@echo "... workers.i"
	@echo "... workers.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

