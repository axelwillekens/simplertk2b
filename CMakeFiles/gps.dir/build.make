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
CMAKE_SOURCE_DIR = /home/axelilvo/simplertk2b

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/axelilvo/simplertk2b

# Include any dependencies generated for this target.
include CMakeFiles/gps.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gps.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gps.dir/flags.make

CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o: CMakeFiles/gps.dir/flags.make
CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o: gps/serialcomm/serialComm.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o"
	/usr/bin/gcc-8 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o   -c /home/axelilvo/simplertk2b/gps/serialcomm/serialComm.c

CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.i"
	/usr/bin/gcc-8 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/axelilvo/simplertk2b/gps/serialcomm/serialComm.c > CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.i

CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.s"
	/usr/bin/gcc-8 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/axelilvo/simplertk2b/gps/serialcomm/serialComm.c -o CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.s

CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.requires:

.PHONY : CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.requires

CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.provides: CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.requires
	$(MAKE) -f CMakeFiles/gps.dir/build.make CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.provides.build
.PHONY : CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.provides

CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.provides.build: CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o


CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o: CMakeFiles/gps.dir/flags.make
CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o: gps/ntrip/ntrip.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o"
	/usr/bin/gcc-8 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o   -c /home/axelilvo/simplertk2b/gps/ntrip/ntrip.c

CMakeFiles/gps.dir/gps/ntrip/ntrip.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gps.dir/gps/ntrip/ntrip.c.i"
	/usr/bin/gcc-8 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/axelilvo/simplertk2b/gps/ntrip/ntrip.c > CMakeFiles/gps.dir/gps/ntrip/ntrip.c.i

CMakeFiles/gps.dir/gps/ntrip/ntrip.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gps.dir/gps/ntrip/ntrip.c.s"
	/usr/bin/gcc-8 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/axelilvo/simplertk2b/gps/ntrip/ntrip.c -o CMakeFiles/gps.dir/gps/ntrip/ntrip.c.s

CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.requires:

.PHONY : CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.requires

CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.provides: CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.requires
	$(MAKE) -f CMakeFiles/gps.dir/build.make CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.provides.build
.PHONY : CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.provides

CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.provides.build: CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o


CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o: CMakeFiles/gps.dir/flags.make
CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o: gps/simplertk2b/gganmealine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o"
	/usr/bin/g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o -c /home/axelilvo/simplertk2b/gps/simplertk2b/gganmealine.cpp

CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/axelilvo/simplertk2b/gps/simplertk2b/gganmealine.cpp > CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.i

CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/axelilvo/simplertk2b/gps/simplertk2b/gganmealine.cpp -o CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.s

CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.requires:

.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.requires

CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.provides: CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.requires
	$(MAKE) -f CMakeFiles/gps.dir/build.make CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.provides.build
.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.provides

CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.provides.build: CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o


CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o: CMakeFiles/gps.dir/flags.make
CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o: gps/simplertk2b/nmealine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o"
	/usr/bin/g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o -c /home/axelilvo/simplertk2b/gps/simplertk2b/nmealine.cpp

CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/axelilvo/simplertk2b/gps/simplertk2b/nmealine.cpp > CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.i

CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/axelilvo/simplertk2b/gps/simplertk2b/nmealine.cpp -o CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.s

CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.requires:

.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.requires

CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.provides: CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.requires
	$(MAKE) -f CMakeFiles/gps.dir/build.make CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.provides.build
.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.provides

CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.provides.build: CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o


CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o: CMakeFiles/gps.dir/flags.make
CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o: gps/simplertk2b/rmcnmealine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o"
	/usr/bin/g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o -c /home/axelilvo/simplertk2b/gps/simplertk2b/rmcnmealine.cpp

CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/axelilvo/simplertk2b/gps/simplertk2b/rmcnmealine.cpp > CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.i

CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/axelilvo/simplertk2b/gps/simplertk2b/rmcnmealine.cpp -o CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.s

CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.requires:

.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.requires

CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.provides: CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.requires
	$(MAKE) -f CMakeFiles/gps.dir/build.make CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.provides.build
.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.provides

CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.provides.build: CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o


CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o: CMakeFiles/gps.dir/flags.make
CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o: gps/simplertk2b/simplertk2b.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o"
	/usr/bin/g++-8  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o -c /home/axelilvo/simplertk2b/gps/simplertk2b/simplertk2b.cpp

CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/axelilvo/simplertk2b/gps/simplertk2b/simplertk2b.cpp > CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.i

CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/axelilvo/simplertk2b/gps/simplertk2b/simplertk2b.cpp -o CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.s

CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.requires:

.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.requires

CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.provides: CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.requires
	$(MAKE) -f CMakeFiles/gps.dir/build.make CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.provides.build
.PHONY : CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.provides

CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.provides.build: CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o


# Object files for target gps
gps_OBJECTS = \
"CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o" \
"CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o" \
"CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o" \
"CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o" \
"CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o" \
"CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o"

# External object files for target gps
gps_EXTERNAL_OBJECTS =

libgps.so: CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o
libgps.so: CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o
libgps.so: CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o
libgps.so: CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o
libgps.so: CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o
libgps.so: CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o
libgps.so: CMakeFiles/gps.dir/build.make
libgps.so: CMakeFiles/gps.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/axelilvo/simplertk2b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libgps.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gps.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gps.dir/build: libgps.so

.PHONY : CMakeFiles/gps.dir/build

CMakeFiles/gps.dir/requires: CMakeFiles/gps.dir/gps/serialcomm/serialComm.c.o.requires
CMakeFiles/gps.dir/requires: CMakeFiles/gps.dir/gps/ntrip/ntrip.c.o.requires
CMakeFiles/gps.dir/requires: CMakeFiles/gps.dir/gps/simplertk2b/gganmealine.cpp.o.requires
CMakeFiles/gps.dir/requires: CMakeFiles/gps.dir/gps/simplertk2b/nmealine.cpp.o.requires
CMakeFiles/gps.dir/requires: CMakeFiles/gps.dir/gps/simplertk2b/rmcnmealine.cpp.o.requires
CMakeFiles/gps.dir/requires: CMakeFiles/gps.dir/gps/simplertk2b/simplertk2b.cpp.o.requires

.PHONY : CMakeFiles/gps.dir/requires

CMakeFiles/gps.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gps.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gps.dir/clean

CMakeFiles/gps.dir/depend:
	cd /home/axelilvo/simplertk2b && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/axelilvo/simplertk2b /home/axelilvo/simplertk2b /home/axelilvo/simplertk2b /home/axelilvo/simplertk2b /home/axelilvo/simplertk2b/CMakeFiles/gps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gps.dir/depend

