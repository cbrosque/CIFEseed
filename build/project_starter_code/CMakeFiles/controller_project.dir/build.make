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
CMAKE_SOURCE_DIR = /home/cbrosque/Documents/apps/construction-robotics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cbrosque/Documents/apps/construction-robotics/build

# Include any dependencies generated for this target.
include project_starter_code/CMakeFiles/controller_project.dir/depend.make

# Include the progress variables for this target.
include project_starter_code/CMakeFiles/controller_project.dir/progress.make

# Include the compile flags for this target's objects.
include project_starter_code/CMakeFiles/controller_project.dir/flags.make

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o: project_starter_code/CMakeFiles/controller_project.dir/flags.make
project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o: ../project_starter_code/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cbrosque/Documents/apps/construction-robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o"
	cd /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_project.dir/controller.cpp.o -c /home/cbrosque/Documents/apps/construction-robotics/project_starter_code/controller.cpp

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_project.dir/controller.cpp.i"
	cd /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cbrosque/Documents/apps/construction-robotics/project_starter_code/controller.cpp > CMakeFiles/controller_project.dir/controller.cpp.i

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_project.dir/controller.cpp.s"
	cd /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cbrosque/Documents/apps/construction-robotics/project_starter_code/controller.cpp -o CMakeFiles/controller_project.dir/controller.cpp.s

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.requires:

.PHONY : project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.requires

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.provides: project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.requires
	$(MAKE) -f project_starter_code/CMakeFiles/controller_project.dir/build.make project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.provides.build
.PHONY : project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.provides

project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.provides.build: project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o


# Object files for target controller_project
controller_project_OBJECTS = \
"CMakeFiles/controller_project.dir/controller.cpp.o"

# External object files for target controller_project
controller_project_EXTERNAL_OBJECTS =

../bin/project_starter_code/controller_project: project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o
../bin/project_starter_code/controller_project: project_starter_code/CMakeFiles/controller_project.dir/build.make
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-common/build/libsai2-common.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGLU.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGL.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-simulation-master/build/libsai2-simulation.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-model/build/libsai2-model.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-model/rbdl/build/librbdl.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-graphics/build/libsai2-graphics.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGLU.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGL.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-primitives/build/libsai2-primitives.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-primitives/../external/ReflexxesTypeII/Linux/x86/release/lib/shared/libReflexxesTypeII.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-common/build/libsai2-common.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGLU.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGL.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-simulation-master/build/libsai2-simulation.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-model/build/libsai2-model.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-model/rbdl/build/librbdl.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-graphics/build/libsai2-graphics.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGLU.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGL.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libhiredis.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libglfw.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-primitives/build/libsai2-primitives.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-primitives/../external/ReflexxesTypeII/Linux/x86/release/lib/shared/libReflexxesTypeII.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-common/build/libsai2-common.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGLU.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGL.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-simulation-master/build/libsai2-simulation.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-model/build/libsai2-model.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-model/rbdl/build/librbdl.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-graphics/build/libsai2-graphics.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/project_starter_code/controller_project: /home/cbrosque/Documents/core/chai3d/build/libchai3d.a
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGLU.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libGL.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libhiredis.so
../bin/project_starter_code/controller_project: /usr/lib/i386-linux-gnu/libglfw.so
../bin/project_starter_code/controller_project: project_starter_code/CMakeFiles/controller_project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cbrosque/Documents/apps/construction-robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/project_starter_code/controller_project"
	cd /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller_project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
project_starter_code/CMakeFiles/controller_project.dir/build: ../bin/project_starter_code/controller_project

.PHONY : project_starter_code/CMakeFiles/controller_project.dir/build

project_starter_code/CMakeFiles/controller_project.dir/requires: project_starter_code/CMakeFiles/controller_project.dir/controller.cpp.o.requires

.PHONY : project_starter_code/CMakeFiles/controller_project.dir/requires

project_starter_code/CMakeFiles/controller_project.dir/clean:
	cd /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code && $(CMAKE_COMMAND) -P CMakeFiles/controller_project.dir/cmake_clean.cmake
.PHONY : project_starter_code/CMakeFiles/controller_project.dir/clean

project_starter_code/CMakeFiles/controller_project.dir/depend:
	cd /home/cbrosque/Documents/apps/construction-robotics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cbrosque/Documents/apps/construction-robotics /home/cbrosque/Documents/apps/construction-robotics/project_starter_code /home/cbrosque/Documents/apps/construction-robotics/build /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code /home/cbrosque/Documents/apps/construction-robotics/build/project_starter_code/CMakeFiles/controller_project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : project_starter_code/CMakeFiles/controller_project.dir/depend

