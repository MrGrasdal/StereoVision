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
CMAKE_SOURCE_DIR = /home/stereo/newspin_ws/src/cv_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stereo/newspin_ws/build/cv_camera

# Utility rule file for roslint_cv_camera.

# Include the progress variables for this target.
include CMakeFiles/roslint_cv_camera.dir/progress.make

roslint_cv_camera: CMakeFiles/roslint_cv_camera.dir/build.make
	cd /home/stereo/newspin_ws/src/cv_camera && /opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/cpplint /home/stereo/newspin_ws/src/cv_camera/src/capture.cpp /home/stereo/newspin_ws/src/cv_camera/src/cv_camera_node.cpp /home/stereo/newspin_ws/src/cv_camera/src/driver.cpp /home/stereo/newspin_ws/src/cv_camera/src/cv_camera_nodelet.cpp /home/stereo/newspin_ws/src/cv_camera/test/test_cv_camera_no_yaml.cpp /home/stereo/newspin_ws/src/cv_camera/test/test_cv_camera.cpp /home/stereo/newspin_ws/src/cv_camera/include/cv_camera/capture.h /home/stereo/newspin_ws/src/cv_camera/include/cv_camera/driver.h /home/stereo/newspin_ws/src/cv_camera/include/cv_camera/exception.h
	cd /home/stereo/newspin_ws/src/cv_camera && /opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/cpplint src/capture.cpp src/cv_camera_node.cpp src/cv_camera_nodelet.cpp src/driver.cpp
.PHONY : roslint_cv_camera

# Rule to build all files generated by this target.
CMakeFiles/roslint_cv_camera.dir/build: roslint_cv_camera

.PHONY : CMakeFiles/roslint_cv_camera.dir/build

CMakeFiles/roslint_cv_camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/roslint_cv_camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/roslint_cv_camera.dir/clean

CMakeFiles/roslint_cv_camera.dir/depend:
	cd /home/stereo/newspin_ws/build/cv_camera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stereo/newspin_ws/src/cv_camera /home/stereo/newspin_ws/src/cv_camera /home/stereo/newspin_ws/build/cv_camera /home/stereo/newspin_ws/build/cv_camera /home/stereo/newspin_ws/build/cv_camera/CMakeFiles/roslint_cv_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/roslint_cv_camera.dir/depend

