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
CMAKE_SOURCE_DIR = /home/robot/2.120-project/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/2.120-project/catkin_ws/build

# Include any dependencies generated for this target.
include apriltags/CMakeFiles/apriltags.dir/depend.make

# Include the progress variables for this target.
include apriltags/CMakeFiles/apriltags.dir/progress.make

# Include the compile flags for this target's objects.
include apriltags/CMakeFiles/apriltags.dir/flags.make

apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o: apriltags/CMakeFiles/apriltags.dir/flags.make
apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o: /home/robot/2.120-project/catkin_ws/src/apriltags/src/apriltags.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/2.120-project/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o"
	cd /home/robot/2.120-project/catkin_ws/build/apriltags && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/apriltags.dir/src/apriltags.cpp.o -c /home/robot/2.120-project/catkin_ws/src/apriltags/src/apriltags.cpp

apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/apriltags.dir/src/apriltags.cpp.i"
	cd /home/robot/2.120-project/catkin_ws/build/apriltags && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/2.120-project/catkin_ws/src/apriltags/src/apriltags.cpp > CMakeFiles/apriltags.dir/src/apriltags.cpp.i

apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/apriltags.dir/src/apriltags.cpp.s"
	cd /home/robot/2.120-project/catkin_ws/build/apriltags && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/2.120-project/catkin_ws/src/apriltags/src/apriltags.cpp -o CMakeFiles/apriltags.dir/src/apriltags.cpp.s

apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.requires:

.PHONY : apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.requires

apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.provides: apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.requires
	$(MAKE) -f apriltags/CMakeFiles/apriltags.dir/build.make apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.provides.build
.PHONY : apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.provides

apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.provides.build: apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o


# Object files for target apriltags
apriltags_OBJECTS = \
"CMakeFiles/apriltags.dir/src/apriltags.cpp.o"

# External object files for target apriltags
apriltags_EXTERNAL_OBJECTS =

/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: apriltags/CMakeFiles/apriltags.dir/build.make
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libcv_bridge.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libimage_transport.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libmessage_filters.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libclass_loader.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/libPocoFoundation.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libdl.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libroslib.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libroscpp.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/librosconsole.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/librostime.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libcpp_common.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /usr/local/lib/libapriltagscpp.so
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
/home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags: apriltags/CMakeFiles/apriltags.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/2.120-project/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags"
	cd /home/robot/2.120-project/catkin_ws/build/apriltags && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/apriltags.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apriltags/CMakeFiles/apriltags.dir/build: /home/robot/2.120-project/catkin_ws/devel/lib/apriltags/apriltags

.PHONY : apriltags/CMakeFiles/apriltags.dir/build

apriltags/CMakeFiles/apriltags.dir/requires: apriltags/CMakeFiles/apriltags.dir/src/apriltags.cpp.o.requires

.PHONY : apriltags/CMakeFiles/apriltags.dir/requires

apriltags/CMakeFiles/apriltags.dir/clean:
	cd /home/robot/2.120-project/catkin_ws/build/apriltags && $(CMAKE_COMMAND) -P CMakeFiles/apriltags.dir/cmake_clean.cmake
.PHONY : apriltags/CMakeFiles/apriltags.dir/clean

apriltags/CMakeFiles/apriltags.dir/depend:
	cd /home/robot/2.120-project/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/2.120-project/catkin_ws/src /home/robot/2.120-project/catkin_ws/src/apriltags /home/robot/2.120-project/catkin_ws/build /home/robot/2.120-project/catkin_ws/build/apriltags /home/robot/2.120-project/catkin_ws/build/apriltags/CMakeFiles/apriltags.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apriltags/CMakeFiles/apriltags.dir/depend

