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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src

# Include any dependencies generated for this target.
include CMakeFiles/virtual_scanner.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/virtual_scanner.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/virtual_scanner.dir/flags.make

CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o: CMakeFiles/virtual_scanner.dir/flags.make
CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o: virtual_scanner.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o -c /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src/virtual_scanner.cpp

CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src/virtual_scanner.cpp > CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.i

CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src/virtual_scanner.cpp -o CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.s

CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.requires:
.PHONY : CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.requires

CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.provides: CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.requires
	$(MAKE) -f CMakeFiles/virtual_scanner.dir/build.make CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.provides.build
.PHONY : CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.provides

CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.provides.build: CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o

# Object files for target virtual_scanner
virtual_scanner_OBJECTS = \
"CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o"

# External object files for target virtual_scanner
virtual_scanner_EXTERNAL_OBJECTS =

virtual_scanner: CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o
virtual_scanner: CMakeFiles/virtual_scanner.dir/build.make
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_system.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_thread.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libpthread.so
virtual_scanner: /usr/lib/libpcl_common.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
virtual_scanner: /usr/lib/libpcl_kdtree.so
virtual_scanner: /usr/lib/libpcl_octree.so
virtual_scanner: /usr/lib/libpcl_search.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libqhull.so
virtual_scanner: /usr/lib/libpcl_surface.so
virtual_scanner: /usr/lib/libpcl_sample_consensus.so
virtual_scanner: /usr/lib/libOpenNI.so
virtual_scanner: /usr/lib/libOpenNI2.so
virtual_scanner: /usr/lib/libvtkCommon.so.5.8.0
virtual_scanner: /usr/lib/libvtkFiltering.so.5.8.0
virtual_scanner: /usr/lib/libvtkImaging.so.5.8.0
virtual_scanner: /usr/lib/libvtkGraphics.so.5.8.0
virtual_scanner: /usr/lib/libvtkGenericFiltering.so.5.8.0
virtual_scanner: /usr/lib/libvtkIO.so.5.8.0
virtual_scanner: /usr/lib/libvtkRendering.so.5.8.0
virtual_scanner: /usr/lib/libvtkVolumeRendering.so.5.8.0
virtual_scanner: /usr/lib/libvtkHybrid.so.5.8.0
virtual_scanner: /usr/lib/libvtkWidgets.so.5.8.0
virtual_scanner: /usr/lib/libvtkParallel.so.5.8.0
virtual_scanner: /usr/lib/libvtkInfovis.so.5.8.0
virtual_scanner: /usr/lib/libvtkGeovis.so.5.8.0
virtual_scanner: /usr/lib/libvtkViews.so.5.8.0
virtual_scanner: /usr/lib/libvtkCharts.so.5.8.0
virtual_scanner: /usr/lib/libpcl_io.so
virtual_scanner: /usr/lib/libpcl_filters.so
virtual_scanner: /usr/lib/libpcl_features.so
virtual_scanner: /usr/lib/libpcl_keypoints.so
virtual_scanner: /usr/lib/libpcl_registration.so
virtual_scanner: /usr/lib/libpcl_segmentation.so
virtual_scanner: /usr/lib/libpcl_recognition.so
virtual_scanner: /usr/lib/libpcl_visualization.so
virtual_scanner: /usr/lib/libpcl_people.so
virtual_scanner: /usr/lib/libpcl_outofcore.so
virtual_scanner: /usr/lib/libpcl_tracking.so
virtual_scanner: /usr/lib/libpcl_apps.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_system.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_thread.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libpthread.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libqhull.so
virtual_scanner: /usr/lib/libOpenNI.so
virtual_scanner: /usr/lib/libOpenNI2.so
virtual_scanner: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
virtual_scanner: /usr/lib/libvtkCommon.so.5.8.0
virtual_scanner: /usr/lib/libvtkFiltering.so.5.8.0
virtual_scanner: /usr/lib/libvtkImaging.so.5.8.0
virtual_scanner: /usr/lib/libvtkGraphics.so.5.8.0
virtual_scanner: /usr/lib/libvtkGenericFiltering.so.5.8.0
virtual_scanner: /usr/lib/libvtkIO.so.5.8.0
virtual_scanner: /usr/lib/libvtkRendering.so.5.8.0
virtual_scanner: /usr/lib/libvtkVolumeRendering.so.5.8.0
virtual_scanner: /usr/lib/libvtkHybrid.so.5.8.0
virtual_scanner: /usr/lib/libvtkWidgets.so.5.8.0
virtual_scanner: /usr/lib/libvtkParallel.so.5.8.0
virtual_scanner: /usr/lib/libvtkInfovis.so.5.8.0
virtual_scanner: /usr/lib/libvtkGeovis.so.5.8.0
virtual_scanner: /usr/lib/libvtkViews.so.5.8.0
virtual_scanner: /usr/lib/libvtkCharts.so.5.8.0
virtual_scanner: /usr/lib/libpcl_common.so
virtual_scanner: /usr/lib/libpcl_kdtree.so
virtual_scanner: /usr/lib/libpcl_octree.so
virtual_scanner: /usr/lib/libpcl_search.so
virtual_scanner: /usr/lib/libpcl_surface.so
virtual_scanner: /usr/lib/libpcl_sample_consensus.so
virtual_scanner: /usr/lib/libpcl_io.so
virtual_scanner: /usr/lib/libpcl_filters.so
virtual_scanner: /usr/lib/libpcl_features.so
virtual_scanner: /usr/lib/libpcl_keypoints.so
virtual_scanner: /usr/lib/libpcl_registration.so
virtual_scanner: /usr/lib/libpcl_segmentation.so
virtual_scanner: /usr/lib/libpcl_recognition.so
virtual_scanner: /usr/lib/libpcl_visualization.so
virtual_scanner: /usr/lib/libpcl_people.so
virtual_scanner: /usr/lib/libpcl_outofcore.so
virtual_scanner: /usr/lib/libpcl_tracking.so
virtual_scanner: /usr/lib/libpcl_apps.so
virtual_scanner: /usr/lib/libvtkViews.so.5.8.0
virtual_scanner: /usr/lib/libvtkInfovis.so.5.8.0
virtual_scanner: /usr/lib/libvtkWidgets.so.5.8.0
virtual_scanner: /usr/lib/libvtkVolumeRendering.so.5.8.0
virtual_scanner: /usr/lib/libvtkHybrid.so.5.8.0
virtual_scanner: /usr/lib/libvtkParallel.so.5.8.0
virtual_scanner: /usr/lib/libvtkRendering.so.5.8.0
virtual_scanner: /usr/lib/libvtkImaging.so.5.8.0
virtual_scanner: /usr/lib/libvtkGraphics.so.5.8.0
virtual_scanner: /usr/lib/libvtkIO.so.5.8.0
virtual_scanner: /usr/lib/libvtkFiltering.so.5.8.0
virtual_scanner: /usr/lib/libvtkCommon.so.5.8.0
virtual_scanner: /usr/lib/libvtksys.so.5.8.0
virtual_scanner: CMakeFiles/virtual_scanner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable virtual_scanner"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/virtual_scanner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/virtual_scanner.dir/build: virtual_scanner
.PHONY : CMakeFiles/virtual_scanner.dir/build

CMakeFiles/virtual_scanner.dir/requires: CMakeFiles/virtual_scanner.dir/virtual_scanner.cpp.o.requires
.PHONY : CMakeFiles/virtual_scanner.dir/requires

CMakeFiles/virtual_scanner.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/virtual_scanner.dir/cmake_clean.cmake
.PHONY : CMakeFiles/virtual_scanner.dir/clean

CMakeFiles/virtual_scanner.dir/depend:
	cd /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src /home/rodrigosalgueiro/Dissertacao/gitEnsaioCorresp/BinPicking/my_virtual_scanner/src/CMakeFiles/virtual_scanner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/virtual_scanner.dir/depend

