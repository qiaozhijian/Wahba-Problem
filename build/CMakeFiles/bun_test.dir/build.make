# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /home/qzj/Software/clion-2022.3.1/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /home/qzj/Software/clion-2022.3.1/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/qzj/code/Wahba-Problem

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qzj/code/Wahba-Problem/build

# Include any dependencies generated for this target.
include CMakeFiles/bun_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/bun_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/bun_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bun_test.dir/flags.make

CMakeFiles/bun_test.dir/node/bun_test.cpp.o: CMakeFiles/bun_test.dir/flags.make
CMakeFiles/bun_test.dir/node/bun_test.cpp.o: /home/qzj/code/Wahba-Problem/node/bun_test.cpp
CMakeFiles/bun_test.dir/node/bun_test.cpp.o: CMakeFiles/bun_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qzj/code/Wahba-Problem/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bun_test.dir/node/bun_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/bun_test.dir/node/bun_test.cpp.o -MF CMakeFiles/bun_test.dir/node/bun_test.cpp.o.d -o CMakeFiles/bun_test.dir/node/bun_test.cpp.o -c /home/qzj/code/Wahba-Problem/node/bun_test.cpp

CMakeFiles/bun_test.dir/node/bun_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bun_test.dir/node/bun_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qzj/code/Wahba-Problem/node/bun_test.cpp > CMakeFiles/bun_test.dir/node/bun_test.cpp.i

CMakeFiles/bun_test.dir/node/bun_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bun_test.dir/node/bun_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qzj/code/Wahba-Problem/node/bun_test.cpp -o CMakeFiles/bun_test.dir/node/bun_test.cpp.s

# Object files for target bun_test
bun_test_OBJECTS = \
"CMakeFiles/bun_test.dir/node/bun_test.cpp.o"

# External object files for target bun_test
bun_test_EXTERNAL_OBJECTS =

bun_test: CMakeFiles/bun_test.dir/node/bun_test.cpp.o
bun_test: CMakeFiles/bun_test.dir/build.make
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_people.so
bun_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
bun_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bun_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bun_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bun_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
bun_test: /usr/lib/x86_64-linux-gnu/libqhull.so
bun_test: /usr/lib/libOpenNI.so
bun_test: /usr/lib/libOpenNI2.so
bun_test: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libfreetype.so
bun_test: /usr/lib/x86_64-linux-gnu/libz.so
bun_test: /usr/lib/x86_64-linux-gnu/libjpeg.so
bun_test: /usr/lib/x86_64-linux-gnu/libpng.so
bun_test: /usr/lib/x86_64-linux-gnu/libtiff.so
bun_test: /usr/lib/x86_64-linux-gnu/libexpat.so
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_features.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_search.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_io.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
bun_test: /usr/lib/x86_64-linux-gnu/libpcl_common.so
bun_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libfreetype.so
bun_test: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
bun_test: /usr/lib/x86_64-linux-gnu/libz.so
bun_test: /usr/lib/x86_64-linux-gnu/libGLEW.so
bun_test: /usr/lib/x86_64-linux-gnu/libSM.so
bun_test: /usr/lib/x86_64-linux-gnu/libICE.so
bun_test: /usr/lib/x86_64-linux-gnu/libX11.so
bun_test: /usr/lib/x86_64-linux-gnu/libXext.so
bun_test: /usr/lib/x86_64-linux-gnu/libXt.so
bun_test: CMakeFiles/bun_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qzj/code/Wahba-Problem/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bun_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bun_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bun_test.dir/build: bun_test
.PHONY : CMakeFiles/bun_test.dir/build

CMakeFiles/bun_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bun_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bun_test.dir/clean

CMakeFiles/bun_test.dir/depend:
	cd /home/qzj/code/Wahba-Problem/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qzj/code/Wahba-Problem /home/qzj/code/Wahba-Problem /home/qzj/code/Wahba-Problem/build /home/qzj/code/Wahba-Problem/build /home/qzj/code/Wahba-Problem/build/CMakeFiles/bun_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bun_test.dir/depend
