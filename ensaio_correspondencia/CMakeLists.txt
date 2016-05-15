cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

add_executable (correspondence_grouping_multiview correspondence_grouping_multiview.cpp)
target_link_libraries (correspondence_grouping_multiview ${PCL_LIBRARIES})

#An optional "allclean" target

add_custom_target ( allclean
        COMMAND make clean
        COMMAND rm -rf
        "${PROJECT_SOURCE_DIR}/bin"
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/lib"
        CMakeCache.txt
        CMakeFiles
        cmake_install.cmake
        install_manifest.txt
        Makefile
        COMMAND echo "CLEARED ALL EXCEPT THE SOURCES!"
        )
