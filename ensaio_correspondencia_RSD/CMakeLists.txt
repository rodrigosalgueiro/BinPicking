cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(correspondence_grouping_RSD)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (correspondencia_RSD correspondencia_RSD.cpp)
target_link_libraries (correspondencia_RSD ${PCL_LIBRARIES})

add_executable (correspondencia_multivista_RSD correspondencia_multivista_RSD.cpp)
target_link_libraries (correspondencia_multivista_RSD ${PCL_LIBRARIES})

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
