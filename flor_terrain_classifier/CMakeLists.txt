cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

set(HEADERS
    include/flor_terrain_classifier/terrain_classifier.h
    include/flor_terrain_classifier/terrain_classifier_node.h
)

set(SOURCES
    src/terrain_classifier.cpp
)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(${PROJECT_NAME} ${SOURCES})
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(terrain_classifier_node src/terrain_classifier_node.cpp ${HEADERS})
rosbuild_add_executable(terrain_classifier_test src/terrain_classifier_test.cpp ${HEADERS})
#target_link_libraries(example ${PROJECT_NAME})
target_link_libraries(terrain_classifier_node ${PROJECT_NAME} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
target_link_libraries(terrain_classifier_test ${PROJECT_NAME} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
