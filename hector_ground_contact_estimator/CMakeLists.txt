cmake_minimum_required(VERSION 2.8.3)
project(hector_ground_contact_estimator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs visualization_msgs pcl_conversions)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  TestModel.msg
  TerrainModel.msg
  TerrainModelRequest.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TerrainModelService.srv
  TestModelService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs visualization_msgs pcl_conversions
    DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(HEADERS
    include/${PROJECT_NAME}/ground_contact_estimator.h
    include/${PROJECT_NAME}/pcl_math_utils.h)

set(SOURCES
    src/ground_contact_estimator_test.cpp
    src/ground_contact_estimator.cpp
    src/pcl_math_utils.cpp
)





## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
#add_executable(terrain_classifier_node src/terrain_classifier_node.cpp)
add_executable(ground_contact_estimator src/ground_contact_estimator_test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)



## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
#target_link_libraries(terrain_classifier_node ${PROJECT_NAME})
target_link_libraries(ground_contact_estimator ${PROJECT_NAME})
