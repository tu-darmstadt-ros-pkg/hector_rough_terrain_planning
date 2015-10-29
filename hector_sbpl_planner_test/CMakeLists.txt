cmake_minimum_required(VERSION 2.8.3)
project(hector_sbpl_planner_test)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hector_sbpl_terrain_planner flor_terrain_classifier)


## System dependencies are found with CMake's conventions

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
    INCLUDE_DIRS  
    LIBRARIES planner_test
    CATKIN_DEPENDS hector_sbpl_terrain_planner flor_terrain_classifier
    DEPENDS
)


set(HEADERS
)

set(SOURCES
    src/planner_test.cpp
)


## Declare a cpp executable
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_executable(planner_test src/planner_test.cpp)


target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )
target_link_libraries(planner_test  ${PROJECT_NAME})



