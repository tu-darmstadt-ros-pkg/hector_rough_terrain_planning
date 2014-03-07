cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# ROS fuerte system install:
#find_package(sbpl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

set(HEADERS
    include/footstep_planner/Footstep.h
    include/footstep_planner/FootstepNavigation.h
    include/footstep_planner/FootstepPlanner.h
    include/footstep_planner/FootstepPlannerEnvironment.h
    include/footstep_planner/FootstepPlannerNode.h
    include/footstep_planner/helper.h
    include/footstep_planner/PlanningState.h
    include/footstep_planner/PlanningStateChangeQuery.h
    include/footstep_planner/State.h
    include/footstep_planner/terrain_model.h
    include/footstep_planner/heuristics/Heuristic.h
    include/footstep_planner/heuristics/PathCostHeuristic.h
    include/footstep_planner/step_cost_estimators/step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/boundary_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/const_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/dynamics_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/euclidean_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/ground_contact_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/gpr_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/map_step_cost_estimator.h
    include/footstep_planner/step_cost_estimators/step_cost_key.h
)

set(FOOTSTEP_PLANNER_FILES
    src/FootstepNavigation.cpp
    src/FootstepPlanner.cpp
    src/FootstepPlannerEnvironment.cpp
    src/FootstepPlannerNode.cpp
    src/Footstep.cpp
    src/PlanningState.cpp
    src/helper.cpp
    src/PlanningStateChangeQuery.cpp
    src/State.cpp
    src/terrain_model.cpp
    src/heuristics/Heuristic.cpp
    src/heuristics/PathCostHeuristic.cpp
    src/step_cost_estimators/step_cost_estimator.cpp
    src/step_cost_estimators/boundary_step_cost_estimator.cpp
    src/step_cost_estimators/const_step_cost_estimator.cpp
    src/step_cost_estimators/dynamics_step_cost_estimator.cpp
    src/step_cost_estimators/euclidean_step_cost_estimator.cpp
    src/step_cost_estimators/ground_contact_step_cost_estimator.cpp
    src/step_cost_estimators/gpr_step_cost_estimator.cpp
    src/step_cost_estimators/map_step_cost_estimator.cpp
    src/step_cost_estimators/step_cost_key.cpp
)

rosbuild_add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES})
rosbuild_add_executable(footstep_planner_node src/footstep_planner.cpp ${HEADERS})
target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${PCL_LIBRARIES} ${SBPL_LIBRARIES})
rosbuild_add_executable(footstep_planner_walls src/footstep_planner_walls.cpp ${HEADERS})
target_link_libraries(footstep_planner_walls ${PROJECT_NAME} ${PCL_LIBRARIES} ${SBPL_LIBRARIES} ${OpenCV_LIBS})
rosbuild_add_executable(footstep_navigation_node src/footstep_navigation.cpp ${HEADERS})
target_link_libraries(footstep_navigation_node ${PROJECT_NAME} ${PCL_LIBRARIES} ${SBPL_LIBRARIES})
