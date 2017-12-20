cmake_minimum_required(VERSION 2.8.3)
project(behavior_go_to_point_angle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_definitions(-std=c++11)

# Directories definition
set(BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_DIR
  src/source
)

set(BEHAVIOR_GO_TO_POINT_ANGLE_INCLUDE_DIR
  src/include
)

# Files declaration
set(BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_FILES
  ${BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_DIR}/behavior_go_to_point_angle.cpp
  ${BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_DIR}/behavior_go_to_point_angle.cpp
)

set(BEHAVIOR_GO_TO_POINT_ANGLE_HEADER_FILES
  ${BEHAVIOR_GO_TO_POINT_ANGLE_INCLUDE_DIR}/behavior_go_to_point_angle.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  behavior_process
  droneMsgsROS
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS ${BEHAVIOR_GO_TO_POINT_INCLUDE_DIR}
    CATKIN_DEPENDS
    roscpp
    std_msgs
    drone_process
    behavior_process
    droneMsgsROS
    DEPENDS yaml-cpp
)

###########
## Build ##
###########
include_directories(
    ${BEHAVIOR_GO_TO_POINT_ANGLE_INCLUDE_DIR}
    ${BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_DIR}
)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(behavior_go_to_point_angle_lib ${BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_FILES} ${BEHAVIOR_GO_TO_POINT_ANGLE_HEADER_FILES})
add_dependencies(behavior_go_to_point_angle_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_go_to_point_angle_lib ${catkin_LIBRARIES})
target_link_libraries(behavior_go_to_point_angle_lib yaml-cpp)


add_executable(behavior_go_to_point_angle ${BEHAVIOR_GO_TO_POINT_ANGLE_SOURCE_DIR}/behavior_go_to_point_angle_main.cpp)
add_dependencies(behavior_go_to_point_angle ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_go_to_point_angle behavior_go_to_point_angle_lib)
target_link_libraries(behavior_go_to_point_angle ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_behavior_go_to_point_angle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
