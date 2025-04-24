cmake_minimum_required(VERSION 3.0.2)
project(robotiq_hande_ros_driver)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
)

## Generate messages
add_message_files(
  FILES
  HandECommand.msg
  HandEStatus.msg
)

## Generate services
add_service_files(
  FILES
  HandEGrip.srv
  HandERelease.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  LIBRARIES robotiq_hande_driver robotiq_hande_client
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_runtime
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Move headers to include directory
file(GLOB HEADER_FILES
  src/*.h
)
file(COPY ${HEADER_FILES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Library targets
add_library(robotiq_hande_driver src/robotiq_hande_driver.cpp)
add_dependencies(robotiq_hande_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotiq_hande_driver ${catkin_LIBRARIES})

add_library(robotiq_hande_client src/robotiq_hande_client.cpp)
add_dependencies(robotiq_hande_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotiq_hande_client ${catkin_LIBRARIES})

# Node target
add_executable(robotiq_hande_driver_node src/robotiq_hande_driver_node.cpp)
add_dependencies(robotiq_hande_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotiq_hande_driver_node robotiq_hande_driver ${catkin_LIBRARIES})

## Install
install(TARGETS robotiq_hande_driver robotiq_hande_client robotiq_hande_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)