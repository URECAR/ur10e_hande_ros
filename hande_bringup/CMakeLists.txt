cmake_minimum_required(VERSION 3.0.2)
project(hande_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

# 메시지 생성
add_message_files(
  FILES
  HandEGripperCommand.msg
  HandEGripperStatus.msg
)

# 서비스 생성
add_service_files(
  FILES
  GripperControl.srv
)

# 의존성 설정
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# 패키지 정보 추가
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 파이썬 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/hande_driver.py
  scripts/gripper_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)