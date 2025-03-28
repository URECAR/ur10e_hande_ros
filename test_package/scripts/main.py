#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import signal
import rospy
import moveit_commander
from PyQt5.QtWidgets import QApplication

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from test_package.robot_controller import URRobotController
from test_package.gripper_controller import GripperController
from test_package.gui import URControlGUI


def init_ros_node():
    """ROS 노드 초기화"""
    if not rospy.core.is_initialized():
        rospy.init_node('ur_gripper_controller', anonymous=True)


def init_moveit():
    """MoveIt 초기화"""
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("MoveIt Commander 초기화 성공")
        return True
    except Exception as e:
        rospy.logerr(f"MoveIt Commander 초기화 실패: {e}")
        return False


def main():
    # QApplication 생성
    app = QApplication(sys.argv)
    
    # SIGINT 핸들러 (Ctrl+C 종료 가능)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # ROS 노드 초기화
    init_ros_node()
    
    # MoveIt 초기화
    if not init_moveit():
        rospy.logerr("MoveIt 초기화 실패, 프로그램을 종료합니다.")
        sys.exit(1)
    
    # 명령줄 인수 처리
    robot_ip = "192.168.56.101"  # 기본 IP (가상 모드)
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    
    try:
        # 로봇 컨트롤러 초기화
        robot_controller = URRobotController()
        
        # 그리퍼 컨트롤러 초기화
        gripper_controller = GripperController(robot_ip)
        
        # GUI 생성 및 표시
        gui = URControlGUI(robot_controller, gripper_controller)
        gui.show()
        
        # QApplication 이벤트 루프 실행
        sys.exit(app.exec_())
    
    except Exception as e:
        rospy.logerr(f"초기화 오류: {e}")
        sys.exit(1)
    
    finally:
        # 종료 전 리소스 정리
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Application closed")
        try:
            moveit_commander.roscpp_shutdown()
        except:
            pass


if __name__ == '__main__':
    main()