#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import signal
import rospy
import moveit_commander
from PyQt5.QtWidgets import QApplication, QMessageBox

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


def check_gripper_service():
    """그리퍼 서비스 연결 확인"""
    try:
        # 서비스 대기 (짧은 타임아웃)
        rospy.wait_for_service('hande_gripper/control', timeout=2.0)
        return True
    except rospy.ROSException:
        return False

def cleanup_resources(robot_controller=None, gripper_controller=None):
    """프로그램 종료 시 리소스 정리"""
    if robot_controller:
        try:
            robot_controller.cleanup()  # 작업 상자 제거
        except Exception as e:
            rospy.logerr(f"로봇 컨트롤러 정리 중 오류: {e}")
    
    if gripper_controller:
        try:
            gripper_controller.close()
        except Exception as e:
            rospy.logerr(f"그리퍼 컨트롤러 정리 중 오류: {e}")
    
    # ROS 종료
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Application closed")
    
    # MoveIt 종료
    try:
        moveit_commander.roscpp_shutdown()
    except:
        pass


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
    
    # 그리퍼 서비스 확인
    if not check_gripper_service():
        # GUI 경고 창 표시
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText("그리퍼 서비스를 찾을 수 없습니다.")
        msg_box.setInformativeText("hande_driver.py가 실행 중인지 확인하세요. 계속 진행할까요?")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.Yes)
        
        # 사용자 선택에 따라 계속 또는 종료
        if msg_box.exec() == QMessageBox.No:
            sys.exit(0)
    
    try:
        # 로봇 컨트롤러 초기화
        robot_controller = URRobotController()
        
        # 그리퍼 컨트롤러 초기화
        gripper_controller = GripperController(robot_ip)
        
        # GUI 생성 및 표시
        gui = URControlGUI(robot_controller, gripper_controller)
        gui.show()

        # 종료 시 정리 함수 등록
        app.aboutToQuit.connect(lambda: cleanup_resources(robot_controller, gripper_controller))

        # QApplication 이벤트 루프 실행
        sys.exit(app.exec_())
    
    except Exception as e:
        rospy.logerr(f"초기화 오류: {e}")
        cleanup_resources(robot_controller, gripper_controller)
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