#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import signal
import rospy
import moveit_commander
from PyQt5.QtWidgets import QApplication, QMessageBox

# 경로 설정
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))

# 필요한 모듈 임포트
from test_package.robot_controller import URRobotController
from test_package.gripper_controller import GripperController
from test_package.pose_manager import PoseManager
from test_package.gui import URControlGUI

# 소켓 관련 모듈 임포트
from test_package.socket_manager import SocketManager
from test_package.socket_tab import SocketTab


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


def check_camera_topics():
    """카메라 토픽 확인"""
    topics = ['/camera/color/image_raw', '/camera/depth/image_rect_raw']
    # 잠시 대기 (토픽 검색 시간)
    rospy.sleep(0.5)
    
    # 실제 토픽 목록 가져오기
    published_topics = dict(rospy.get_published_topics())
    
    # 필요한 토픽이 있는지 확인
    missing_topics = [t for t in topics if t not in published_topics]
    
    if missing_topics:
        rospy.logwarn(f"카메라 토픽을 찾을 수 없습니다: {', '.join(missing_topics)}")
        return False
    
    return True


def cleanup_resources(robot_controller=None, gripper_controller=None, socket_manager=None):
    """프로그램 종료 시 리소스 정리"""
    if socket_manager:
        try:
            socket_manager.stop_server()
        except Exception as e:
            rospy.logerr(f"소켓 매니저 정리 중 오류: {e}")
    
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
        msg_box.setText("그리퍼 서비스를: /hande_gripper/control'를 찾을 수 없습니다.")
        msg_box.setInformativeText("hande_driver.py가 실행 중인지 확인하세요. 계속 진행할까요?")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.Yes)
        
        # 사용자 선택에 따라 계속 또는 종료
        if msg_box.exec() == QMessageBox.No:
            sys.exit(0)
    
    # 카메라 토픽 확인
    if not check_camera_topics():
        # GUI 경고 창 표시
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText("카메라 관련 토픽을 찾을 수 없습니다.")
        msg_box.setInformativeText("Realsense 카메라 노드가 실행 중인지 확인하세요.\n카메라 기능을 사용하지 않고 계속할까요?")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.Yes)
        
        # 사용자 선택에 따라 계속 또는 종료
        if msg_box.exec() == QMessageBox.No:
            sys.exit(0)
    
    try:
        # 로봇 컨트롤러 초기화
        robot_controller = URRobotController()
        robot_controller.add_box('workspace_box', [0, 0, 0], [0.795, 0.6, 1.0], center=True)
        robot_controller.add_box('workspace_box2', [0.6, 0.066, 0.151], [0.50, 0.50, 1.151], center=True)

        # 그리퍼 컨트롤러 초기화
        gripper_controller = GripperController(robot_ip)
        
        # 소켓 매니저 초기화
        socket_manager = SocketManager(robot_controller, gripper_controller)
        
        # GUI 생성 (소켓 매니저 전달)
        gui = URControlGUI(robot_controller, gripper_controller, socket_manager)
        
        # 소켓 탭 생성 및 추가
        socket_tab = SocketTab(socket_manager)
        gui.tabs.addTab(socket_tab, "소켓 통신")
        
        # GUI 표시
        gui.show()

        # 종료 시 정리 함수 등록
        app.aboutToQuit.connect(lambda: cleanup_resources(robot_controller, gripper_controller, socket_manager))

        # QApplication 이벤트 루프 실행
        sys.exit(app.exec_())
    
    except Exception as e:
        rospy.logerr(f"초기화 오류: {e}")
        cleanup_resources()
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