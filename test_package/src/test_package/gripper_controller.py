#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from hande_bringup.srv import GripperControl, GripperControlRequest

class GripperController(QObject):
    """그리퍼 제어 및 상태 모니터링 클래스"""
    status_updated = pyqtSignal(dict)  # 그리퍼 상태 업데이트
    
    def __init__(self, ip_address="192.168.56.101"):
        super().__init__()
        
        # 그리퍼 모드 (IP 주소는 현재 사용하지 않음)
        self.virtual_mode = (ip_address == "192.168.56.101")
        rospy.loginfo(f"그리퍼 컨트롤러 초기화 - {'가상' if self.virtual_mode else '실제'} 모드")
        
        # 상태 변수
        self.width = 0.0       # 그리퍼 너비 (미터)
        self.position = 0.0    # 위치 (0.0-1.0)
        self.force = 255       # 힘 (0-255)
        self.speed = 255       # 속도 (0-255)
        self.moving = False    # 이동 중 여부
        self.activated = True  # 활성화 여부
        self.object_detected = False  # 물체 감지 여부 (사용하지 않음)
        
        # 그리퍼 너비 구독
        self.width_sub = rospy.Subscriber('hande_gripper/width', Float64, self.width_callback)
        
        # 서비스 연결
        self.control_service = None
        try:
            rospy.loginfo("그리퍼 서비스 연결 대기 중...")
            rospy.wait_for_service('hande_gripper/control', timeout=2.0)
            self.control_service = rospy.ServiceProxy('hande_gripper/control', GripperControl)
            rospy.loginfo("그리퍼 서비스 연결 성공")
        except rospy.ROSException:
            rospy.logwarn("그리퍼 서비스 연결 실패")
        except Exception as e:
            rospy.logerr(f"그리퍼 서비스 연결 오류: {e}")
        
        # 상태 업데이트 타이머
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.emit_status)
        self.update_timer.start(100)  # 10Hz
        
        rospy.loginfo("그리퍼 컨트롤러 초기화 완료")
    
    def width_callback(self, msg):
        """그리퍼 너비 콜백"""
        self.width = msg.data
        self.position = self.width / 0.025  # 0.025m = 25mm (최대 열림 폭)
        
        # 너비가 변경되면 이동 중 상태를 업데이트
        # (실제로는 이동 중 상태는 서비스에서 받아와야 하지만, 간소화를 위해 여기서는 생략)
        if abs(self.width - self.old_width) > 0.001 if hasattr(self, 'old_width') else True:
            self.moving = True
        else:
            self.moving = False
        
        self.old_width = self.width
    
    def emit_status(self):
        """현재 상태를 Qt 시그널로 발생"""
        status_dict = {
            'position': self.position,
            'force': self.force,
            'speed': self.speed,
            'width': self.width,
            'activated': self.activated,
            'moving': self.moving,
            'object_detected': self.object_detected,  # 항상 False
            'virtual_mode': self.virtual_mode
        }
        self.status_updated.emit(status_dict)
    
    def call_gripper_service(self, command_type, value=0):
        """그리퍼 서비스 호출 공통 함수"""
        if self.control_service is None:
            rospy.logwarn("그리퍼 서비스에 연결되어 있지 않습니다")
            return False
        
        try:
            req = GripperControlRequest()
            req.command_type = command_type
            req.value = value
            
            resp = self.control_service(req)
            if not resp.success:
                rospy.logwarn(f"그리퍼 서비스 호출 실패: {resp.message}")
            
            return resp.success
        except Exception as e:
            rospy.logerr(f"그리퍼 서비스 호출 오류: {e}")
            return False
    
    def set_force(self, force):
        """그리퍼 힘 설정 (0-255)"""
        self.force = force
        return self.call_gripper_service(GripperControlRequest.FORCE, force)
    
    def set_speed(self, speed):
        """그리퍼 속도 설정 (0-255)"""
        self.speed = speed
        return self.call_gripper_service(GripperControlRequest.SPEED, speed)
    
    def set_position(self, position):
        """그리퍼 위치 설정 (0.0-1.0)"""
        # 위치 범위 제한
        position = max(0.0, min(1.0, position))
        
        # 위치 값을 0-255 범위로 변환
        position_value = int(position * 255)
        
        # 서비스 호출
        success = self.call_gripper_service(GripperControlRequest.POSITION, position_value)
        
        if success:
            # 이미 moving 상태로 설정, 실제 상태는 width_callback에서 업데이트됨
            self.moving = True
        
        return success
    
    def open_gripper(self):
        """그리퍼 완전히 열기"""
        rospy.loginfo("그리퍼 열기 명령 발행")
        # GripperControlRequest.OPEN 대신 숫자 값 3 사용
        success = self.call_gripper_service(3)  # OPEN=3으로 정의됨
        if success:
            self.moving = True
        return success

    def close_gripper(self):
        """그리퍼 완전히 닫기"""
        rospy.loginfo("그리퍼 닫기 명령 발행")
        # GripperControlRequest.CLOSE 대신 숫자 값 4 사용
        success = self.call_gripper_service(4)  # CLOSE=4로 정의됨
        if success:
            self.moving = True
        return success
    def close(self):
        """컨트롤러 종료"""
        # 타이머 정지
        if self.update_timer and self.update_timer.isActive():
            self.update_timer.stop()
        
        # 구독 해제
        if self.width_sub:
            self.width_sub.unregister()