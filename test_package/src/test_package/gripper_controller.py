#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Bool, UInt8
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from hande_bringup.srv import GripperControl, GripperControlRequest

class GripperController(QObject):
    """그리퍼 제어 및 상태 모니터링 클래스"""
    status_updated = pyqtSignal(dict)  # 그리퍼 상태 업데이트
    
    # 명령 타입 상수 정의 (하드 코딩)
    CMD_POSITION = 0  # 위치 설정 (0-255)
    CMD_SPEED = 1     # 속도 설정 (0-255)
    CMD_FORCE = 2     # 힘 설정 (0-255)
    CMD_OPEN = 3      # 그리퍼 열기
    CMD_CLOSE = 4     # 그리퍼 닫기
    
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
        self.object_detected = False  # 물체 감지 여부
        
        # 그리퍼 상태 구독
        self.width_sub = rospy.Subscriber('hande_gripper/width', Float64, self.width_callback)
        self.object_sub = rospy.Subscriber('hande_gripper/object_detected', Bool, self.object_callback)
        self.moving_sub = rospy.Subscriber('hande_gripper/moving', Bool, self.moving_callback)
        self.position_sub = rospy.Subscriber('hande_gripper/position', UInt8, self.position_callback)
        
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
        max_width = 0.050  # 최대 열림 폭 50mm = 0.050m
        self.position = self.width / max_width  # 0.0-1.0 범위로 정규화
        
        # 너비가 변경되면 이동 중 상태를 업데이트
        if hasattr(self, 'old_width'):
            if abs(self.width - self.old_width) > 0.001:
                self.moving = True
            else:
                self.moving = False
                
        self.old_width = self.width
    
    def object_callback(self, msg):
        """물체 감지 콜백"""
        self.object_detected = msg.data
        rospy.loginfo(f"물체 감지 상태 업데이트: {self.object_detected}")
    
    def moving_callback(self, msg):
        """이동 중 상태 콜백"""
        self.moving = msg.data
    
    def position_callback(self, msg):
        """위치 값 콜백"""
        # 0-255 범위의 값을 0.0-1.0 범위로 변환
        self.position = msg.data / 255.0
    
    def emit_status(self):
        """현재 상태를 Qt 시그널로 발생"""
        status_dict = {
            'position': self.position,
            'force': self.force,
            'speed': self.speed,
            'width': self.width,
            'activated': self.activated,
            'moving': self.moving,
            'object_detected': self.object_detected,
            'virtual_mode': self.virtual_mode
        }
        self.status_updated.emit(status_dict)
    
    def call_gripper_service(self, command_type, value=0):
        """그리퍼 서비스 호출 공통 함수"""
        if self.control_service is None:
            rospy.logwarn("그리퍼 서비스에 연결되어 있지 않습니다")
            self.try_reconnect_service()
            return False
        
        try:
            req = GripperControlRequest()
            req.command_type = command_type
            req.value = value
            
            resp = self.control_service(req)
            if not resp.success:
                rospy.logwarn(f"그리퍼 서비스 호출 실패: {resp.message}")
            else:
                rospy.loginfo(f"그리퍼 서비스 호출 성공: {resp.message}")
            
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"그리퍼 서비스 호출 오류: {e}")
            self.try_reconnect_service()
            return False
        except Exception as e:
            rospy.logerr(f"그리퍼 서비스 호출 중 예외 발생: {e}")
            return False
    
    def try_reconnect_service(self):
        """서비스 재연결 시도"""
        try:
            rospy.loginfo("그리퍼 서비스 재연결 시도...")
            self.control_service = rospy.ServiceProxy('hande_gripper/control', GripperControl)
            # 서비스 가용성 테스트
            self.control_service.wait_for_service(timeout=1.0)
            rospy.loginfo("그리퍼 서비스 재연결 성공")
            return True
        except (rospy.ROSException, rospy.ServiceException):
            rospy.logwarn("그리퍼 서비스 재연결 실패")
            return False
    
    def set_force(self, force):
        """그리퍼 힘 설정 (0-255)"""
        self.force = force
        return self.call_gripper_service(self.CMD_FORCE, force)
    
    def set_speed(self, speed):
        """그리퍼 속도 설정 (0-255)"""
        self.speed = speed
        return self.call_gripper_service(self.CMD_SPEED, speed)
    
    def set_position(self, position):
        """그리퍼 위치 설정 (0.0-1.0)"""
        # 위치 범위 제한
        position = max(0.0, min(1.0, position))
        
        # 위치 값을 0-255 범위로 변환
        position_value = int(position * 255)
        
        # 서비스 호출
        success = self.call_gripper_service(self.CMD_POSITION, position_value)
        
        if success:
            # 이미 moving 상태로 설정, 실제 상태는 콜백에서 업데이트됨
            self.moving = True
        
        return success
    
    def open_gripper(self):
        """그리퍼 완전히 열기"""
        rospy.loginfo("그리퍼 열기 명령 발행")
        success = self.call_gripper_service(self.CMD_OPEN)
        if success:
            self.moving = True
        return success
    
    def close_gripper(self):
        """그리퍼 완전히 닫기"""
        rospy.loginfo("그리퍼 닫기 명령 발행")
        success = self.call_gripper_service(self.CMD_CLOSE)
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
        if self.object_sub:
            self.object_sub.unregister()
        if self.moving_sub:
            self.moving_sub.unregister()
        if self.position_sub:
            self.position_sub.unregister()