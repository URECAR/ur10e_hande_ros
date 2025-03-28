#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import serial
import binascii
from std_msgs.msg import Bool, UInt8, Float64
from sensor_msgs.msg import JointState
from PyQt5.QtCore import QObject, QTimer, pyqtSignal

class SimpleGripperController:
    """직접 serial 통신을 사용한 간단한 그리퍼 제어 클래스"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        
    def connect(self):
        """시리얼 포트 연결 및 그리퍼 초기화 설정"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.serial.is_open:
                rospy.loginfo(f"그리퍼와 연결 성공: {self.port}")
                self.connected = True
                
                # 그리퍼 초기화 명령 전송
                rospy.loginfo("그리퍼 초기화 시작...")
                self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
                data_raw = self.serial.readline()
                rospy.loginfo(f"초기화 응답 1: {binascii.hexlify(data_raw)}")
                time.sleep(0.1)
                
                # 상태 확인 명령
                self.serial.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
                data_raw = self.serial.readline()
                rospy.loginfo(f"상태 응답: {binascii.hexlify(data_raw)}")
                time.sleep(1)
                
                rospy.loginfo("그리퍼 초기화 성공")
                return True
            else:
                rospy.logerr(f"시리얼 포트 열기 실패: {self.port}")
                return False
        except Exception as e:
            rospy.logerr(f"그리퍼 연결 오류: {e}")
            return False
    
    def open_gripper(self):
        """그리퍼 열기 (물리적으로 열기)"""
        if not self.connected:
            return False
            
        try:
            # 열기 명령
            self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
            response = self.serial.readline()
            rospy.loginfo(f"열기 응답: {binascii.hexlify(response)}")
            time.sleep(0.5)
            return True
        except Exception as e:
            rospy.logerr(f"그리퍼 열기 오류: {e}")
            return False
    
    def close_gripper(self):
        """그리퍼 닫기 (물리적으로 닫기)"""
        if not self.connected:
            return False
            
        try:
            # 닫기 명령
            self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
            response = self.serial.readline()
            rospy.loginfo(f"닫기 응답: {binascii.hexlify(response)}")
            time.sleep(0.5)
            return True
        except Exception as e:
            rospy.logerr(f"그리퍼 닫기 오류: {e}")
            return False
    
    def get_status(self):
        """그리퍼 상태 읽기"""
        if not self.connected:
            return None
            
        try:
            # 상태 확인 명령
            self.serial.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
            data_raw = self.serial.readline()
            status_data = binascii.hexlify(data_raw)
            
            # 상태 파싱 (실제 그리퍼 문서 참조 필요)
            # 여기서는 단순화된 파싱만 수행
            if len(data_raw) >= 5:
                status_byte = data_raw[4] if len(data_raw) > 4 else 0
                return {
                    'activated': bool(status_byte & 0x01),
                    'moving': bool(status_byte & 0x02),
                    'object_detected': bool(status_byte & 0x04),
                    'position': 0  # 실제 위치는 추가 명령이 필요할 수 있음
                }
            return None
        except Exception as e:
            rospy.logerr(f"그리퍼 상태 읽기 오류: {e}")
            return None
    
    def close(self):
        """연결 종료"""
        if self.serial and self.connected:
            self.serial.close()
            self.connected = False


class GripperController(QObject):
    """그리퍼 제어 및 상태 모니터링 클래스"""
    status_updated = pyqtSignal(dict)  # 그리퍼 상태 업데이트
    
    def __init__(self, ip_address="192.168.56.101"):
        super().__init__()
        
        # IP 주소에 따라 모드 설정 (192.168.56.101은 가상 모드)
        self.virtual_mode = (ip_address == "192.168.56.101")
        
        if self.virtual_mode:
            rospy.loginfo("가상 그리퍼 모드로 초기화합니다.")
            self.gripper = None
        else:
            rospy.loginfo("실제 그리퍼 모드로 초기화합니다.")
            try:
                self.gripper = SimpleGripperController('/dev/ttyUSB0')
                if not self.gripper.connect():
                    rospy.logwarn("실제 그리퍼 연결 실패, 가상 모드로 전환합니다.")
                    self.virtual_mode = True
                    self.gripper = None
            except Exception as e:
                rospy.logerr(f"그리퍼 초기화 오류: {e}")
                self.virtual_mode = True
                self.gripper = None
        
        # 상태 변수
        self.position = 0.0  # 0.0 = 닫힘, 1.0 = 열림
        self.force = 255     # 0-255
        self.speed = 255     # 0-255
        self.activated = False
        self.moving = False
        self.object_detected = False
        
        # 퍼블리셔 설정
        self.joint_state_pub = rospy.Publisher('hande_gripper/joint_state', Float64, queue_size=10)
        self.activated_pub = rospy.Publisher('hande_gripper/status/activated', Bool, queue_size=10)
        self.moving_pub = rospy.Publisher('hande_gripper/status/moving', Bool, queue_size=10)
        self.position_pub = rospy.Publisher('hande_gripper/status/position', UInt8, queue_size=10)
        self.object_pub = rospy.Publisher('hande_gripper/status/object', UInt8, queue_size=10)
        
        # 그리퍼 상태 업데이트 타이머
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gripper_state)
        self.update_timer.start(100)  # 10Hz
        
        rospy.loginfo("그리퍼 컨트롤러 초기화 완료")
    
    def update_gripper_state(self):
        """그리퍼 상태 업데이트 및 발행"""
        # 실제 그리퍼인 경우 상태 읽기
        if not self.virtual_mode and self.gripper:
            status = self.gripper.get_status()
            if status:
                self.activated = status.get('activated', self.activated)
                self.moving = status.get('moving', self.moving)
                self.object_detected = status.get('object_detected', self.object_detected)
                if 'position' in status and status['position'] is not None:
                    self.position = status['position'] / 255.0
        
        # 상태 토픽 발행
        self.publish_status()
        
        # 상태 시그널 발행
        status_dict = {
            'position': self.position,
            'force': self.force,
            'speed': self.speed,
            'activated': self.activated,
            'moving': self.moving,
            'object_detected': self.object_detected,
            'virtual_mode': self.virtual_mode
        }
        self.status_updated.emit(status_dict)
    
    def publish_status(self):
        """ROS 토픽으로 그리퍼 상태 발행"""
        # 활성화 상태
        self.activated_pub.publish(Bool(data=self.activated))
        
        # 이동 상태
        self.moving_pub.publish(Bool(data=self.moving))
        
        # 위치 (0-255 범위)
        pos_value = int(self.position * 255)
        self.position_pub.publish(UInt8(data=pos_value))
        
        # 물체 감지 상태
        obj_status = 0
        if not self.activated:
            obj_status = 0  # 비활성화
        elif self.object_detected:
            obj_status = 1  # 물체 감지
        elif self.position <= 0.01:
            obj_status = 2  # 최대 닫힘
        elif self.position >= 0.99:
            obj_status = 3  # 최대 열림
        
        self.object_pub.publish(UInt8(data=obj_status))
        
        # 조인트 상태 발행 (float64)
        joint_pos = self.position * 0.025  # 최대 열림 폭 25mm
        self.joint_state_pub.publish(Float64(data=joint_pos))
    
    def set_force(self, force):
        """그리퍼 힘 설정 (0-255)"""
        self.force = force
    
    def set_speed(self, speed):
        """그리퍼 속도 설정 (0-255)"""
        self.speed = speed
    
    def set_position(self, position):
        """그리퍼 위치 설정 (0.0-1.0)"""
        # 위치 범위 제한
        position = max(0.0, min(1.0, position))
        self.position = position
        
        # 실제 그리퍼 제어
        if not self.virtual_mode and self.gripper:
            position_value = int(position * 255)
            self.gripper.set_position(position_value, self.force, self.speed)
            self.activated = True
            self.moving = True
            # 이동 완료 상태는 상태 업데이트에서 처리
        else:
            # 가상 모드에서는 즉시 상태 변경
            self.activated = True
            self.moving = True
            # 1초 후 이동 완료로 표시
            QTimer.singleShot(1000, self.complete_movement)
    
    def complete_movement(self):
        """이동 완료 처리 (가상 모드용)"""
        self.moving = False
        # 닫혔을 때 물체 감지 시뮬레이션
        if self.position < 0.2 and self.virtual_mode:
            # 20% 확률로 물체 감지
            if rospy.Time.now().to_sec() % 10 < 2:
                self.object_detected = True
            else:
                self.object_detected = False
        elif self.position > 0.8:
            # 열리면 물체 감지하지 않음
            self.object_detected = False
    
    def open_gripper(self):
        """그리퍼 완전히 열기"""
        if not self.virtual_mode and self.gripper:
            success = self.gripper.open_gripper()
            if success:
                self.position = 1.0
                self.moving = False
                self.activated = True
                self.object_detected = False
            else:
                rospy.logwarn("그리퍼 열기 명령 실패")
        else:
            # 가상 모드에서는 위치 설정 함수 사용
            self.set_position(1.0)
    
    def close_gripper(self):
        """그리퍼 완전히 닫기"""
        if not self.virtual_mode and self.gripper:
            success = self.gripper.close_gripper()
            if success:
                self.position = 0.0
                self.moving = False
                self.activated = True
                # 물체 감지는 그리퍼 상태 업데이트에서 처리
            else:
                rospy.logwarn("그리퍼 닫기 명령 실패")
        else:
            # 가상 모드에서는 위치 설정 함수 사용
            self.set_position(0.0)
    
    def close(self):
        """컨트롤러 종료"""
        if not self.virtual_mode and self.gripper:
            self.gripper.close()