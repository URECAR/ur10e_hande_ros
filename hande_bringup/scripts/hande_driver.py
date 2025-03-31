#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import time
import serial
import binascii
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from hande_bringup.srv import GripperControl, GripperControlResponse

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
    
    def set_position(self, position_value, force=255, speed=255):
        """그리퍼 위치 설정 (0-255)"""
        if not self.connected:
            return False
            
        try:
            # 위치 명령 생성
            cmd = bytearray([0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00])
            cmd.append(position_value)  # 위치 (0-255)
            cmd.append(speed)           # 속도
            cmd.append(force)           # 힘
            
            # CRC 계산 (간략화된 방식)
            cmd.extend([0x42, 0x29])  # 실제로는 정확한 CRC 계산 필요
            
            self.serial.write(cmd)
            response = self.serial.readline()
            rospy.loginfo(f"위치 설정 응답: {binascii.hexlify(response)}")
            return True
        except Exception as e:
            rospy.logerr(f"그리퍼 위치 설정 오류: {e}")
            return False
    
    def get_position(self):
        """그리퍼 현재 위치 읽기 (0-255)"""
        if not self.connected:
            return 0
            
        try:
            # 상태 요청 명령
            self.serial.write(b"\x09\x03\x07\xD0\x00\x03\x04\x0E")
            data_raw = self.serial.readline()
            
            # 응답 파싱 (간략화)
            if len(data_raw) >= 6:
                position = data_raw[5] if len(data_raw) > 5 else 0
                return position
            return 0
        except Exception as e:
            rospy.logerr(f"그리퍼 위치 읽기 오류: {e}")
            return 0
    
    def close(self):
        """연결 종료"""
        if self.serial and self.connected:
            self.serial.close()
            self.connected = False


class HandEGripperDriver:
    """간소화된 Hand-E 그리퍼 드라이버"""
    
    def __init__(self):
        # ROS 노드 초기화 (이미 초기화되었는지 확인)
        if not rospy.core.is_initialized():
            rospy.init_node('hande_gripper_driver', anonymous=True)
        
        # 모드 확인 (real or virtual)
        self.mode = rospy.get_param('~mode', 'virtual')
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        
        rospy.loginfo(f"Hand-E 그리퍼 드라이버 시작: {self.mode} 모드")
        
        # 실제 그리퍼 컨트롤러 (real 모드일 때만 사용)
        self.gripper = None
        
        if self.mode == 'real':
            try:
                self.gripper = SimpleGripperController(self.port)
                if not self.gripper.connect():
                    rospy.logwarn("실제 그리퍼 연결 실패, 가상 모드로 전환합니다")
                    self.mode = 'virtual'
            except Exception as e:
                rospy.logerr(f"그리퍼 컨트롤러 초기화 오류: {e}")
                self.mode = 'virtual'
                rospy.logwarn(f"가상 모드로 전환: {e}")
        
        # 상태 변수
        self.position = 0.0  # 그리퍼 위치 (0.0 = 닫힘, 1.0 = 열림)
        self.target_position = 0.0  # 목표 위치
        self.force = 255  # 힘 (0-255)
        self.speed = 255  # 속도 (0-255)
        self.moving = False  # 이동 중 상태
        
        # 초기 활성화
        self.activated = True
        
        # 그리퍼 너비 발행자 (미터 단위)
        self.width_pub = rospy.Publisher('hande_gripper/width', Float64, queue_size=10)
        
        # Joint State 발행자 - RViz 시각화를 위해 추가
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 서비스 서버 설정
        self.control_service = rospy.Service('hande_gripper/control', GripperControl, self.handle_control_service)
        
        # 타이머 설정
        self.update_timer = None  # 초기화: 아래에서 실제 타이머 시작
        
        # 타이머 시작
        self.start_update_timer()
        
        # 시뮬레이션용 변수
        self.movement_active = False
        self.movement_start_time = None
        
        rospy.loginfo("Hand-E 그리퍼 드라이버 초기화 완료")
    
    def start_update_timer(self):
        """상태 업데이트 타이머 시작"""
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_state)  # 10Hz
    
    def handle_control_service(self, req):
        """그리퍼 제어 서비스 핸들러"""
        response = GripperControlResponse()
        
        try:
            if req.command_type == 0:  # POSITION
                position_value = req.value  # 0-255 범위
                self.target_position = float(position_value) / 255.0
                
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.set_position(position_value, self.force, self.speed)
                else:
                    success = True
                    # 가상 모드에서 이동 시뮬레이션
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"위치 이동 명령 {'성공' if success else '실패'}"
            
            elif req.command_type == 1:  # SPEED
                self.speed = req.value
                response.success = True
                response.message = f"속도 설정: {self.speed}"
            
            elif req.command_type == 2:  # FORCE
                self.force = req.value
                response.success = True
                response.message = f"힘 설정: {self.force}"
            
            elif req.command_type == 3:  # OPEN
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.open_gripper()
                else:
                    success = True
                    self.target_position = 1.0
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"그리퍼 열기 {'성공' if success else '실패'}"
            
            elif req.command_type == 4:  # CLOSE
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.close_gripper()
                else:
                    success = True
                    self.target_position = 0.0
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"그리퍼 닫기 {'성공' if success else '실패'}"
            
            else:
                response.success = False
                response.message = "알 수 없는 명령 유형"
        
        except Exception as e:
            response.success = False
            response.message = f"명령 처리 오류: {str(e)}"
            rospy.logerr(f"서비스 처리 오류: {e}")
        
        return response
    
    def start_movement_simulation(self):
        """가상 모드에서 그리퍼 이동 시뮬레이션 시작"""
        self.moving = True
        self.movement_active = True
        self.movement_start_time = rospy.Time.now()
    
    def update_state(self, event=None):
        """주기적으로 상태 업데이트 및 발행"""
        # 실제 그리퍼에서 상태 업데이트
        if self.mode == 'real' and self.gripper and self.activated:
            real_position = self.gripper.get_position()
            self.position = float(real_position) / 255.0
            
            # 이동 상태 체크
            if abs(self.position - self.target_position) < 0.02:
                self.moving = False
            
        # 가상 모드에서 위치 업데이트
        elif self.mode == 'virtual' and self.movement_active:
            current_time = rospy.Time.now()
            elapsed = (current_time - self.movement_start_time).to_sec()
            
            # 0.5초 동안 이동 (선형 보간)
            if elapsed < 0.5:
                # 출발 위치에서 목표 위치로 선형 이동
                progress = elapsed / 0.5  # 0.0 ~ 1.0
                start_pos = self.position
                self.position = start_pos + (self.target_position - start_pos) * progress
                self.moving = True
            else:
                # 이동 완료
                self.position = self.target_position
                self.moving = False
                self.movement_active = False
        
        # 그리퍼 너비 발행 (위치 값을 미터 단위로 변환)
        width = self.position * 0.025  # 최대 열림 폭 25mm = 0.025m
        self.width_pub.publish(Float64(data=width))
        
        # Joint State 발행 - RViz 시각화를 위해 추가
        self.publish_joint_state(width)
    
    def publish_joint_state(self, width):
        """Joint State 메시지 발행 (RViz 시각화용)"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        
        # 그리퍼 조인트 이름 (URDF에 정의된 이름과 일치해야 함)
        joint_state.name = ['gripper_robotiq_hande_left_finger_joint', 'gripper_robotiq_hande_right_finger_joint']
        
        # 그리퍼 조인트 위치 (미터 단위)
        joint_state.position = [width, width]  # 양쪽 손가락 동일하게 이동
        
        # 속도와 힘은 옵션 (생략 가능)
        joint_state.velocity = []
        joint_state.effort = []
        
        # Joint State 발행
        self.joint_state_pub.publish(joint_state)
    
    def run(self):
        """그리퍼 드라이버 실행"""
        rospy.loginfo("Hand-E 그리퍼 드라이버 실행 중...")
        rospy.spin()
    
    def shutdown(self):
        """종료 처리"""
        if self.mode == 'real' and self.gripper:
            self.gripper.close()
        
        # 타이머 정지
        if self.update_timer is not None:
            self.update_timer.shutdown()
        
        rospy.loginfo("Hand-E 그리퍼 드라이버 종료")


def main():
    try:
        # 그리퍼 드라이버 생성 및 실행
        driver = HandEGripperDriver()
        
        # 종료 핸들러 등록
        rospy.on_shutdown(driver.shutdown)
        
        # 드라이버 실행
        driver.run()
        
    except Exception as e:
        rospy.logerr(f"그리퍼 드라이버 오류: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()