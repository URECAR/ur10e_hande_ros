#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import time
import serial
import binascii
import threading
import struct
from std_msgs.msg import Float64, Bool, UInt8
from sensor_msgs.msg import JointState
from hande_bringup.srv import GripperControl, GripperControlResponse

class ModbusRTU:
    """Modbus RTU 통신 헬퍼 클래스"""
    
    @staticmethod
    def calculate_crc(data):
        """Modbus CRC-16 계산"""
        crc = 0xFFFF
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        
        # CRC를 리틀 엔디안으로 반환
        return crc.to_bytes(2, byteorder='little')
    
    @staticmethod
    def create_command(slave_id, function_code, address, data=None, count=None):
        """Modbus 명령 생성"""
        cmd = bytearray([slave_id, function_code])
        
        # 주소 추가 (빅 엔디안)
        cmd.extend(address.to_bytes(2, byteorder='big'))
        
        if function_code == 0x03:  # 읽기 명령
            # 레지스터 개수 추가 (빅 엔디안)
            cmd.extend(count.to_bytes(2, byteorder='big'))
        elif function_code == 0x10:  # 쓰기 명령
            # 레지스터 개수 추가 (빅 엔디안)
            cmd.extend(count.to_bytes(2, byteorder='big'))
            # 바이트 개수 추가
            cmd.append(len(data))
            # 데이터 추가
            cmd.extend(data)
        
        # CRC 추가
        cmd.extend(ModbusRTU.calculate_crc(cmd))
        
        return cmd
    
    @staticmethod
    def parse_response(response, function_code):
        """Modbus 응답 파싱"""
        if len(response) < 5:  # 최소 길이 확인
            return None
        
        # 헤더 확인 (slave ID, function code)
        if response[0] != 0x09 or response[1] != function_code:
            return None
        
        # CRC 확인
        received_crc = response[-2:]
        calculated_crc = ModbusRTU.calculate_crc(response[:-2])
        
        if received_crc != calculated_crc:
            return None
        
        # 데이터 부분 반환
        if function_code == 0x03:  # 읽기 응답
            byte_count = response[2]
            data = response[3:-2]
            return data
        elif function_code == 0x10:  # 쓰기 응답
            return True
        
        return None


class RobotiqHandEGripper:
    """Robotiq Hand-E 그리퍼 컨트롤러"""
    
    # 상수 정의
    SLAVE_ID = 0x09
    
    # 레지스터 주소
    REG_STATUS = 0x07D0  # 상태 레지스터 시작 주소
    REG_COMMAND = 0x03E8  # 명령 레지스터 시작 주소
    
    # 상태 비트 마스크
    MASK_gACT = 0x01      # 활성화 상태
    MASK_gGTO = 0x02      # 이동 중 상태
    MASK_gSTA = 0x0C      # 그리퍼 상태 (2비트)
    MASK_gOBJ = 0x30      # 물체 감지 상태 (2비트)
    MASK_gFLT = 0xC0      # 폴트 상태 (2비트)
    
    # 명령 비트 마스크
    MASK_rACT = 0x01      # 활성화 명령
    MASK_rGTO = 0x02      # 이동 명령
    MASK_rATR = 0x04      # 자동 해제
    MASK_rARD = 0x08      # 자동 해제 방향
    
    # 물체 감지 상태 값
    OBJ_MOVING = 0x00     # 이동 중
    OBJ_DETECTED = 0x01   # 물체 감지됨
    OBJ_MAX_OPEN = 0x02   # 최대 열림
    OBJ_MAX_CLOSE = 0x03  # 최대 닫힘
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.lock = threading.Lock()  # 시리얼 통신 동기화를 위한 락
        
        # 상태 변수
        self.activated = False
        self.moving = False
        self.obj_status = 0
        self.fault_status = 0
        self.position = 0
        self.current = 0
        
        # 명령 변수
        self.target_position = 0
        self.speed = 255
        self.force = 255
    
    def connect(self):
        """시리얼 포트 연결 및 그리퍼 초기화 설정"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.5,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.serial.is_open:
                rospy.loginfo(f"그리퍼와 연결 성공: {self.port}")
                self.connected = True
                
                # 먼저 활성화 상태 확인
                status = self.get_status()
                
                if status and status.get('activated', False):
                    rospy.loginfo("그리퍼가 이미 활성화되어 있습니다.")
                    self.activated = True
                    return True
                
                # 그리퍼 활성화 명령 전송
                rospy.loginfo("그리퍼 활성화 시작...")
                if self.activate_gripper():
                    rospy.loginfo("그리퍼 활성화 성공")
                    self.activated = True
                    return True
                else:
                    rospy.logerr("그리퍼 활성화 실패")
                    return False
            else:
                rospy.logerr(f"시리얼 포트 열기 실패: {self.port}")
                return False
        except Exception as e:
            rospy.logerr(f"그리퍼 연결 오류: {e}")
            return False
    
    def close_connection(self):
        """연결 종료"""
        if self.serial and self.connected:
            try:
                self.serial.close()
            except:
                pass
            self.connected = False
    
    def activate_gripper(self):
        """그리퍼 활성화"""
        if not self.connected:
            return False
        
        try:
            with self.lock:
                # 활성화 명령 생성 (rACT=1, 나머지 0)
                data = bytearray([1, 0, 0, 0, 0, 0])
                cmd = ModbusRTU.create_command(
                    self.SLAVE_ID, 0x10, self.REG_COMMAND, data, 3)
                
                self.serial.write(cmd)
                response = self.serial.read(8)  # 응답은 8바이트
                
                if len(response) != 8:
                    rospy.logerr(f"활성화 응답 오류: 응답이 8바이트가 아님 ({len(response)}바이트)")
                    return False
                
                # 제대로 된 응답인지 확인
                if response[0] == self.SLAVE_ID and response[1] == 0x10:
                    rospy.loginfo("활성화 명령 전송 성공")
                    
                    # 활성화 상태 확인을 위해 1초 대기
                    time.sleep(1)
                    
                    # 상태 확인
                    status = self.get_status()
                    self.activated = status and status.get('activated', False)
                    
                    return self.activated
                else:
                    rospy.logerr(f"활성화 응답 오류: 예상치 못한 응답 {binascii.hexlify(response)}")
                    return False
                
        except Exception as e:
            rospy.logerr(f"그리퍼 활성화 오류: {e}")
            return False
    
    def send_command(self, position, speed=None, force=None):
        """그리퍼에 이동 명령 전송"""
        if not self.connected or not self.activated:
            return False
        
        # 속도와 힘 값이 지정되지 않았으면 이전 값 사용
        if speed is not None:
            self.speed = speed
        if force is not None:
            self.force = force
        
        # 목표 위치 저장
        self.target_position = position
        
        try:
            with self.lock:
                # 명령 생성 (rACT=1, rGTO=1)
                cmd_byte = 0x09  # rACT=1, rGTO=1
                position_byte = position & 0xFF
                speed_byte = self.speed & 0xFF
                force_byte = self.force & 0xFF
                
                data = bytearray([cmd_byte, 0, 0, position_byte, speed_byte, force_byte])
                cmd = ModbusRTU.create_command(
                    self.SLAVE_ID, 0x10, self.REG_COMMAND, data, 3)
                
                self.serial.write(cmd)
                response = self.serial.read(8)  # 응답은 8바이트
                
                if len(response) != 8:
                    rospy.logerr(f"명령 응답 오류: 응답이 8바이트가 아님 ({len(response)}바이트)")
                    return False
                
                # 제대로 된 응답인지 확인
                if response[0] == self.SLAVE_ID and response[1] == 0x10:
                    rospy.loginfo(f"이동 명령 전송 성공: 위치={position}, 속도={self.speed}, 힘={self.force}")
                    return True
                else:
                    rospy.logerr(f"명령 응답 오류: 예상치 못한 응답 {binascii.hexlify(response)}")
                    return False
                
        except Exception as e:
            rospy.logerr(f"그리퍼 명령 전송 오류: {e}")
            return False
    
    def open_gripper(self):
        """그리퍼 완전히 열기"""
        return self.send_command(0)  # 0 = 완전 열림
    
    def close_gripper(self):
        """그리퍼 완전히 닫기"""
        return self.send_command(255)  # 255 = 완전 닫힘
    
    def get_status(self):
        """그리퍼 상태 읽기"""
        if not self.connected:
            return None
        
        try:
            with self.lock:
                # 상태 읽기 명령 생성
                cmd = ModbusRTU.create_command(
                    self.SLAVE_ID, 0x03, self.REG_STATUS, count=3)
                
                self.serial.write(cmd)
                response = self.serial.read(11)  # 응답은 11바이트
                
                if len(response) != 11:
                    rospy.logwarn(f"상태 응답 오류: 응답이 11바이트가 아님 ({len(response)}바이트)")
                    return None
                
                # 응답 파싱
                if response[0] == self.SLAVE_ID and response[1] == 0x03 and response[2] == 0x06:
                    # 상태 바이트
                    status_byte = response[3]
                    
                    # 위치 값 (0-255)
                    position = response[7]
                    
                    # 전류 값
                    current = response[9]
                    
                    # 상태 비트 해석
                    activated = bool(status_byte & self.MASK_gACT)
                    moving = bool(status_byte & self.MASK_gGTO)
                    
                    # 물체 감지 상태 (2비트)
                    obj_status = (status_byte & self.MASK_gOBJ) >> 4
                    
                    # 폴트 상태 (2비트)
                    fault_status = (status_byte & self.MASK_gFLT) >> 6
                    
                    # 물체 감지 여부
                    object_detected = (obj_status == self.OBJ_DETECTED)
                    
                    # 상태 변수 업데이트
                    self.activated = activated
                    self.moving = moving
                    self.obj_status = obj_status
                    self.fault_status = fault_status
                    self.position = position
                    self.current = current
                    
                    # 상태 정보 딕셔너리 반환
                    return {
                        'activated': activated,
                        'moving': moving,
                        'obj_status': obj_status,
                        'object_detected': object_detected,
                        'fault_status': fault_status,
                        'position': position,
                        'current': current
                    }
                else:
                    rospy.logwarn(f"상태 응답 오류: 예상치 못한 응답 {binascii.hexlify(response)}")
                    return None
                
        except Exception as e:
            rospy.logwarn(f"그리퍼 상태 읽기 오류: {e}")
            return None
            

class HandEGripperDriver:
    """Hand-E 그리퍼 ROS 드라이버 노드"""
    
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
                self.gripper = RobotiqHandEGripper(self.port)
                if not self.gripper.connect():
                    rospy.logwarn("실제 그리퍼 연결 실패, 가상 모드로 전환합니다")
                    self.mode = 'virtual'
            except Exception as e:
                rospy.logerr(f"그리퍼 컨트롤러 초기화 오류: {e}")
                self.mode = 'virtual'
                rospy.logwarn(f"가상 모드로 전환: {e}")
        
        # 상태 변수
        self.position = 0.0  # 그리퍼 위치 (0.0 = 열림, 1.0 = 닫힘) - 논리적 표현
        self.target_position = 0.0  # 목표 위치
        self.force = 255  # 힘 (0-255)
        self.speed = 255  # 속도 (0-255)
        self.moving = False  # 이동 중 상태
        self.activated = True  # 활성화 상태
        self.object_detected = False  # 물체 감지 상태
        
        # 마지막 명령 및 상태 업데이트 시간
        self.last_command_time = rospy.Time.now()
        self.last_status_update = rospy.Time.now()
        
        # 그리퍼 정보 발행자
        self.width_pub = rospy.Publisher('hande_gripper/width', Float64, queue_size=10)
        self.object_pub = rospy.Publisher('hande_gripper/object_detected', Bool, queue_size=10)
        self.moving_pub = rospy.Publisher('hande_gripper/moving', Bool, queue_size=10)
        self.position_pub = rospy.Publisher('hande_gripper/position', UInt8, queue_size=10)
        
        # Joint State 발행자 - RViz 시각화
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 서비스 서버
        self.control_service = rospy.Service('hande_gripper/control', GripperControl, self.handle_control_service)
        
        # 상태 업데이트 타이머
        self.update_timer = None
        self.start_update_timer()
        
        # 시뮬레이션용 변수
        self.movement_active = False
        self.movement_start_time = None
        
        # 실제 그리퍼 초기 상태 확인
        if self.mode == 'real' and self.gripper:
            self.update_real_gripper_state()
        
        rospy.loginfo("Hand-E 그리퍼 드라이버 초기화 완료")
    
    def start_update_timer(self):
        """상태 업데이트 타이머 시작"""
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_state)  # 10Hz
    
    def update_real_gripper_state(self):
        """실제 그리퍼 상태 업데이트"""
        if self.mode != 'real' or not self.gripper:
            return False
            
        try:
            status = self.gripper.get_status()
            
            if status:
                # Robotiq 그리퍼는 0=열림, 255=닫힘이므로 ROS 표현(0.0=닫힘, 1.0=열림)으로 변환
                self.position = 1.0 - (status['position'] / 255.0)
                self.activated = status['activated']
                self.moving = status['moving']
                self.object_detected = status['object_detected']
                
                self.last_status_update = rospy.Time.now()
                return True
            
            return False
            
        except Exception as e:
            rospy.logwarn(f"실제 그리퍼 상태 업데이트 오류: {e}")
            return False
    
    def handle_control_service(self, req):
        """그리퍼 제어 서비스 핸들러"""
        response = GripperControlResponse()
        
        try:
            self.last_command_time = rospy.Time.now()
            
            if req.command_type == 0:  # POSITION (0-255)
                position_value = req.value
                # 입력은 0-255 (0=닫힘, 255=열림) 범위지만 
                # Robotiq은 0=열림, 255=닫힘이므로 변환 필요
                robotiq_position = 255 - position_value
                
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.send_command(robotiq_position, self.speed, self.force)
                    if success:
                        self.target_position = position_value / 255.0
                        self.moving = True
                else:
                    success = True
                    self.target_position = position_value / 255.0
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"위치 이동 명령 {'성공' if success else '실패'}"
                
            elif req.command_type == 1:  # SPEED
                self.speed = req.value
                
                if self.mode == 'real' and self.gripper:
                    # 속도만 변경하는 경우 현재 위치 유지
                    robotiq_position = 255 - int(self.position * 255)
                    success = self.gripper.send_command(robotiq_position, self.speed, self.force)
                else:
                    success = True
                
                response.success = success
                response.message = f"속도 설정: {self.speed}"
                
            elif req.command_type == 2:  # FORCE
                self.force = req.value
                
                if self.mode == 'real' and self.gripper:
                    # 힘만 변경하는 경우 현재 위치 유지
                    robotiq_position = 255 - int(self.position * 255)
                    success = self.gripper.send_command(robotiq_position, self.speed, self.force)
                else:
                    success = True
                
                response.success = success
                response.message = f"힘 설정: {self.force}"
                
            elif req.command_type == 3:  # OPEN
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.open_gripper()
                    if success:
                        self.target_position = 1.0
                        self.moving = True
                else:
                    success = True
                    self.target_position = 1.0
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"그리퍼 열기 {'성공' if success else '실패'}"
                
            elif req.command_type == 4:  # CLOSE
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.close_gripper()
                    if success:
                        self.target_position = 0.0
                        self.moving = True
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
        current_time = rospy.Time.now()
        
        # 실제 그리퍼 상태 업데이트 (10Hz보다 느리게, 1Hz 정도)
        if self.mode == 'real' and self.gripper:
            if (current_time - self.last_status_update).to_sec() >= 0.5:
                self.update_real_gripper_state()
        # 가상 모드 시뮬레이션
        elif self.mode == 'virtual' and self.movement_active:
            elapsed = (current_time - self.movement_start_time).to_sec()
            
            # 0.5초 동안 이동 (선형 보간)
            if elapsed < 0.5:
                # 출발 위치에서 목표 위치로 선형 이동
                progress = elapsed / 0.5  # 0.0 ~ 1.0
                start_pos = self.position
                self.position = start_pos + (self.target_position - start_pos) * progress
                self.moving = True
                
                # 물체 감지 시뮬레이션 (닫을 때만)
                if self.target_position < 0.3 and self.position < 0.5:
                    # 20% 확률로 물체 감지 시뮬레이션
                    if not self.object_detected and rospy.Time.now().to_sec() % 10 < 2:
                        self.object_detected = True
                        rospy.loginfo("가상 물체 감지 시뮬레이션: 물체 감지됨")
            else:
                # 이동 완료
                self.position = self.target_position
                self.moving = False
                self.movement_active = False
                
                # 완전히 열리면 물체 감지 해제
                if self.position > 0.9:
                    self.object_detected = False
        
        # 그리퍼 정보 발행
        self.publish_gripper_state()
    
    def publish_gripper_state(self):
        """그리퍼 상태 발행"""
        # 가장 먼저 연결 상태 확인 (실제 모드의 경우)
        if self.mode == 'real' and self.gripper and not self.gripper.connected:
            try:
                self.gripper.connect()
            except:
                pass
        
        # 위치 값을 미터 단위로 변환
        max_width = 0.050  # 최대 열림 폭 50mm = 0.050m
        width = self.position * max_width
        
        # 위치 값 (0-255)으로 변환
        position_value = int(self.position * 255)
        
        # 토픽 발행
        self.width_pub.publish(Float64(data=width))
        self.object_pub.publish(Bool(data=self.object_detected))
        self.moving_pub.publish(Bool(data=self.moving))
        self.position_pub.publish(UInt8(data=position_value))
        
        # Joint State 발행 (RViz 시각화용)
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['gripper_robotiq_hande_left_finger_joint', 'gripper_robotiq_hande_right_finger_joint']
        
        # 관절 위치는 대칭적으로 움직이므로 양쪽에 같은 값
        # ROS에서는 일반적으로 gripper_width/2가 손가락 위치
        # width의 절반이 각 손가락의 움직임
        finger_position = width / 2.0
        joint_state.position = [finger_position, finger_position]
        
        # 속도와 힘은 옵션 (생략)
        joint_state.velocity = []
        joint_state.effort = []
        
        # Joint State 발행
        self.joint_state_pub.publish(joint_state)
        
        # # 로깅 (5초에 한 번 정도)
        # if int(rospy.Time.now().to_sec() * 10) % 50 == 0:
        #     status_str = f"위치={self.position:.2f}, 너비={width*1000:.1f}mm"
        #     status_str += f", 물체감지={'O' if self.object_detected else 'X'}"
        #     status_str += f", 이동중={'O' if self.moving else 'X'}"
        #     rospy.loginfo(f"그리퍼 상태: {status_str}")
    
    def run(self):
        """그리퍼 드라이버 실행"""
        rospy.loginfo("Hand-E 그리퍼 드라이버 실행 중...")
        rospy.spin()
    
    def shutdown(self):
        """종료 처리"""
        if self.mode == 'real' and self.gripper:
            self.gripper.close_connection()
        
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