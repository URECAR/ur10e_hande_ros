#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import time
import threading
import socket
from enum import Enum
from collections import OrderedDict
from std_msgs.msg import Float64, Bool, UInt8
from sensor_msgs.msg import JointState
from hande_bringup.srv import GripperControl, GripperControlResponse
from hande_bringup.msg import HandEGripperCommand, HandEGripperStatus

class RobotiqHandEDriver:
    """Robotiq Hand-E 그리퍼 드라이버 (소켓 통신 방식)"""
    
    # WRITE VARIABLES (요청 변수)
    ACT = 'ACT'  # 활성화 (1 = 활성화됨)
    GTO = 'GTO'  # 이동 명령
    ATR = 'ATR'  # 자동 해제
    ADR = 'ADR'  # 자동 해제 방향
    FOR = 'FOR'  # 힘 (0-255)
    SPE = 'SPE'  # 속도 (0-255)
    POS = 'POS'  # 위치 (0-255)

    # READ VARIABLES (응답 변수)
    STA = 'STA'  # 상태
    PRE = 'PRE'  # 위치 에코
    OBJ = 'OBJ'  # 물체 감지
    FLT = 'FLT'  # 폴트

    # 소켓 통신 인코딩
    ENCODING = 'UTF-8'

    # 그리퍼 상태 열거형
    class GripperStatus(Enum):
        RESET = 0
        ACTIVATING = 1
        ACTIVE = 3

    # 물체 감지 상태 열거형
    class ObjectStatus(Enum):
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1  # 외부 물체와 접촉
        STOPPED_INNER_OBJECT = 2  # 내부 물체와 접촉
        AT_DEST = 3               # 목표 지점에 도달
    
    def __init__(self, host='localhost', port=63352):
        """생성자"""
        # 소켓 통신 관련 변수
        self.socket = None
        self.command_lock = threading.Lock()
        self.connected = False
        self.host = host
        self.port = port
        
        # 그리퍼 범위 설정
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255
        
        # 그리퍼 상태 변수
        self.activated = False
        self.moving = False
        self.obj_status = 0
        self.fault_status = 0
        self.position = 0
        self.current = 0
        self.target_position = 0
        self.speed = 255
        self.force = 255
    
    def connect(self):
        """소켓 연결 설정"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(2.0)
            self.connected = True
            
            # 초기 상태 확인
            rospy.loginfo(f"그리퍼와 연결 성공: {self.host}:{self.port}")
            return True
        except socket.error as e:
            rospy.logerr(f"그리퍼 연결 오류: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """연결 종료"""
        if self.socket:
            try:
                self.socket.close()
                self.connected = False
            except:
                pass
    
    def _set_vars(self, var_dict):
        """여러 변수 설정 및 응답 대기"""
        if not self.connected:
            return False
            
        cmd = 'SET'
        for variable, value in var_dict.items():
            cmd += f' {variable} {value}'
        cmd += '\n'
        
        try:
            with self.command_lock:
                self.socket.sendall(cmd.encode(self.ENCODING))
                data = self.socket.recv(1024)
            return self._is_ack(data)
        except socket.error as e:
            rospy.logerr(f"그리퍼 명령 전송 오류: {e}")
            return False
    
    def _set_var(self, variable, value):
        """단일 변수 설정"""
        return self._set_vars(OrderedDict([(variable, value)]))
    
    def _get_var(self, variable):
        """변수 값 가져오기"""
        if not self.connected:
            return 0
            
        try:
            with self.command_lock:
                self.socket.sendall(f"GET {variable}\n".encode(self.ENCODING))
                data = self.socket.recv(1024)
            
            response = data.decode(self.ENCODING).strip()
            var_name, value_str = response.split()
            if var_name != variable:
                rospy.logwarn(f"예상치 못한 응답: {response}")
                return 0
            return int(value_str)
        except (socket.error, ValueError) as e:
            rospy.logerr(f"그리퍼 상태 읽기 오류: {e}")
            return 0
    
    @staticmethod
    def _is_ack(data):
        """응답이 ack인지 확인"""
        try:
            return data.strip().lower() == b'ack'
        except:
            return False
    
    def _reset(self):
        """그리퍼 리셋 (폴트 해제)"""
        rospy.loginfo("그리퍼 리셋 시작...")
        
        # ACT와 ATR 비트를 0으로 설정해 비활성화
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        
        # 비활성화 완료될 때까지 대기
        retry_count = 0
        while (self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0) and retry_count < 5:
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
            time.sleep(0.1)
            retry_count += 1
        
        time.sleep(0.5)
        rospy.loginfo("그리퍼 리셋 완료")
    
    def activate_gripper(self):
        """그리퍼 활성화"""
        if not self.connected:
            rospy.logwarn("활성화 실패: 그리퍼 연결되지 않음")
            return False
        
        if self.is_active():
            rospy.loginfo("그리퍼가 이미 활성화되어 있습니다")
            self.activated = True
            return True
        
        # 그리퍼 리셋
        self._reset()
        
        # 활성화 명령 전송
        rospy.loginfo("그리퍼 활성화 중...")
        self._set_var(self.ACT, 1)
        
        # 활성화 대기
        time.sleep(1.0)
        retry_count = 0
        while (self._get_var(self.ACT) != 1 or self._get_var(self.STA) != 3) and retry_count < 5:
            time.sleep(0.2)
            retry_count += 1
        
        # 활성화 상태 확인
        self.activated = self.is_active()
        if self.activated:
            rospy.loginfo("그리퍼 활성화 성공")
        else:
            rospy.logerr("그리퍼 활성화 실패")
        
        return self.activated
    
    def is_active(self):
        """그리퍼가 활성화되어 있는지 확인"""
        if not self.connected:
            return False
        try:
            sta_value = self._get_var(self.STA)
            return RobotiqHandEDriver.GripperStatus(sta_value) == RobotiqHandEDriver.GripperStatus.ACTIVE
        except:
            return False
    
    def get_current_position(self):
        """현재 위치 가져오기"""
        if not self.connected:
            return self.position  # 현재 저장된 위치 반환
        try:
            self.position = self._get_var(self.POS)
            return self.position
        except:
            return self.position
    
    def auto_calibrate(self):
        """개방/폐쇄 위치 자동 보정"""
        if not self.is_active():
            rospy.logwarn("보정 실패: 그리퍼가 활성화되지 않음")
            return False
        
        try:
            # 완전 열기
            rospy.loginfo("그리퍼 보정 중: 열기...")
            pos, stat = self.move_and_wait_for_pos(self._min_position, 64, 1)
            if RobotiqHandEDriver.ObjectStatus(stat) != RobotiqHandEDriver.ObjectStatus.AT_DEST:
                rospy.logerr(f"열기 보정 실패: {stat}")
                return False
            
            # 완전 닫기
            rospy.loginfo("그리퍼 보정 중: 닫기...")
            pos, stat = self.move_and_wait_for_pos(self._max_position, 64, 1)
            if RobotiqHandEDriver.ObjectStatus(stat) != RobotiqHandEDriver.ObjectStatus.AT_DEST:
                rospy.logerr(f"닫기 보정 실패: {stat}")
                return False
            self._max_position = pos
            
            # 다시 열기
            rospy.loginfo("그리퍼 보정 중: 다시 열기...")
            pos, stat = self.move_and_wait_for_pos(self._min_position, 64, 1)
            if RobotiqHandEDriver.ObjectStatus(stat) != RobotiqHandEDriver.ObjectStatus.AT_DEST:
                rospy.logerr(f"재열기 보정 실패: {stat}")
                return False
            self._min_position = pos
            
            rospy.loginfo(f"그리퍼 보정 완료: 위치 범위 [{self._min_position}, {self._max_position}]")
            return True
            
        except Exception as e:
            rospy.logerr(f"보정 중 오류 발생: {e}")
            return False
    
    def send_command(self, position, speed=None, force=None):
        """그리퍼 위치 명령 전송"""
        if not self.connected or not self.activated:
            rospy.logwarn("명령 전송 실패: 그리퍼가 연결되지 않았거나 활성화되지 않음")
            return False
        
        # 속도와 힘을 지정하지 않은 경우 저장된 값 사용
        if speed is not None:
            self.speed = speed
        if force is not None:
            self.force = force
        
        # 목표 위치 저장
        self.target_position = position
        
        # 위치 명령 전송
        success, clip_pos = self.move(position, self.speed, self.force)
        
        if success:
            rospy.loginfo(f"위치 명령 전송 성공: {position}, 속도: {self.speed}, 힘: {self.force}")
            self.moving = True
        else:
            rospy.logerr("위치 명령 전송 실패")
        
        return success
    
    def move(self, position, speed, force):
        """비차단 이동 명령 전송"""
        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))
        
        # 입력값 범위 제한
        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)
        
        # 명령 변수 설정
        var_dict = OrderedDict([
            (self.POS, clip_pos),
            (self.SPE, clip_spe),
            (self.FOR, clip_for),
            (self.GTO, 1)
        ])
        
        # 명령 전송
        success = self._set_vars(var_dict)
        return success, clip_pos
    
    def move_and_wait_for_pos(self, position, speed, force):
        """이동 완료까지 대기하는 차단 이동 명령"""
        success, clip_pos = self.move(position, speed, force)
        if not success:
            rospy.logerr('이동 명령 전송 실패')
            return position, 0
        
        # 목표 위치 도달 대기
        timeout = 5.0  # 최대 대기 시간(초)
        start_time = time.time()
        
        # 우선 PRE(위치 에코)가 명령 위치와 일치할 때까지 대기
        while self._get_var(self.PRE) != clip_pos:
            if time.time() - start_time > timeout:
                rospy.logwarn("이동 타임아웃: 위치 에코 대기 중")
                break
            time.sleep(0.01)
        
        # 이제 이동이 완료될 때까지 대기
        status = self._get_var(self.OBJ)
        while RobotiqHandEDriver.ObjectStatus(status) == RobotiqHandEDriver.ObjectStatus.MOVING:
            if time.time() - start_time > timeout:
                rospy.logwarn("이동 타임아웃: 이동 완료 대기 중")
                break
            status = self._get_var(self.OBJ)
            time.sleep(0.01)
        
        # 최종 위치와 상태 반환
        final_pos = self._get_var(self.POS)
        return final_pos, status
    
    def open_gripper(self):
        """그리퍼 완전히 열기"""
        rospy.loginfo("그리퍼 열기 시작")
        return self.send_command(0, self.speed, self.force)
    
    def close_gripper(self):
        """그리퍼 완전히 닫기"""
        rospy.loginfo("그리퍼 닫기 시작")
        return self.send_command(255, self.speed, self.force)
    
    def reset_gripper(self):
        """그리퍼 초기화/리셋"""
        if not self.connected:
            rospy.logwarn("리셋 실패: 그리퍼가 연결되지 않음")
            return False
        
        try:
            # 리셋 수행
            self._reset()
            
            # 다시 활성화
            success = self.activate_gripper()
            
            if success:
                rospy.loginfo("그리퍼 초기화 및 재활성화 성공")
            else:
                rospy.logerr("그리퍼 재활성화 실패")
                
            return success
        except Exception as e:
            rospy.logerr(f"그리퍼 리셋 중 오류 발생: {e}")
            return False
    
    def get_status(self):
        """그리퍼 상태 읽기"""
        if not self.connected:
            rospy.logwarn("상태 읽기 실패: 그리퍼가 연결되지 않음")
            return None
        
        try:
            # 상태 변수 읽기
            sta_value = self._get_var(self.STA)
            pos_value = self._get_var(self.POS)
            obj_value = self._get_var(self.OBJ)
            flt_value = self._get_var(self.FLT)
            pre_value = self._get_var(self.PRE)
            
            # 상태 파싱
            self.activated = (sta_value == RobotiqHandEDriver.GripperStatus.ACTIVE.value)
            self.position = pos_value
            self.obj_status = obj_value
            self.fault_status = flt_value
            
            # 물체 감지 상태 확인
            object_detected = (obj_value == RobotiqHandEDriver.ObjectStatus.STOPPED_OUTER_OBJECT.value or 
                               obj_value == RobotiqHandEDriver.ObjectStatus.STOPPED_INNER_OBJECT.value)
            
            # 이동 상태 확인 (PRE != POS 또는 OBJ == MOVING)
            self.moving = (pre_value != pos_value or obj_value == RobotiqHandEDriver.ObjectStatus.MOVING.value)
            
            # 상태 정보 반환
            status = {
                'activated': self.activated,
                'moving': self.moving,
                'obj_status': obj_value,
                'object_detected': object_detected,
                'fault_status': flt_value,
                'position': pos_value,
                'current': 0  # 현재 전류 정보는 없음
            }
            
            return status
            
        except Exception as e:
            rospy.logerr(f"상태 읽기 중 오류 발생: {e}")
            return None


class HandEGripperDriver:
    """Hand-E 그리퍼 ROS 드라이버 노드"""
    
    def __init__(self):
        # ROS 노드 초기화 (이미 초기화되었는지 확인)
        if not rospy.core.is_initialized():
            rospy.init_node('hande_gripper_driver', anonymous=True)
        
        # 모드 및 설정 가져오기
        self.mode = rospy.get_param('~mode', 'virtual')
        
        # 실제 모드일 경우 호스트 및 포트 설정
        if self.mode == 'real':
            self.host = rospy.get_param('~host', 'localhost')
            # 포트 값을 문자열에서 정수로 변환
            self.port = int(rospy.get_param('~port', '63352'))
        else:
            self.host = 'localhost'
            self.port = 0  # 가상 모드에서는 사용하지 않음
        
        rospy.loginfo(f"Hand-E 그리퍼 드라이버 시작: {self.mode} 모드")
        
        # 그리퍼 컨트롤러 (실제 모드일 때만 사용)
        self.gripper = None
        
        if self.mode == 'real':
            try:
                self.gripper = RobotiqHandEDriver(self.host, self.port)
                if not self.gripper.connect():
                    rospy.logwarn("실제 그리퍼 연결 실패, 가상 모드로 전환합니다")
                    self.mode = 'virtual'
                else:
                    # 그리퍼 활성화
                    if not self.gripper.activate_gripper():
                        rospy.logwarn("그리퍼 활성화 실패, 가상 모드로 전환합니다")
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
        
        # 마지막 발행 위치 추적 변수
        self.last_published_position = 0
        self.last_published_status = 0
        
        # 그리퍼 메시지 관련 발행자 및 구독자
        self.status_pub = rospy.Publisher('hande_gripper/status', HandEGripperStatus, queue_size=10)
        self.cmd_sub = rospy.Subscriber('hande_gripper/command', HandEGripperCommand, self.command_callback)
        
        # 그리퍼 정보 발행자 (사용자 편의를 위한 추가 토픽)
        self.width_pub = rospy.Publisher('hande_gripper/width', Float64, queue_size=10)
        self.object_pub = rospy.Publisher('hande_gripper/object_detected', Bool, queue_size=10)
        self.moving_pub = rospy.Publisher('hande_gripper/moving', Bool, queue_size=10)
        self.position_pub = rospy.Publisher('hande_gripper/position', UInt8, queue_size=10)
        
        # Joint State 발행자 - RViz 시각화 (원래 형식으로 복원)
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
                # 위치는 0=닫힘, 255=열림이 일반적이지만, ROS에서는 
                # 일반적으로 0.0=열림, 1.0=닫힘을 사용하므로 변환
                # Robotiq은 0=열림, 255=닫힘이므로 1.0 - (pos/255)로 변환
                self.position = status['position'] / 255.0
                self.activated = status['activated']
                self.moving = status['moving']
                self.object_detected = status['object_detected']
                
                self.last_status_update = rospy.Time.now()
                return True
            
            return False
            
        except Exception as e:
            rospy.logwarn(f"실제 그리퍼 상태 업데이트 오류: {e}")
            return False
    
    def reset_gripper(self):
        """그리퍼 초기화/리셋"""
        if self.mode != 'real' or not self.gripper:
            # 가상 모드에서는 항상 성공한 것으로 처리
            rospy.loginfo("가상 모드에서 그리퍼 초기화 시뮬레이션")
            self.activated = True
            return True
        
        try:
            # 실제 그리퍼 초기화
            success = self.gripper.reset_gripper()
            
            if success:
                # 상태 변수 업데이트
                self.activated = True
                rospy.loginfo("그리퍼 초기화 성공")
            else:
                rospy.logerr("그리퍼 초기화 실패")
            
            return success
        except Exception as e:
            rospy.logerr(f"그리퍼 초기화 중 오류 발생: {str(e)}")
            return False
    
    def command_callback(self, msg):
        """그리퍼 명령 콜백 함수"""
        # 활성화 명령 처리
        if msg.rACT and not self.activated:
            self.activated = True
            if self.mode == 'real' and self.gripper:
                self.gripper.activate_gripper()
            rospy.loginfo("그리퍼 활성화")
        elif not msg.rACT and self.activated:
            self.activated = False
            if self.mode == 'real' and self.gripper:
                self.gripper._reset()
            rospy.loginfo("그리퍼 비활성화")
        
        # 위치, 속도, 힘 명령 처리
        if self.activated and msg.rGTO:
            # GUI에서는 0(닫힘) ~ 255(열림) 방식으로 전달됨
            # 내부 표현은 0(열림) ~ 1(닫힘)으로 유지
            target_position = msg.rPR
            internal_position = 1.0 - (target_position / 255.0)
            
            self.speed = msg.rSP
            self.force = msg.rFR
            
            if self.mode == 'real' and self.gripper:
                # 실제 그리퍼는 0(열림) ~ 255(닫힘) 방식을 사용하므로 변환 필요
                self.gripper.send_command(target_position, self.speed, self.force)
            else:
                # 가상 모드에서는 이동 시뮬레이션 시작
                self.target_position = internal_position
                self.start_movement_simulation()
            
            self.moving = True
            rospy.loginfo(f"이동 명령 수신: 위치={target_position} (내부: {internal_position:.2f}), 속도={self.speed}, 힘={self.force}")
    
    def handle_control_service(self, req):
        """그리퍼 제어 서비스 핸들러"""
        response = GripperControlResponse()
        
        try:
            self.last_command_time = rospy.Time.now()
            
            if req.command_type == 0:  # POSITION (0-255)
                position_value = req.value
                
                # UI 값 반전: 0(닫힘) ~ 255(열림) → 내부 표현: 0(열림) ~ 1(닫힘)
                internal_position = 1.0 - (position_value / 255.0)
                
                if self.mode == 'real' and self.gripper:
                    # 실제 그리퍼는 0(열림) ~ 255(닫힘) 이므로 값을 그대로 사용
                    success = self.gripper.send_command(position_value, self.speed, self.force)
                    if success:
                        self.target_position = internal_position
                        self.moving = True
                else:
                    success = True
                    self.target_position = internal_position
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"위치 이동 명령 {'성공' if success else '실패'}"
                
            elif req.command_type == 1:  # SPEED
                self.speed = req.value
                
                if self.mode == 'real' and self.gripper:
                    # 속도만 변경하는 경우 현재 위치 유지
                    position_value = int((1.0 - self.position) * 255)  # 방향 반전
                    success = self.gripper.send_command(position_value, self.speed, self.force)
                else:
                    success = True
                
                response.success = success
                response.message = f"속도 설정: {self.speed}"
                
            elif req.command_type == 2:  # FORCE
                self.force = req.value
                
                if self.mode == 'real' and self.gripper:
                    # 힘만 변경하는 경우 현재 위치 유지
                    position_value = int((1.0 - self.position) * 255)  # 방향 반전
                    success = self.gripper.send_command(position_value, self.speed, self.force)
                else:
                    success = True
                
                response.success = success
                response.message = f"힘 설정: {self.force}"
                
            elif req.command_type == 3:  # OPEN
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.open_gripper()
                    if success:
                        # 내부 표현에서는 열림이 0
                        self.target_position = 0.0
                        self.moving = True
                else:
                    success = True
                    self.target_position = 0.0
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"그리퍼 열기 {'성공' if success else '실패'}"
                
            elif req.command_type == 4:  # CLOSE
                if self.mode == 'real' and self.gripper:
                    success = self.gripper.close_gripper()
                    if success:
                        # 내부 표현에서는 닫힘이 1
                        self.target_position = 1.0
                        self.moving = True
                else:
                    success = True
                    self.target_position = 1.0
                    self.start_movement_simulation()
                
                response.success = success
                response.message = f"그리퍼 닫기 {'성공' if success else '실패'}"
                
            elif req.command_type == 5:  # RESET/INITIALIZE
                rospy.loginfo("그리퍼 초기화/리셋 명령 수신")
                success = self.reset_gripper()
                
                response.success = success
                if success:
                    response.message = "그리퍼 초기화 성공"
                else:
                    response.message = "그리퍼 초기화 실패"
            
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
                if self.target_position > 0.7 and self.position > 0.5:
                    # 20% 확률로 물체 감지 시뮬레이션
                    if not self.object_detected and current_time.to_sec() % 10 < 2:
                        self.object_detected = True
                        rospy.loginfo("가상 물체 감지 시뮬레이션: 물체 감지됨")
            else:
                # 이동 완료
                self.position = self.target_position
                self.moving = False
                self.movement_active = False
                
                # 완전히 열리면 물체 감지 해제
                if self.position < 0.1:
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
        # 위치는 0(닫힘)~1(열림) 로직 표현, width는 물리적 거리(m)
        width = (1.0 - self.position) * max_width
        
        # 위치 값 (0-255)으로 변환
        # UI와 일관성을 맞추기 위해 방향 반전: 0(닫힘)~255(열림)
        position_value = int((1.0 - self.position) * 255)
        
        # 토픽 발행
        self.width_pub.publish(Float64(data=width))
        self.object_pub.publish(Bool(data=self.object_detected))
        self.moving_pub.publish(Bool(data=self.moving))
        
        # position_pub는 그리퍼 UI와 연동되는 중요한 값이므로,
        # 이동 중이거나 값이 크게 변경되었을 때만 발행
        if self.moving or not hasattr(self, 'last_published_position') or abs(self.last_published_position - position_value) > 10:
            self.position_pub.publish(UInt8(data=position_value))
            self.last_published_position = position_value
        
        # 상태 메시지 생성 및 발행
        status_msg = HandEGripperStatus()
        status_msg.gACT = self.activated
        status_msg.gGTO = self.moving
        status_msg.gSTA = self.activated
        
        # 물체 감지 상태 설정
        if not self.activated:
            status_msg.gOBJ = 0
        elif self.object_detected:
            status_msg.gOBJ = 1
        elif self.position >= 0.99:
            status_msg.gOBJ = 3  # 최대 닫힘 위치 도달
        elif self.position <= 0.01:
            status_msg.gOBJ = 2  # 최대 열림 위치 도달
        else:
            status_msg.gOBJ = 0  # 이동 중
        
        status_msg.gFLT = 0  # 결함 없음
        status_msg.gPR = int((1.0 - self.target_position) * 255)  # 방향 반전
        status_msg.gPO = position_value
        status_msg.gCU = int(0.5 * 255) if self.moving else int(0.1 * 255)  # 이동 중일 때 가상 전류값 설정
        
        # 상태 발행 - 이동 중이거나 값이 크게 변경되었을 때만 발행
        if self.moving or not hasattr(self, 'last_published_status') or abs(self.last_published_status - position_value) > 10:
            self.status_pub.publish(status_msg)
            self.last_published_status = position_value
        
        # 조인트 상태 발행 (시각화를 위해) - JointState 메시지로 변경
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['gripper_robotiq_hande_left_finger_joint', 'gripper_robotiq_hande_right_finger_joint']
        
        # 관절 위치는 대칭적으로 움직이므로 양쪽에 같은 값
        finger_position = width / 2.0  # 핑거 한 쪽당 이동 거리
        joint_state.position = [finger_position, finger_position]
        
        # 속도와 힘은 옵션 (생략)
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
            self.gripper.disconnect()
        
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