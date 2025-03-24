#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import signal
import rospy
import threading
import time
import serial
import binascii
from std_msgs.msg import Float64, Bool, UInt8
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                            QSlider, QPushButton, QLabel, QCheckBox, QGroupBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

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
                
                # 그리퍼 초기화 명령 전송 (공유해주신 코드와 동일)
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
            # 열기 명령 (공유해주신 코드 참조)
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
            # 닫기 명령 (공유해주신 코드 참조)
            self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
            response = self.serial.readline()
            rospy.loginfo(f"닫기 응답: {binascii.hexlify(response)}")
            time.sleep(0.5)
            return True
        except Exception as e:
            rospy.logerr(f"그리퍼 닫기 오류: {e}")
            return False
    
    def close(self):
        """연결 종료"""
        if self.serial and self.connected:
            self.serial.close()
            self.connected = False


class GripperControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        # ROS 노드 초기화 (이미 초기화되었는지 확인)
        if not rospy.core.is_initialized():
            rospy.init_node('hande_gripper_gui_driver', anonymous=True)
        
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
        self.position = 0.0  # 그리퍼 위치 (0.0 = 닫힘, 1.0 = 열림) - 논리적 표현
        self.activated = False  # 활성화 상태
        self.moving = False     # 이동 중 상태
        self.object_detected = False  # 물체 감지 상태
        self.slider_updating = False  # 슬라이더 업데이트 중 플래그
        
        # 마지막 상태 업데이트 시간
        self.last_update_time = rospy.Time.now()
        
        # 퍼블리셔 설정
        self.activated_pub = rospy.Publisher('hande_gripper/status/activated', Bool, queue_size=10)
        self.moving_pub = rospy.Publisher('hande_gripper/status/moving', Bool, queue_size=10)
        self.position_pub = rospy.Publisher('hande_gripper/status/position', UInt8, queue_size=10)
        self.object_pub = rospy.Publisher('hande_gripper/status/object', UInt8, queue_size=10)
        
        # 조인트 상태 발행자
        self.joint_state_pub = rospy.Publisher('hande_gripper/joint_state', Float64, queue_size=10)
        
        # 전체 로봇 조인트 상태에 그리퍼 조인트 추가
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # UI 초기화
        self.initUI()
        
        # ROS 타이머 - 상태 발행 및 가상 모드일 때 시뮬레이션
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_gripper_state)
        self.update_timer.start(100)  # 10Hz
        
        rospy.loginfo("Robotiq Hand-E 그리퍼 GUI 드라이버 초기화 완료")
    
    def initUI(self):
        # 레이아웃 설정
        main_layout = QVBoxLayout()
        from PyQt5.QtGui import QFont
        default_font = QFont("Sans", 9)  # Sans 폰트, 크기 9
        self.setFont(default_font)       
        
        # 상태 표시 영역
        status_box = QGroupBox("그리퍼 상태")
        status_layout = QVBoxLayout()
        
        # 동작 모드 표시
        self.mode_label = QLabel(f"모드: {'실제 그리퍼' if self.mode == 'real' else '가상 그리퍼'}")
        status_layout.addWidget(self.mode_label)
        
        # 위치 슬라이더 및 표시
        slider_layout = QHBoxLayout()
        self.position_slider = QSlider(Qt.Horizontal)
        self.position_slider.setMinimum(0)
        self.position_slider.setMaximum(255)
        self.position_slider.setValue(0)  # 초기값: 닫힘
        self.position_slider.setTickPosition(QSlider.TicksBelow)
        self.position_slider.setTickInterval(25)
        self.position_slider.valueChanged.connect(self.slider_value_changed)
        slider_layout.addWidget(self.position_slider)
        
        self.position_label = QLabel("위치: 닫힘 (0)")
        slider_layout.addWidget(self.position_label)
        status_layout.addLayout(slider_layout)
        
        # 물체 감지 상태
        self.object_label = QLabel("물체 감지: 없음")
        status_layout.addWidget(self.object_label)
        
        status_box.setLayout(status_layout)
        main_layout.addWidget(status_box)
        
        # 제어 버튼
        control_box = QGroupBox("그리퍼 제어")
        control_layout = QHBoxLayout()
        
        # 열기 버튼
        self.open_button = QPushButton("그리퍼 열기")
        self.open_button.clicked.connect(self.open_gripper)
        control_layout.addWidget(self.open_button)
        
        # 닫기 버튼
        self.close_button = QPushButton("그리퍼 닫기")
        self.close_button.clicked.connect(self.close_gripper)
        control_layout.addWidget(self.close_button)
        
        control_box.setLayout(control_layout)
        main_layout.addWidget(control_box)
        
        self.setLayout(main_layout)
        
        # 창 설정
        self.setWindowTitle(f'Robotiq Hand-E 그리퍼 컨트롤 ({self.mode} 모드)')
        self.setGeometry(300, 300, 400, 200)
    
    def slider_value_changed(self):
        """슬라이더 값 변경 시 호출되는 함수"""
        # 다른 곳에서 슬라이더를 업데이트하는 중이면 무시
        if self.slider_updating:
            return
            
        # 슬라이더 값을 그리퍼 위치로 변환 (0-255 -> 0.0-1.0)
        position_value = self.position_slider.value()
        self.position = position_value / 255.0
        
        # 위치 레이블 업데이트
        self.update_position_label()
        
        # 실제 모드에서는 위치에 따라 열기/닫기 명령 선택
        if self.mode == 'real' and self.gripper:
            if position_value > 127:  # 중간값 이상이면 열기
                self.open_gripper()
            else:  # 중간값 미만이면 닫기
                self.close_gripper()
            self.activated = True
            self.moving = True
        else:
            # 가상 모드에서는 즉시 위치 변경
            self.activated = True
            self.moving = True
            
            # 0.5초 후 이동 완료로 표시
            QTimer.singleShot(500, self.complete_movement)
    
    def complete_movement(self):
        """이동 완료 처리"""
        self.moving = False
        # 물체 감지 시뮬레이션 (닫혔을 때만)
        if self.position < 0.2 and self.mode == 'virtual':
            # 20% 확률로 물체 감지
            if rospy.Time.now().to_sec() % 10 < 2:
                self.object_detected = True
            else:
                self.object_detected = False
    
    def update_position_label(self):
        """위치 레이블 업데이트"""
        value = self.position_slider.value()
        if value < 10:
            text = f"위치: 닫힘 ({value})"
        elif value > 245:
            text = f"위치: 완전 열림 ({value})"
        else:
            text = f"위치: {value} (0:닫힘, 255:열림)"
        self.position_label.setText(text)
    
    def open_gripper(self):
        """그리퍼 열기"""
        if self.mode == 'real' and self.gripper:
            success = self.gripper.open_gripper()
            if success:
                self.slider_updating = True
                self.position = 1.0  # 열림
                self.position_slider.setValue(255)
                self.slider_updating = False
                self.moving = False
                self.activated = True
                self.object_detected = False
            else:
                rospy.logwarn("그리퍼 열기 명령 실패")
        else:
            # 가상 모드에서는 즉시 위치 변경
            self.slider_updating = True
            self.position = 1.0  # 열림
            self.position_slider.setValue(255)
            self.slider_updating = False
            self.moving = False
            self.activated = True
            self.object_detected = False
        
        self.update_ui()
    
    def close_gripper(self):
        """그리퍼 닫기"""
        if self.mode == 'real' and self.gripper:
            success = self.gripper.close_gripper()
            if success:
                self.slider_updating = True
                self.position = 0.0  # 닫힘
                self.position_slider.setValue(0)
                self.slider_updating = False
                self.moving = False
                self.activated = True
                # 물체 감지는 실제 그리퍼에서 알 수 없으므로 변경하지 않음
            else:
                rospy.logwarn("그리퍼 닫기 명령 실패")
        else:
            # 가상 모드에서는 즉시 위치 변경 및 물체 감지 시뮬레이션
            self.slider_updating = True
            self.position = 0.0  # 닫힘
            self.position_slider.setValue(0)
            self.slider_updating = False
            self.moving = False
            self.activated = True
            
            # 물체 감지 시뮬레이션 (닫힐 때 20% 확률로 물체 감지)
            if rospy.Time.now().to_sec() % 10 < 2:
                self.object_detected = True
            else:
                self.object_detected = False
        
        self.update_ui()
    
    def update_ui(self):
        """UI 상태 업데이트"""
        # 위치 레이블 업데이트
        self.update_position_label()
        
        # 물체 감지 레이블 업데이트
        if self.object_detected:
            self.object_label.setText("물체 감지: 있음")
        else:
            self.object_label.setText("물체 감지: 없음")
    
    def update_gripper_state(self):
        """그리퍼 상태 업데이트 및 ROS 토픽 발행"""
        # 상태 발행
        self.publish_status()
        self.update_ui()
    
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
        
        # 로봇 조인트 상태에 그리퍼 조인트 추가 (JointState)
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['gripper_robotiq_hande_left_finger_joint', 'gripper_robotiq_hande_right_finger_joint']
        joint_state.position = [joint_pos, joint_pos]
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        self.joint_states_pub.publish(joint_state)
    
    def closeEvent(self, event):
        """창이 닫힐 때 호출되는 이벤트 핸들러"""
        # 그리퍼 연결 종료
        if self.mode == 'real' and self.gripper:
            self.gripper.close()
        
        # 창 닫기 이벤트 수락
        event.accept()


def main():
    # QApplication 생성
    app = QApplication(sys.argv)
    
    # SIGINT 핸들러 설정 (Ctrl+C로 종료할 수 있도록)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # GUI 생성 및 표시
    gui = GripperControlGUI()
    gui.show()
    
    # QApplication 이벤트 루프 실행
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()