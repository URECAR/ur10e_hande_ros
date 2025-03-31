#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QGroupBox, QGridLayout, QFrame, QPushButton, QLineEdit, 
                           QTabWidget, QSlider)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont


class URControlGUI(QMainWindow):
    """UR 로봇 제어 및 모니터링 GUI"""
    
    def __init__(self, robot_controller, gripper_controller):
        super().__init__()
        
        # 컨트롤러 참조 저장
        self.robot_controller = robot_controller
        self.gripper_controller = gripper_controller
        
        # 신호 연결
        self.robot_controller.pose_updated.connect(self.update_pose_display)
        self.robot_controller.joints_updated.connect(self.update_joints_display)
        self.robot_controller.program_state_updated.connect(self.update_program_state)
        self.robot_controller.planning_result.connect(self.update_planning_result)
        self.gripper_controller.status_updated.connect(self.update_gripper_display)
        
        # UI 초기화
        self.init_ui()
        
        # 마지막 업데이트 시간 초기화
        self.last_update_time = time.time()
        
        # 현재 조인트 및 TCP 값
        self.current_joints = [0.0] * 6
        self.current_tcp = [0.0] * 6
        
        # 실행 버튼 초기 비활성화
        self.execute_joint_button.setEnabled(False)
        self.execute_tcp_button.setEnabled(False)
        
        # 실행 중 플래그
        self.executing = False
        
        # 연결 상태 타이머
        self.conn_timer = QTimer(self)
        self.conn_timer.timeout.connect(self.update_connection_status)
        self.conn_timer.start(1000)  # 1초마다 업데이트
        
        # 로그 메시지 초기화
        self.log_label.setText("시스템 준비 완료.")
    
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle('UR 로봇 및 그리퍼 제어')
        self.setGeometry(100, 100, 200, 650)
        
        # 기본 폰트 설정
        default_font = QFont("Sans", 10)
        self.setFont(default_font)
        
        # 메인 위젯 및 레이아웃
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # 상태 프레임
        status_frame = QFrame()
        status_frame.setFrameShape(QFrame.StyledPanel)
        status_frame.setFrameShadow(QFrame.Raised)
        status_layout = QHBoxLayout(status_frame)
        
        self.connection_status_label = QLabel("상태: 초기화 중...")
        status_layout.addWidget(self.connection_status_label)
        
        # 프로그램 상태 레이블
        self.program_state_label = QLabel("프로그램 상태: 불명")
        status_layout.addWidget(self.program_state_label)
        
        main_layout.addWidget(status_frame)
        
        # 로봇/그리퍼 탭 위젯
        self.tabs = QTabWidget()
        
        # 로봇 제어 탭 생성
        robot_tab = QWidget()
        robot_layout = QVBoxLayout(robot_tab)
        
        # 모니터링 섹션
        monitoring_group = QGroupBox("로봇 상태 모니터링")
        monitoring_layout = QVBoxLayout()
        
        # TCP 정보 (3x2 그리드로 표시)
        tcp_group = QGroupBox("TCP 포즈 (mm, 도)")
        tcp_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        tcp_grid = QGridLayout()
        
        # TCP 값을 표시할 레이블들 (3x2 그리드)
        self.tcp_labels = []
        tcp_names = ["X:", "Y:", "Z:", "RX:", "RY:", "RZ:"]
        
        for i, name in enumerate(tcp_names):
            row = i // 2
            col = i % 2 * 2  # 각 항목은 2칸씩 차지 (이름 + 값)
            
            # 이름 레이블
            name_label = QLabel(name)
            name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            tcp_grid.addWidget(name_label, row, col)
            
            # 값 레이블
            value_label = QLabel("0.00")
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value_label.setStyleSheet("background-color: #f0f0f0; padding: 3px; border: 1px solid #cccccc;")
            value_label.setMinimumWidth(80)
            self.tcp_labels.append(value_label)
            tcp_grid.addWidget(value_label, row, col + 1)
        
        tcp_group.setLayout(tcp_grid)
        monitoring_layout.addWidget(tcp_group)
        
        # 조인트 값 (3x2 그리드로 표시)
        joints_group = QGroupBox("조인트 각도 (도)")
        joints_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        joints_grid = QGridLayout()
        
        # 조인트 값을 표시할 레이블들 (3x2 그리드)
        self.joint_labels = []
        joint_names = ["베이스:", "숄더:", "엘보:", "손목 1:", "손목 2:", "손목 3:"]
        
        for i, name in enumerate(joint_names):
            row = i // 2
            col = i % 2 * 2  # 각 항목은 2칸씩 차지 (이름 + 값)
            
            # 이름 레이블
            name_label = QLabel(name)
            name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            joints_grid.addWidget(name_label, row, col)
            
            # 값 레이블
            value_label = QLabel("0.00")
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value_label.setStyleSheet("background-color: #f0f0f0; padding: 3px; border: 1px solid #cccccc;")
            value_label.setMinimumWidth(80)
            self.joint_labels.append(value_label)
            joints_grid.addWidget(value_label, row, col + 1)
        
        joints_group.setLayout(joints_grid)
        monitoring_layout.addWidget(joints_group)
        
        monitoring_group.setLayout(monitoring_layout)
        robot_layout.addWidget(monitoring_group)
        
        # 제어 탭 위젯
        self.control_tabs = QTabWidget()
        
        # 조인트 이동 탭
        joint_tab = QWidget()
        joint_layout = QVBoxLayout(joint_tab)
        
        # 속도 및 가속도 슬라이더 그룹
        joint_slider_group = QGroupBox("속도 및 가속도 설정")
        joint_slider_layout = QGridLayout()
        
        # 속도 슬라이더
        joint_slider_layout.addWidget(QLabel("속도:"), 0, 0)
        self.joint_velocity_slider = QSlider(Qt.Horizontal)
        self.joint_velocity_slider.setRange(1, 100)
        self.joint_velocity_slider.setValue(25)
        self.joint_velocity_slider.setTickPosition(QSlider.TicksBelow)
        self.joint_velocity_slider.setTickInterval(10)
        self.joint_velocity_value = QLabel("25%")
        self.joint_velocity_slider.valueChanged.connect(
            lambda value: self.joint_velocity_value.setText(f"{value}%")
        )
        joint_slider_layout.addWidget(self.joint_velocity_slider, 0, 1)
        joint_slider_layout.addWidget(self.joint_velocity_value, 0, 2)
        
        # 가속도 슬라이더
        joint_slider_layout.addWidget(QLabel("가속도:"), 1, 0)
        self.joint_accel_slider = QSlider(Qt.Horizontal)
        self.joint_accel_slider.setRange(1, 100)
        self.joint_accel_slider.setValue(25)
        self.joint_accel_slider.setTickPosition(QSlider.TicksBelow)
        self.joint_accel_slider.setTickInterval(10)
        self.joint_accel_value = QLabel("25%")
        self.joint_accel_slider.valueChanged.connect(
            lambda value: self.joint_accel_value.setText(f"{value}%")
        )
        joint_slider_layout.addWidget(self.joint_accel_slider, 1, 1)
        joint_slider_layout.addWidget(self.joint_accel_value, 1, 2)
        
        joint_slider_group.setLayout(joint_slider_layout)
        joint_layout.addWidget(joint_slider_group)
        
        joint_input_group = QGroupBox("조인트 공간 이동 (movej)")
        joint_input_layout = QGridLayout()
        
        # 조인트 입력 필드 생성
        self.joint_inputs = []
        joint_names = ["베이스", "숄더", "엘보", "손목 1", "손목 2", "손목 3"]
        
        for i, name in enumerate(joint_names):
            joint_input_layout.addWidget(QLabel(f"{name} (°):"), i, 0)
            joint_input = QLineEdit("0.00")
            self.joint_inputs.append(joint_input)
            joint_input_layout.addWidget(joint_input, i, 1)
        
        joint_input_group.setLayout(joint_input_layout)
        joint_layout.addWidget(joint_input_group)
        
        # 업데이트 버튼
        self.update_joint_button = QPushButton("FeedBack")
        self.update_joint_button.clicked.connect(self.update_joint_inputs)
        joint_layout.addWidget(self.update_joint_button)
        
        # 버튼 레이아웃
        joint_button_layout = QHBoxLayout()
        
        self.plan_joint_button = QPushButton("계획 (Plan)")
        self.plan_joint_button.clicked.connect(self.plan_joint_movement)
        joint_button_layout.addWidget(self.plan_joint_button)
        
        self.execute_joint_button = QPushButton("실행 (Execute)")
        self.execute_joint_button.clicked.connect(self.execute_plan)
        joint_button_layout.addWidget(self.execute_joint_button)
        
        joint_layout.addLayout(joint_button_layout)
        self.control_tabs.addTab(joint_tab, "조인트 이동")
        
        # TCP 이동 탭
        tcp_tab = QWidget()
        tcp_layout = QVBoxLayout(tcp_tab)
        
        # 속도 및 가속도 슬라이더 그룹
        tcp_slider_group = QGroupBox("속도 및 가속도 설정")
        tcp_slider_layout = QGridLayout()
        
        # 속도 슬라이더
        tcp_slider_layout.addWidget(QLabel("속도:"), 0, 0)
        self.tcp_velocity_slider = QSlider(Qt.Horizontal)
        self.tcp_velocity_slider.setRange(1, 100)
        self.tcp_velocity_slider.setValue(25)
        self.tcp_velocity_slider.setTickPosition(QSlider.TicksBelow)
        self.tcp_velocity_slider.setTickInterval(10)
        self.tcp_velocity_value = QLabel("25%")
        self.tcp_velocity_slider.valueChanged.connect(
            lambda value: self.tcp_velocity_value.setText(f"{value}%")
        )
        tcp_slider_layout.addWidget(self.tcp_velocity_slider, 0, 1)
        tcp_slider_layout.addWidget(self.tcp_velocity_value, 0, 2)
        
        # 가속도 슬라이더
        tcp_slider_layout.addWidget(QLabel("가속도:"), 1, 0)
        self.tcp_accel_slider = QSlider(Qt.Horizontal)
        self.tcp_accel_slider.setRange(1, 100)
        self.tcp_accel_slider.setValue(25)
        self.tcp_accel_slider.setTickPosition(QSlider.TicksBelow)
        self.tcp_accel_slider.setTickInterval(10)
        self.tcp_accel_value = QLabel("25%")
        self.tcp_accel_slider.valueChanged.connect(
            lambda value: self.tcp_accel_value.setText(f"{value}%")
        )
        tcp_slider_layout.addWidget(self.tcp_accel_slider, 1, 1)
        tcp_slider_layout.addWidget(self.tcp_accel_value, 1, 2)
        
        tcp_slider_group.setLayout(tcp_slider_layout)
        tcp_layout.addWidget(tcp_slider_group)
        
        tcp_input_group = QGroupBox("TCP 이동 (moveTCP)")
        tcp_input_layout = QGridLayout()
        
        # TCP 입력 필드 생성
        self.tcp_inputs = []
        tcp_labels = ["X (mm)", "Y (mm)", "Z (mm)", "RX (°)", "RY (°)", "RZ (°)"]
        
        for i, label in enumerate(tcp_labels):
            tcp_input_layout.addWidget(QLabel(label), i, 0)
            tcp_input = QLineEdit("0.00")
            self.tcp_inputs.append(tcp_input)
            tcp_input_layout.addWidget(tcp_input, i, 1)
        
        tcp_input_group.setLayout(tcp_input_layout)
        tcp_layout.addWidget(tcp_input_group)
        
        # 업데이트 버튼
        self.update_tcp_button = QPushButton("Update")
        self.update_tcp_button.clicked.connect(self.update_tcp_inputs)
        tcp_layout.addWidget(self.update_tcp_button)
        
        # 버튼 레이아웃
        tcp_button_layout = QHBoxLayout()
        
        self.plan_tcp_button = QPushButton("계획 (Plan)")
        self.plan_tcp_button.clicked.connect(self.plan_tcp_movement)
        tcp_button_layout.addWidget(self.plan_tcp_button)
        
        self.execute_tcp_button = QPushButton("실행 (Execute)")
        self.execute_tcp_button.clicked.connect(self.execute_plan)
        tcp_button_layout.addWidget(self.execute_tcp_button)
        
        tcp_layout.addLayout(tcp_button_layout)
        self.control_tabs.addTab(tcp_tab, "TCP 이동")
        
        robot_layout.addWidget(self.control_tabs)
        
        # 로봇 탭 추가
        self.tabs.addTab(robot_tab, "로봇 제어")
        
        # 그리퍼 탭 생성
        # 그리퍼 탭 생성
        gripper_tab = QWidget()
        gripper_layout = QVBoxLayout(gripper_tab)
        
        # 그리퍼 상태 그룹
        gripper_status_group = QGroupBox("그리퍼 상태")
        gripper_status_layout = QGridLayout()
        
        # 너비 표시
        gripper_status_layout.addWidget(QLabel("너비:"), 0, 0)
        self.gripper_width_label = QLabel("0.00 mm")
        self.gripper_width_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        gripper_status_layout.addWidget(self.gripper_width_label, 0, 1)
        
        # 물체 감지 상태
        gripper_status_layout.addWidget(QLabel("물체 감지:"), 1, 0)
        self.gripper_object_label = QLabel("감지 없음")
        gripper_status_layout.addWidget(self.gripper_object_label, 1, 1)
        
        # 이동 상태
        gripper_status_layout.addWidget(QLabel("상태:"), 2, 0)
        self.gripper_moving_label = QLabel("정지")
        gripper_status_layout.addWidget(self.gripper_moving_label, 2, 1)
        
        gripper_status_group.setLayout(gripper_status_layout)
        gripper_layout.addWidget(gripper_status_group)
        
        # 그리퍼 파라미터 그룹
        gripper_param_group = QGroupBox("그리퍼 파라미터")
        gripper_param_layout = QGridLayout()
        
        # 속도 슬라이더
        gripper_param_layout.addWidget(QLabel("속도:"), 0, 0)
        self.gripper_speed_slider = QSlider(Qt.Horizontal)
        self.gripper_speed_slider.setRange(0, 255)
        self.gripper_speed_slider.setValue(255)
        self.gripper_speed_slider.setTickPosition(QSlider.TicksBelow)
        self.gripper_speed_slider.setTickInterval(25)
        self.gripper_speed_value = QLabel("255")
        self.gripper_speed_slider.valueChanged.connect(self.set_gripper_speed)
        gripper_param_layout.addWidget(self.gripper_speed_slider, 0, 1)
        gripper_param_layout.addWidget(self.gripper_speed_value, 0, 2)
        
        # 힘 슬라이더
        gripper_param_layout.addWidget(QLabel("힘:"), 1, 0)
        self.gripper_force_slider = QSlider(Qt.Horizontal)
        self.gripper_force_slider.setRange(0, 255)
        self.gripper_force_slider.setValue(255)
        self.gripper_force_slider.setTickPosition(QSlider.TicksBelow)
        self.gripper_force_slider.setTickInterval(25)
        self.gripper_force_value = QLabel("255")
        self.gripper_force_slider.valueChanged.connect(self.set_gripper_force)
        gripper_param_layout.addWidget(self.gripper_force_slider, 1, 1)
        gripper_param_layout.addWidget(self.gripper_force_value, 1, 2)
        
        gripper_param_group.setLayout(gripper_param_layout)
        gripper_layout.addWidget(gripper_param_group)
        
    # 그리퍼 위치 제어 그룹 (버튼 대신 텍스트 상자 사용)
        gripper_control_group = QGroupBox("위치 제어")
        gripper_control_layout = QVBoxLayout()

        # 위치 슬라이더
        slider_layout = QHBoxLayout()

        # 닫힘 텍스트 상자 (버튼 대체)
        self.gripper_close_box = QPushButton("닫힘")
        self.gripper_close_box.setStyleSheet("background-color: #f0f0f0; border: 1px solid #cccccc; padding: 5px;")
        self.gripper_close_box.clicked.connect(self.close_gripper)
        slider_layout.addWidget(self.gripper_close_box)

        # 슬라이더
        self.gripper_position_slider = QSlider(Qt.Horizontal)
        self.gripper_position_slider.setRange(0, 255)
        self.gripper_position_slider.setValue(0)
        self.gripper_position_slider.setTickPosition(QSlider.TicksBelow)
        self.gripper_position_slider.setTickInterval(25)
        self.gripper_position_slider.valueChanged.connect(self.set_gripper_position)
        slider_layout.addWidget(self.gripper_position_slider)

        # 열림 텍스트 상자 (버튼 대체)
        self.gripper_open_box = QPushButton("열림")
        self.gripper_open_box.setStyleSheet("background-color: #f0f0f0; border: 1px solid #cccccc; padding: 5px;")
        self.gripper_open_box.clicked.connect(self.open_gripper)
        slider_layout.addWidget(self.gripper_open_box)

        gripper_control_layout.addLayout(slider_layout)
        gripper_control_group.setLayout(gripper_control_layout)
        gripper_layout.addWidget(gripper_control_group)    
        # 그리퍼 탭에 레이아웃 설정 및 탭에 추가
        gripper_tab.setLayout(gripper_layout)
        self.tabs.addTab(gripper_tab, "그리퍼 제어")
        
        # 탭 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.tabs)
        
        # 로그 표시
        self.log_label = QLabel("시스템 준비 중...")
        self.log_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; border: 1px solid #cccccc;")
        main_layout.addWidget(self.log_label)
    
    def update_program_state(self, state):
        """UR 프로그램 상태 업데이트"""
        self.program_state_label.setText(f"프로그램 상태: {state}")
        if state == "RUNNING":
            self.program_state_label.setStyleSheet("color: green")
        else:
            self.program_state_label.setStyleSheet("color: orange")
    
    def update_pose_display(self, x, y, z, rx, ry, rz):
        """TCP 위치 및 방향 업데이트 (3x2 그리드 형식)"""
        # 각 값을 해당 레이블에 표시
        pose_values = [x, y, z, rx, ry, rz]
        for i, value in enumerate(pose_values):
            format_str = "{:.2f}" if i < 3 else "{:.2f}°"  # 위치는 mm, 각도는 도 단위 표시
            self.tcp_labels[i].setText(format_str.format(value))
        
        # 현재 TCP 값 저장
        self.current_tcp = pose_values
        
        # 업데이트 시간 기록
        self.last_update_time = time.time()
    
    def update_joints_display(self, joint_values):
        """조인트 값 업데이트 (3x2 그리드 형식)"""
        # 각 조인트 값을 해당 레이블에 표시
        for i, value in enumerate(joint_values):
            self.joint_labels[i].setText(f"{value:.2f}°")
        
        # 현재 조인트 값 저장
        self.current_joints = joint_values
        
        # 업데이트 시간 기록
        self.last_update_time = time.time()
    
    def update_joint_inputs(self):
        """조인트 이동 탭에서 Update 버튼 클릭시 입력 필드 업데이트"""
        for i, value in enumerate(self.current_joints):
            self.joint_inputs[i].setText(f"{value:.2f}")
        self.log_label.setText("조인트 입력 필드 업데이트 완료")
    
    def update_tcp_inputs(self):
        """TCP 이동 탭에서 Update 버튼 클릭시 입력 필드 업데이트"""
        for i, value in enumerate(self.current_tcp):
            self.tcp_inputs[i].setText(f"{value:.2f}")
        self.log_label.setText("TCP 입력 필드 업데이트 완료")
    
    def update_connection_status(self):
        """연결 상태 업데이트"""
        current_time = time.time()
        if current_time - self.last_update_time > 2.0:  # 2초 이상 업데이트 없으면 연결 문제로 간주
            self.connection_status_label.setText("상태: 연결 끊김")
            self.connection_status_label.setStyleSheet("color: red")
        else:
            self.connection_status_label.setText("상태: 연결됨")
            self.connection_status_label.setStyleSheet("color: green")
    
    def update_planning_result(self, success, message):
        """계획 결과 업데이트"""
        self.log_label.setText(message)
        
        # 계획 성공 시에만 실행 버튼 활성화 (실행 중이 아닐 때만)
        if not self.executing:
            self.execute_joint_button.setEnabled(success)
            self.execute_tcp_button.setEnabled(success)
        
        if success:
            self.log_label.setStyleSheet("color: green")
        else:
            self.log_label.setStyleSheet("color: red")
        
        # 실행 완료 후에는 계획 버튼 활성화
        if "실행" in message and "성공" in message:
            self.executing = False
            self.plan_joint_button.setEnabled(True)
            self.plan_tcp_button.setEnabled(True)
    

    def update_gripper_display(self, status):
        """그리퍼 상태 업데이트 (물체 감지 포함)"""
        # 그리퍼 너비 표시 업데이트 (미터 -> 밀리미터)
        width_mm = status['width'] * 1000  # 미터 -> 밀리미터
        self.gripper_width_label.setText(f"{width_mm:.2f} mm")
        
        # 물체 감지 상태 표시
        if status['object_detected']:
            self.gripper_object_label.setText("물체 감지됨")
            self.gripper_object_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.gripper_object_label.setText("감지 없음")
            self.gripper_object_label.setStyleSheet("color: gray;")
        
        # 이동 상태 표시
        if status['moving']:
            self.gripper_moving_label.setText("이동 중")
            self.gripper_moving_label.setStyleSheet("color: blue; font-weight: bold;")
        else:
            self.gripper_moving_label.setText("정지")
            self.gripper_moving_label.setStyleSheet("color: black;")
        
        # 슬라이더 업데이트 (순환 업데이트 방지)
        position_value = int(status['position'] * 255)
        if abs(position_value - self.gripper_position_slider.value()) > 5:
            self.gripper_position_slider.blockSignals(True)
            self.gripper_position_slider.setValue(position_value)
            self.gripper_position_slider.blockSignals(False)
    
    def set_gripper_speed(self, value):
        """그리퍼 속도 설정"""
        self.gripper_speed_value.setText(str(value))
        self.gripper_controller.set_speed(value)
    
    def set_gripper_force(self, value):
        """그리퍼 힘 설정"""
        self.gripper_force_value.setText(str(value))
        self.gripper_controller.set_force(value)
    
    def set_gripper_position(self, value):
        """슬라이더를 통한 그리퍼 위치 설정"""
        position = value / 255.0  # 0-255 -> 0.0-1.0 변환
        self.gripper_controller.set_position(position)
    
    def open_gripper(self):
        """그리퍼 열기 버튼 클릭"""
        self.gripper_controller.open_gripper()
    
    def close_gripper(self):
        """그리퍼 닫기 버튼 클릭"""
        self.gripper_controller.close_gripper()
    
    def plan_joint_movement(self):
        """조인트 공간 이동 계획"""
        # 입력 필드에서 조인트 값 가져오기
        joint_values = [input_field.text() for input_field in self.joint_inputs]
        
        # 속도 및 가속도 값 가져오기 (0-1 범위로 변환)
        velocity_scaling = self.joint_velocity_slider.value() / 100.0
        accel_scaling = self.joint_accel_slider.value() / 100.0
        
        # 로그 메시지 업데이트
        self.log_label.setText("조인트 이동 계획 중...")
        self.log_label.setStyleSheet("color: black")
        
        # 계획 실행
        self.robot_controller.plan_joint_movement(joint_values, velocity_scaling, accel_scaling)
    
    def plan_tcp_movement(self):
        """TCP 이동 계획"""
        # 입력 필드에서 TCP 값 가져오기
        tcp_values = [input_field.text() for input_field in self.tcp_inputs]
        
        # 속도 및 가속도 값 가져오기 (0-1 범위로 변환)
        velocity_scaling = self.tcp_velocity_slider.value() / 100.0
        accel_scaling = self.tcp_accel_slider.value() / 100.0
        
        # 로그 메시지 업데이트
        self.log_label.setText("TCP 이동 계획 중...")
        self.log_label.setStyleSheet("color: black")
        
        # 계획 실행
        self.robot_controller.plan_pose_movement(tcp_values, velocity_scaling, accel_scaling)
    
    def execute_plan(self):
        """현재 계획 실행"""
        # 실행 중이면 무시
        if self.executing:
            return
        
        # 로그 메시지 업데이트
        self.log_label.setText("계획 실행 중...")
        self.log_label.setStyleSheet("color: black")
        
        # 실행 버튼 비활성화
        self.execute_joint_button.setEnabled(False)
        self.execute_tcp_button.setEnabled(False)
        
        # 계획 실행
        self.robot_controller.execute_plan()
        
        # 실행 중 플래그 설정
        self.executing = True
    
    def closeEvent(self, event):
        """창이 닫힐 때 호출되는 이벤트 핸들러"""
        # 그리퍼 컨트롤러 종료
        if hasattr(self, 'gripper_controller'):
            self.gripper_controller.close()
        
        # 이벤트 수락
        event.accept()