#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import os
import json
import rospy
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QGroupBox, QGridLayout, QFrame, QPushButton, QLineEdit, 
                           QTabWidget, QSlider, QListWidget, QInputDialog, QMessageBox, QDialog,
                           QRadioButton, QDialogButtonBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

# Import our new camera tab
from test_package.camera_tab import CameraTab

class URControlGUI(QMainWindow):
    """UR 로봇 제어 및 모니터링 GUI"""
    
    def __init__(self, robot_controller, gripper_controller):
        super().__init__()
        
        # 컨트롤러 참조 저장
        self.robot_controller = robot_controller
        self.gripper_controller = gripper_controller
        
        # 포즈 관리자 초기화
        from test_package.pose_manager import PoseManager
        self.pose_manager = PoseManager()
        
        # 선택된 포즈 추적
        self.selected_pose_name = None
        
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
        self.execute_pose_button.setEnabled(False)
        
        # 실행 중 플래그
        self.executing = False
        
        # 연결 상태 타이머
        self.conn_timer = QTimer(self)
        self.conn_timer.timeout.connect(self.update_connection_status)
        self.conn_timer.start(1000)  # 1초마다 업데이트
        
        # 로그 메시지 초기화
        self.log_label.setText("시스템 준비 완료.")
        
        # 시작 시 좌표 업데이트 타이머 (1회 실행)
        QTimer.singleShot(2000, self.initial_update)
        
        # 포즈 목록 로딩 연결 (포즈 관리자 초기화 후 목록 로딩)
        self.pose_manager.pose_list_updated.connect(self.update_pose_list)
        
        # 수동으로 포즈 목록 업데이트 호출
        QTimer.singleShot(1000, lambda: self.update_pose_list(self.pose_manager.get_pose_names()))
        
        # 계획 및 실행 완료 후 자동 업데이트를 위한 플래그
        self.auto_update_after_movement = True
    
    def initial_update(self):
        """GUI 초기화 후 조인트 및 TCP 값을 입력 필드에 설정"""
        self.update_joint_inputs()
        self.update_tcp_inputs()

    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle('UR 로봇 및 그리퍼 제어')
        self.setGeometry(100, 100, 200, 500)
        
        # 기본 폰트 설정

        default_font = QFont("NanumGothic", 10)
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
            row = i % 3  # 3개씩 세로로 배치
            col = i // 3 * 3  # 2열씩 배치 (라벨 + 입력필드)
            # 이름 레이블
            name_label = QLabel(name)
            name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            tcp_grid.addWidget(name_label, row, col)
            
            # 값 레이블
            value_label = QLabel("0.00")
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value_label.setStyleSheet("background-color: #f0f0f0; padding: 3px; border: 1px solid #cccccc;")
            value_label.setMinimumWidth(70)
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
            row = i % 3
            col = i // 3 * 3  # 각 항목은 2칸씩 차지 (이름 + 값)
            
            # 이름 레이블
            name_label = QLabel(name)
            name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            joints_grid.addWidget(name_label, row, col)
            
            # 값 레이블
            value_label = QLabel("0.00")
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value_label.setStyleSheet("background-color: #f0f0f0; padding: 3px; border: 1px solid #cccccc;")
            value_label.setMinimumWidth(70)
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
        
        joint_input_group = QGroupBox("조인트 공간 이동")
        joint_input_layout = QGridLayout()
        
        # 조인트 입력 필드 생성
        self.joint_inputs = []
        joint_names = ["베이스", "숄더", "엘보", "손목 1", "손목 2", "손목 3"]

        for i, name in enumerate(joint_names):
            row = i % 3  # 3개씩 세로로 배치
            col = i // 3 * 3  # 2열씩 배치 (라벨 + 입력필드)

            joint_input_layout.addWidget(QLabel(f"{name} :"), row, col)
            joint_input = QLineEdit("0.00")
            self.joint_inputs.append(joint_input)
            joint_input_layout.addWidget(joint_input, row, col + 1)

        joint_input_group.setLayout(joint_input_layout)
        joint_layout.addWidget(joint_input_group)
        
        # 업데이트 버튼
        self.update_joint_button = QPushButton("Update")
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
        self.control_tabs.addTab(joint_tab, "Movej")
        
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
        
        tcp_input_group = QGroupBox("공간 좌표 이동")
        tcp_input_layout = QGridLayout()
        
        # TCP 입력 필드 생성
        self.tcp_inputs = []
        tcp_labels = ["X: ", "Y: ", "Z: ", "RX: ", "RY: ", "RZ: "]

        for i, label in enumerate(tcp_labels):
            row = i % 3  # 3개씩 나누어 행 계산
            col = i // 3 * 3  # 각 라벨과 입력필드 쌍마다 2칸씩 사용
            
            tcp_input_layout.addWidget(QLabel(label), row, col)
            tcp_input = QLineEdit("0.00")
            self.tcp_inputs.append(tcp_input)
            tcp_input_layout.addWidget(tcp_input, row, col + 1)

        tcp_input_group.setLayout(tcp_input_layout)
        tcp_layout.addWidget(tcp_input_group)
        
        # 업데이트 및 카테시안 버튼 레이아웃
        update_cartesian_layout = QHBoxLayout()

        # 업데이트 버튼 (1x1로 줄임)
        self.update_tcp_button = QPushButton("Update")
        self.update_tcp_button.clicked.connect(self.update_tcp_inputs)
        update_cartesian_layout.addWidget(self.update_tcp_button)

        # Cartesian 이동 계획 버튼
        self.cartesian_tcp_button = QPushButton("Cartesian Plan")
        self.cartesian_tcp_button.clicked.connect(self.plan_cartesian_movement)
        self.cartesian_tcp_button.setToolTip("목표 지점까지 직선으로 이동하는 경로 계획")
        update_cartesian_layout.addWidget(self.cartesian_tcp_button)

        tcp_layout.addLayout(update_cartesian_layout)
        
        # 버튼 레이아웃
        tcp_button_layout = QHBoxLayout()
        
        self.plan_tcp_button = QPushButton("계획 (Plan)")
        self.plan_tcp_button.clicked.connect(self.plan_tcp_movement)
        tcp_button_layout.addWidget(self.plan_tcp_button)
        
        self.execute_tcp_button = QPushButton("실행 (Execute)")
        self.execute_tcp_button.clicked.connect(self.execute_plan)
        tcp_button_layout.addWidget(self.execute_tcp_button)
        
        tcp_layout.addLayout(tcp_button_layout)
        self.control_tabs.addTab(tcp_tab, "Movex")
        
        # 포즈 지정 탭 추가 (새로 추가)
        pose_tab = self.create_pose_tab()
        self.control_tabs.addTab(pose_tab, "포즈 지정")
        
        robot_layout.addWidget(self.control_tabs)
        
        # 로봇 탭 추가
        self.tabs.addTab(robot_tab, "로봇 제어")
        
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
        
        gripper_init_group = QGroupBox("그리퍼 관리")
        gripper_init_layout = QVBoxLayout()
        
        self.init_gripper_button = QPushButton("그리퍼 초기화")
        self.init_gripper_button.setToolTip("그리퍼가 응답하지 않을 때 초기화합니다 (Modbus RTU 프로토콜 사용)")
        self.init_gripper_button.clicked.connect(self.initialize_gripper)
        self.init_gripper_button.setStyleSheet("background-color: #ffeeee; font-weight: bold;")
        gripper_init_layout.addWidget(self.init_gripper_button)
        
        gripper_init_group.setLayout(gripper_init_layout)
        gripper_layout.addWidget(gripper_init_group)

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
        
        # 카메라 탭 생성 및 추가
        camera_tab = CameraTab()
        self.tabs.addTab(camera_tab, "카메라")
        
        # 탭 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.tabs)
        
        # 로그 표시
        self.log_label = QLabel("시스템 준비 중...")
        self.log_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; border: 1px solid #cccccc;")
        main_layout.addWidget(self.log_label)
    

    def create_pose_tab(self):
        pose_tab = QWidget()
        pose_layout = QVBoxLayout(pose_tab)
        
        # 포즈 탭 상단 (속도/가속도 슬라이더)
        pose_slider_group = QGroupBox("속도 및 가속도 설정")
        pose_slider_layout = QGridLayout()
        
        # 속도 슬라이더
        pose_slider_layout.addWidget(QLabel("속도:"), 0, 0)
        self.pose_velocity_slider = QSlider(Qt.Horizontal)
        self.pose_velocity_slider.setRange(1, 100)
        self.pose_velocity_slider.setValue(25)
        self.pose_velocity_slider.setTickPosition(QSlider.TicksBelow)
        self.pose_velocity_slider.setTickInterval(10)
        self.pose_velocity_value = QLabel("25%")
        self.pose_velocity_slider.valueChanged.connect(
            lambda value: self.pose_velocity_value.setText(f"{value}%")
        )
        pose_slider_layout.addWidget(self.pose_velocity_slider, 0, 1)
        pose_slider_layout.addWidget(self.pose_velocity_value, 0, 2)
        
        # 가속도 슬라이더
        pose_slider_layout.addWidget(QLabel("가속도:"), 1, 0)
        self.pose_accel_slider = QSlider(Qt.Horizontal)
        self.pose_accel_slider.setRange(1, 100)
        self.pose_accel_slider.setValue(25)
        self.pose_accel_slider.setTickPosition(QSlider.TicksBelow)
        self.pose_accel_slider.setTickInterval(10)
        self.pose_accel_value = QLabel("25%")
        self.pose_accel_slider.valueChanged.connect(
            lambda value: self.pose_accel_value.setText(f"{value}%")
        )
        pose_slider_layout.addWidget(self.pose_accel_slider, 1, 1)
        pose_slider_layout.addWidget(self.pose_accel_value, 1, 2)
        
        pose_slider_group.setLayout(pose_slider_layout)
        pose_layout.addWidget(pose_slider_group)
        
        # 포즈 목록 및 제어 부분
        pose_list_group = QGroupBox("저장된 포즈 목록")
        pose_list_layout = QVBoxLayout()
        
        # 포즈 목록 리스트 위젯
        self.pose_list_widget = QListWidget()
        self.pose_list_widget.setSelectionMode(QListWidget.SingleSelection)
        self.pose_list_widget.itemClicked.connect(self.on_pose_selected)
        pose_list_layout.addWidget(self.pose_list_widget)
        
        # 선택된 포즈 정보 표시
        self.pose_info_label = QLabel("선택된 포즈: 없음")
        self.pose_info_label.setStyleSheet("font-weight: bold;")
        pose_list_layout.addWidget(self.pose_info_label)
        
        # 포즈 작업 버튼 레이아웃
        pose_manage_layout = QHBoxLayout()
        
        # 새 포즈 추가 버튼
        self.add_pose_button = QPushButton("현재 위치 추가")
        self.add_pose_button.clicked.connect(self.add_current_pose)
        pose_manage_layout.addWidget(self.add_pose_button)
        
        # 포즈 삭제 버튼
        self.delete_pose_button = QPushButton("포즈 삭제")
        self.delete_pose_button.clicked.connect(self.delete_selected_pose)
        pose_manage_layout.addWidget(self.delete_pose_button)
        
        pose_list_layout.addLayout(pose_manage_layout)
        pose_list_group.setLayout(pose_list_layout)
        pose_layout.addWidget(pose_list_group)
        
        # 포즈 이동 제어 버튼 (하단)
        pose_layout.addLayout(self.create_pose_control_layout())
        
        return pose_tab
        
    # 포즈 이동 제어 버튼 (하단)
    def create_pose_control_layout(self):
        pose_control_layout = QHBoxLayout()
        
        # 통합된 계획 버튼
        self.plan_pose_button = QPushButton("계획 (Plan)")
        self.plan_pose_button.clicked.connect(self.plan_selected_pose)
        self.plan_pose_button.setEnabled(False)
        pose_control_layout.addWidget(self.plan_pose_button)
        
        # 실행 버튼
        self.execute_pose_button = QPushButton("실행 (Execute)")
        self.execute_pose_button.clicked.connect(self.execute_plan)
        self.execute_pose_button.setEnabled(False)
        pose_control_layout.addWidget(self.execute_pose_button)
        
        return pose_control_layout
    
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
            self.execute_pose_button.setEnabled(success)
        
        if success:
            self.log_label.setStyleSheet("color: green")
        else:
            self.log_label.setStyleSheet("color: red")
        
        # 실행 완료 후에는 계획 버튼 활성화 및 좌표 자동 업데이트
        if "실행" in message and "성공" in message:
            self.executing = False
            self.plan_joint_button.setEnabled(True)
            self.plan_tcp_button.setEnabled(True)
            self.plan_pose_button.setEnabled(True)
            
            # 포즈 실행 버튼 관련 업데이트
            if self.selected_pose_name:
                self.plan_pose_button.setEnabled(True)
            
            # 움직임 완료 후 입력 필드 자동 업데이트
            if self.auto_update_after_movement:
                QTimer.singleShot(500, self.update_joint_inputs)  # 0.5초 후 조인트 업데이트
                QTimer.singleShot(500, self.update_tcp_inputs)    # 0.5초 후 TCP 업데이트

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
    
    def initialize_gripper(self):
        """그리퍼 초기화 버튼 클릭 처리"""
        # 확인 대화 상자 표시
        reply = QMessageBox.question(
            self, "그리퍼 초기화",
            "그리퍼를 초기화하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.log_label.setText("그리퍼 초기화 중...")
            self.log_label.setStyleSheet("color: black;")
            
            # 실제 초기화 실행
            success = self.gripper_controller.reset_gripper()
            
            if success:
                self.log_label.setText("그리퍼 초기화 성공")
                self.log_label.setStyleSheet("color: green;")
            else:
                self.log_label.setText("그리퍼 초기화 실패")
                self.log_label.setStyleSheet("color: red;")

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
    
    def plan_cartesian_movement(self):
        """Cartesian 직선 이동 계획"""
        # 입력 필드에서 TCP 값 가져오기
        tcp_values = [input_field.text() for input_field in self.tcp_inputs]
        
        # 속도 및 가속도 값 가져오기 (0-1 범위로 변환)
        velocity_scaling = self.tcp_velocity_slider.value() / 100.0
        accel_scaling = self.tcp_accel_slider.value() / 100.0
        
        # 로그 메시지 업데이트
        self.log_label.setText("Cartesian 직선 이동 계획 중...")
        self.log_label.setStyleSheet("color: black")
        
        # 계획 실행 (cartesian=True: Cartesian 직선 이동)
        self.robot_controller.plan_pose_movement(tcp_values, velocity_scaling, accel_scaling, cartesian=True)
    
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
        self.execute_pose_button.setEnabled(False)
        
        # 계획 실행
        self.robot_controller.execute_plan()
        
        # 실행 중 플래그 설정
        self.executing = True
    
    def update_pose_list(self, pose_names):
        """포즈 목록 업데이트"""
        # 현재 선택된 항목 기억
        current_selected = self.pose_list_widget.currentItem()
        selected_name = current_selected.text() if current_selected else None
        
        # 목록 초기화
        self.pose_list_widget.clear()
        
        # 포즈 목록 추가
        for name in pose_names:
            self.pose_list_widget.addItem(name)
        
        # 이전에 선택한 항목 다시 선택
        if selected_name:
            items = self.pose_list_widget.findItems(selected_name, Qt.MatchExactly)
            if items:
                self.pose_list_widget.setCurrentItem(items[0])
    
    def on_pose_selected(self, item):
        """포즈 선택 처리"""
        pose_name = item.text()
        self.selected_pose_name = pose_name
        
        # 포즈 정보 가져오기
        pose_data = self.pose_manager.get_pose(pose_name)
        if pose_data:
            # 포즈 타입 및 값 표시
            pose_type = "posj" if pose_data["type"] == "joint" else "posx"
            values_str = ", ".join([f"{val:.2f}" for val in pose_data["values"]])
            self.pose_info_label.setText(f"{pose_type} - {values_str}")
            
            # 버튼 활성화
            self.plan_pose_button.setEnabled(True)
        else:
            self.pose_info_label.setText(f"선택된 포즈: {pose_name} (정보 없음)")
            # 버튼 비활성화
            self.plan_pose_button.setEnabled(False)
 
    def on_socket_connection_changed(self, connected):
        """소켓 연결 상태 변경 처리"""
        # 소켓이 연결되면 UI 컨트롤 비활성화 (소켓 탭 제외)
        current_tab_index = self.tabs.currentIndex()
        socket_tab_index = self.tabs.count() - 1  # 소켓 탭이 마지막에 추가된 경우
        
        if connected:
            # 소켓 연결되면 소켓 탭을 제외한 다른 탭 비활성화
            for i in range(self.tabs.count()):
                if i != socket_tab_index:
                    self.tabs.setTabEnabled(i, False)
            
            # 소켓 탭으로 자동 전환
            self.tabs.setCurrentIndex(socket_tab_index)
            
            # 상태 메시지 업데이트
            self.log_label.setText("소켓 서버가 연결되었습니다. 소켓 모드에서는 다른 탭이 비활성화됩니다.")
            self.log_label.setStyleSheet("color: blue;")
        else:
            # 소켓 연결 해제되면 모든 탭 활성화
            for i in range(self.tabs.count()):
                self.tabs.setTabEnabled(i, True)
            
            # 원래 탭으로 돌아가기 (소켓 탭이었다면 그대로 유지)
            if current_tab_index == socket_tab_index:
                self.tabs.setCurrentIndex(0)  # 첫 번째 탭으로 이동
            
            # 상태 메시지 업데이트
            self.log_label.setText("소켓 서버가 해제되었습니다. 모든 기능이 활성화되었습니다.")
            self.log_label.setStyleSheet("color: black;")
    
    def plan_selected_pose(self):
        """선택된 포즈로 이동 계획 - 통합된 계획 함수"""
        if not self.selected_pose_name:
            self.log_label.setText("포즈를 먼저 선택해주세요.")
            return
        
        pose_data = self.pose_manager.get_pose(self.selected_pose_name)
        if not pose_data:
            self.log_label.setText("포즈 데이터가 없습니다.")
            return
        
        # 슬라이더에서 속도 및 가속도 값 가져오기
        velocity_scaling = self.pose_velocity_slider.value() / 100.0
        accel_scaling = self.pose_accel_slider.value() / 100.0
        
        # 포즈 타입에 따라 계획 생성 방식 선택
        if pose_data["type"] == "joint":
            # 조인트 포즈는 선택지 없이 바로 계획
            self.log_label.setText(f"포즈 '{self.selected_pose_name}'로 조인트 이동 계획 중...")
            self.robot_controller.plan_joint_movement(pose_data["values"], velocity_scaling, accel_scaling)
        else:
            # TCP 포즈는 계획 방식 선택 다이얼로그 표시
            class PlanTypeDialog(QDialog):
                def __init__(self, parent=None):
                    super().__init__(parent)
                    self.setWindowTitle("계획 방식 선택")
                    self.resize(300, 150)
                    
                    layout = QVBoxLayout(self)
                    
                    # 라디오 버튼 그룹
                    self.regular_radio = QRadioButton("일반 계획")
                    self.regular_radio.setChecked(True)
                    self.cartesian_radio = QRadioButton("Cartesian 직선 계획")
                    
                    layout.addWidget(self.regular_radio)
                    layout.addWidget(self.cartesian_radio)
                    
                    # 버튼 박스
                    button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
                    button_box.accepted.connect(self.accept)
                    button_box.rejected.connect(self.reject)
                    
                    layout.addWidget(button_box)
                
                def use_cartesian(self):
                    return self.cartesian_radio.isChecked()
            
            # 계획 방식 선택 다이얼로그 표시
            plan_dialog = PlanTypeDialog(self)
            if plan_dialog.exec_() == QDialog.Accepted:
                use_cartesian = plan_dialog.use_cartesian()
                
                if use_cartesian:
                    # Cartesian 직선 계획
                    self.log_label.setText(f"포즈 '{self.selected_pose_name}'로 Cartesian 직선 이동 계획 중...")
                    self.robot_controller.plan_pose_movement(
                        pose_data["values"], velocity_scaling, accel_scaling, cartesian=True
                    )
                else:
                    # 일반 TCP 계획
                    self.log_label.setText(f"포즈 '{self.selected_pose_name}'로 TCP 이동 계획 중...")
                    self.robot_controller.plan_pose_movement(
                        pose_data["values"], velocity_scaling, accel_scaling, cartesian=False
                    )
    
    def add_current_pose(self):
        """현재 로봇 위치를 포즈로 저장"""
        # 포즈 이름 입력 대화상자
        pose_name, ok = QInputDialog.getText(self, "포즈 추가", "새 포즈 이름:")
        if ok and pose_name:
            # 포즈 타입 선택
            class PoseTypeDialog(QDialog):
                def __init__(self, parent=None):
                    super().__init__(parent)
                    self.setWindowTitle("포즈 타입 선택")
                    self.resize(300, 150)
                    
                    layout = QVBoxLayout(self)
                    
                    # 라디오 버튼 그룹
                    self.joint_radio = QRadioButton("조인트 포즈")
                    self.joint_radio.setChecked(True)
                    self.tcp_radio = QRadioButton("TCP 포즈")
                    
                    layout.addWidget(self.joint_radio)
                    layout.addWidget(self.tcp_radio)
                    
                    # 버튼 박스
                    button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
                    button_box.accepted.connect(self.accept)
                    button_box.rejected.connect(self.reject)
                    
                    layout.addWidget(button_box)
                
                def get_pose_type(self):
                    return "joint" if self.joint_radio.isChecked() else "tcp"
            
            # 포즈 타입 선택 다이얼로그 표시
            type_dialog = PoseTypeDialog(self)
            if type_dialog.exec_() == QDialog.Accepted:
                pose_type = type_dialog.get_pose_type()
                
                # 현재 값 가져오기
                if pose_type == "joint":
                    values = self.current_joints
                else:
                    values = self.current_tcp
                
                # 포즈 추가
                success, message = self.pose_manager.add_pose(pose_name, pose_type, values)
                
                # 결과 표시
                if success:
                    self.log_label.setText(message)
                    self.log_label.setStyleSheet("color: green;")
                else:
                    self.log_label.setText(message)
                    self.log_label.setStyleSheet("color: red;")
    
    def delete_selected_pose(self):
        """선택된 포즈 삭제"""
        if not self.selected_pose_name:
            self.log_label.setText("삭제할 포즈를 먼저 선택해주세요.")
            return
        
        # 삭제 확인 대화상자
        reply = QMessageBox.question(
            self, "포즈 삭제",
            f"포즈 '{self.selected_pose_name}'을(를) 삭제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # 포즈 삭제
            success, message = self.pose_manager.delete_pose(self.selected_pose_name)
            
            # 결과 표시
            if success:
                self.log_label.setText(message)
                self.log_label.setStyleSheet("color: green;")
                self.selected_pose_name = None
                self.pose_info_label.setText("선택된 포즈: 없음")
                self.plan_pose_button.setEnabled(False)
            else:
                self.log_label.setText(message)
                self.log_label.setStyleSheet("color: red;")
    
    def closeEvent(self, event):
        """창이 닫힐 때 호출되는 이벤트 핸들러"""
        # 로봇 컨트롤러 정리
        if hasattr(self, 'robot_controller'):
            try:
                self.robot_controller.cleanup()
            except Exception as e:
                rospy.logerr(f"로봇 컨트롤러 정리 오류: {e}")
        
        # 그리퍼 컨트롤러 종료
        if hasattr(self, 'gripper_controller'):
            self.gripper_controller.close()
        
        # 이벤트 수락
        event.accept()



