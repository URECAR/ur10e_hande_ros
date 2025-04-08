#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QGroupBox, QGridLayout, QPushButton, QLineEdit,
                            QTextEdit, QScrollArea, QFrame, QSplitter,
                            QMessageBox, QDialog, QDialogButtonBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QSize
from PyQt5.QtGui import QFont, QColor, QTextCursor, QIcon, QPixmap

class LogWidget(QTextEdit):
    """소켓 로그 표시 위젯"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMinimumHeight(150)
        self.setMaximumHeight(300)
        self.setFont(QFont("Monospace", 9))
        self.setStyleSheet("background-color: #f8f8f8; border: 1px solid #ddd;")
        self.document().setMaximumBlockCount(1000)  # 최대 로그 수 제한
    
    def append_log(self, message, level="INFO"):
        """로그 추가"""
        color = QColor("#000000")  # 기본 검정색
        
        if level == "ERROR":
            color = QColor("#FF0000")  # 빨간색
        elif level == "WARNING":
            color = QColor("#FFA500")  # 주황색
        elif level == "SUCCESS":
            color = QColor("#00AA00")  # 초록색
        elif level == "COMMAND":
            color = QColor("#0000FF")  # 파란색
        
        # 타임스탬프 추가
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        
        # 현재 커서 위치 및 색상 저장
        self.moveCursor(QTextCursor.End)
        current_format = self.currentCharFormat()
        
        # 새 형식 설정 및 텍스트 추가
        format_color = self.currentCharFormat()
        format_color.setForeground(color)
        self.setCurrentCharFormat(format_color)
        
        # 로그 추가
        self.insertPlainText(f"[{timestamp}] [{level}] {message}\n")
        
        # 원래 형식 복원
        self.setCurrentCharFormat(current_format)
        
        # 스크롤을 가장 아래로
        self.scrollToAnchor("bottom")


class CommandButton(QPushButton):
    """명령 버튼 위젯"""
    
    def __init__(self, text, command, parent=None):
        super().__init__(text, parent)
        self.command = command
        self.setMinimumHeight(40)
        self.original_style = self.styleSheet()
    
    def set_success(self):
        """명령 성공 상태로 설정"""
        self.setStyleSheet("background-color: #c8f7c8; border: 1px solid #5cb85c;")
        QTimer.singleShot(1000, self.reset_style)
    
    def set_failure(self):
        """명령 실패 상태로 설정"""
        self.setStyleSheet("background-color: #f7c8c8; border: 1px solid #d9534f;")
        QTimer.singleShot(1000, self.reset_style)
    
    def reset_style(self):
        """원래 스타일로 복원"""
        self.setStyleSheet(self.original_style)


class ConnectionDialog(QDialog):
    """소켓 연결 설정 다이얼로그"""
    
    def __init__(self, parent=None, ip="0.0.0.0", port=50000):
        super().__init__(parent)
        self.setWindowTitle("소켓 서버 설정")
        self.resize(300, 150)
        
        layout = QVBoxLayout(self)
        
        # IP 주소 입력
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(QLabel("IP 주소:"))
        self.ip_edit = QLineEdit(ip)
        ip_layout.addWidget(self.ip_edit)
        layout.addLayout(ip_layout)
        
        # 포트 입력
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("포트:"))
        self.port_edit = QLineEdit(str(port))
        port_layout.addWidget(self.port_edit)
        layout.addLayout(port_layout)
        
        # 버튼
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
    
    def get_settings(self):
        """설정 값 반환"""
        ip = self.ip_edit.text().strip()
        try:
            port = int(self.port_edit.text().strip())
        except ValueError:
            port = 50000
        
        return ip, port


class SocketTab(QWidget):
    """소켓 통신 탭 위젯"""
    
    def __init__(self, socket_manager):
        super().__init__()
        
        # 소켓 매니저 저장
        self.socket_manager = socket_manager
        
        # 상태 변수
        self.is_connected = False
        
        # UI 초기화
        self.init_ui()
        
        # 시그널 연결
        self.connect_signals()
    
    def init_ui(self):
        """UI 초기화"""
        main_layout = QVBoxLayout(self)
        
        # 연결 상태 및 컨트롤 그룹
        connection_group = QGroupBox("소켓 서버 제어")
        connection_layout = QHBoxLayout()
        
        # 상태 표시
        status_layout = QVBoxLayout()
        status_layout.addWidget(QLabel("서버 상태:"))
        self.status_label = QLabel("중지됨")
        self.status_label.setStyleSheet("font-weight: bold; color: gray;")
        status_layout.addWidget(self.status_label)
        connection_layout.addLayout(status_layout)
        
        # 연결 / 해제 버튼
        button_layout = QVBoxLayout()
        self.connect_button = QPushButton("서버 시작")
        self.connect_button.clicked.connect(self.on_connect_clicked)
        button_layout.addWidget(self.connect_button)
        
        self.disconnect_button = QPushButton("서버 중지")
        self.disconnect_button.clicked.connect(self.on_disconnect_clicked)
        self.disconnect_button.setEnabled(False)
        button_layout.addWidget(self.disconnect_button)
        
        connection_layout.addLayout(button_layout)
        connection_group.setLayout(connection_layout)
        main_layout.addWidget(connection_group)
        
        # 로그 및 명령 영역 (수평 분할)
        split_widget = QSplitter(Qt.Horizontal)
        
        # 왼쪽: 로그 영역
        log_group = QGroupBox("소켓 통신 로그")
        log_layout = QVBoxLayout()
        
        self.log_widget = LogWidget()
        log_layout.addWidget(self.log_widget)
        
        # 로그 컨트롤 버튼
        log_buttons = QHBoxLayout()
        self.clear_log_button = QPushButton("로그 지우기")
        self.clear_log_button.clicked.connect(self.clear_log)
        log_buttons.addWidget(self.clear_log_button)
        log_layout.addLayout(log_buttons)
        
        log_group.setLayout(log_layout)
        split_widget.addWidget(log_group)
        
        # 오른쪽: 명령 실행 영역
        command_group = QGroupBox("소켓 명령 실행")
        command_layout = QGridLayout()
        
        # 명령 버튼 생성
        self.command_buttons = []
        
        # 그리퍼 명령 (첫 번째 열)
        self.add_command_button("내부 그립 1", "gripinner1", 0, 0, command_layout)
        self.add_command_button("내부 그립 2", "gripinner2", 1, 0, command_layout)
        self.add_command_button("내부 그립 3", "gripinner3", 2, 0, command_layout)
        self.add_command_button("외부 그립 1", "gripouter1", 3, 0, command_layout)
        self.add_command_button("외부 그립 2", "gripouter2", 4, 0, command_layout)
        self.add_command_button("외부 그립 3", "gripouter3", 5, 0, command_layout)
        
        # 배치 및 카메라 명령 (두 번째 열)
        self.add_command_button("배치 1", "place1", 0, 1, command_layout)
        self.add_command_button("배치 2", "place2", 1, 1, command_layout)
        self.add_command_button("배치 3", "place3", 2, 1, command_layout)
        self.add_command_button("카메라 감지", "Cam", 3, 1, command_layout, 3, 1)  # 3행 크기의 버튼
        
        command_group.setLayout(command_layout)
        split_widget.addWidget(command_group)
        
        # 스플리터 영역 비율 설정
        split_widget.setSizes([500, 300])
        
        main_layout.addWidget(split_widget)
        
        # 초기 UI 상태 설정
        self.update_ui_state()
    
    def add_command_button(self, text, command, row, col, layout, row_span=1, col_span=1):
        """명령 버튼 추가"""
        button = CommandButton(text, command)
        button.clicked.connect(lambda: self.on_command_button_clicked(command))
        layout.addWidget(button, row, col, row_span, col_span)
        self.command_buttons.append(button)
        return button
    
    def connect_signals(self):
        """신호 연결"""
        # 소켓 매니저 신호 연결
        self.socket_manager.status_changed.connect(self.update_status)
        self.socket_manager.log_message.connect(self.add_log)
        self.socket_manager.connection_state_changed.connect(self.update_connection_state)
        self.socket_manager.command_executed.connect(self.on_command_executed)
    
    def update_status(self, status):
        """상태 업데이트"""
        self.status_label.setText(status)
        
        if status == "연결됨":
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
        elif status == "대기중":
            self.status_label.setStyleSheet("font-weight: bold; color: orange;")
        elif status == "중지됨":
            self.status_label.setStyleSheet("font-weight: bold; color: gray;")
        else:
            self.status_label.setStyleSheet("font-weight: bold; color: black;")
    
    def update_connection_state(self, connected):
        """연결 상태 업데이트"""
        self.is_connected = connected
        
        # 명령 버튼 활성화/비활성화
        for button in self.command_buttons:
            button.setEnabled(connected)
        
        if connected:
            self.add_log("클라이언트가 연결되었습니다", "SUCCESS")
        else:
            if self.socket_manager.is_running():
                self.add_log("클라이언트 연결이 종료되었습니다", "WARNING")
    
    def add_log(self, message, level="INFO"):
        """로그 추가"""
        self.log_widget.append_log(message, level)
    
    def clear_log(self):
        """로그 지우기"""
        self.log_widget.clear()
        self.add_log("로그가 지워졌습니다")
    
    def on_disconnect_clicked(self):
        """연결 해제 버튼 클릭 이벤트"""
        if not self.socket_manager.is_running():
            return
        
        confirm_msg = "소켓 서버를 중지하시겠습니까?\n연결된 클라이언트가 있다면 연결이 종료됩니다."
        reply = QMessageBox.question(self, "서버 중지 확인", confirm_msg,
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # 서버 중지
            if self.socket_manager.stop_server():
                self.connect_button.setEnabled(True)
                self.disconnect_button.setEnabled(False)
                self.add_log("소켓 서버가 중지되었습니다", "WARNING")
                self.update_ui_state()
    
    def on_command_button_clicked(self, command):
        """명령 버튼 클릭 이벤트"""
        if not self.is_connected:
            QMessageBox.warning(self, "명령 실행 불가", 
                               "클라이언트가 연결되어 있지 않습니다.\n명령을 실행할 수 없습니다.")
            return
        
        # 명령 실행 확인
        confirm_msg = f"명령 '{command}'을(를) 실행하시겠습니까?"
        reply = QMessageBox.question(self, "명령 실행 확인", confirm_msg,
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # 명령 실행
            self.add_log(f"명령 '{command}' 실행 요청", "COMMAND")
            self.socket_manager.execute_command(command)
    
    def on_command_executed(self, command, success):
        """명령 실행 결과 처리"""
        # 해당 명령 버튼 찾기
        button = None
        for btn in self.command_buttons:
            if btn.command == command:
                button = btn
                break
        
        if button:
            if success:
                button.set_success()
                self.add_log(f"명령 '{command}' 실행 성공", "SUCCESS")
            else:
                button.set_failure()
                self.add_log(f"명령 '{command}' 실행 실패", "ERROR")
    
    def update_ui_state(self):
        """UI 상태 업데이트"""
        is_running = self.socket_manager.is_running()
        is_connected = self.socket_manager.is_connected()
        
        # 버튼 상태 업데이트
        self.connect_button.setEnabled(not is_running)
        self.disconnect_button.setEnabled(is_running)
        
        # 명령 버튼 상태 업데이트
        for button in self.command_buttons:
            button.setEnabled(is_connected)
        
        # 상태 업데이트
        if is_running:
            if is_connected:
                self.update_status("연결됨")
            else:
                self.update_status("대기중")
        else:
            self.update_status("중지됨")
    
    def on_connect_clicked(self):
        """연결 버튼 클릭 이벤트"""
        dialog = ConnectionDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            ip, port = dialog.get_settings()
            
            # 서버 시작 확인
            confirm_msg = f"소켓 서버를 {ip}:{port}에서 시작합니다.\n" \
                          f"서버가 실행되는 동안 다른 로봇 제어 기능은 비활성화됩니다.\n" \
                          f"계속하시겠습니까?"
            reply = QMessageBox.question(self, "서버 시작 확인", confirm_msg,
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                # 소켓 서버 시작
                if self.socket_manager.start_server(ip, port):
                    self.connect_button.setEnabled(False)
                    self.disconnect_button.setEnabled(True)
                    self.add_log("소켓 서버를 시작했습니다", "SUCCESS")
                else:
                    self.add_log("소켓 서버 시작 실패", "ERROR")