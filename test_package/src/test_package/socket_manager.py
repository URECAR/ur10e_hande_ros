#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import time
import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread


class SocketCommandThread(QThread):
    """소켓 명령을 실행하는 스레드"""
    command_received = pyqtSignal(str)
    response_sent = pyqtSignal(str)
    client_connected = pyqtSignal(str)
    client_disconnected = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, socket_manager):
        super().__init__()
        self.socket_manager = socket_manager
        self.client_socket = None
        self.client_addr = None
        self.running = False
    
    def setup(self, client_socket, client_addr):
        """클라이언트 소켓 설정"""
        self.client_socket = client_socket
        self.client_addr = client_addr
        self.running = True
    
    def run(self):
        """스레드 실행"""
        if not self.client_socket:
            self.error_occurred.emit("클라이언트 소켓이 설정되지 않았습니다.")
            return
        
        self.client_connected.emit(f"{self.client_addr}에서 연결됨")
        
        try:
            while self.running and self.socket_manager.is_running():
                try:
                    # 데이터 수신 (1024 바이트씩)
                    data = self.client_socket.recv(1024)
                    if not data:
                        # 연결이 닫혔음
                        break
                    
                    # 명령어 처리
                    command = data.decode('utf-8').strip()
                    self.command_received.emit(command)
                    
                    # 클라이언트에게 수신 확인 응답
                    self.client_socket.send("received".encode('utf-8'))
                    self.response_sent.emit("received")
                    
                    # 명령 처리 (socket_manager에 위임)
                    self.socket_manager.process_command(command)
                    
                    # 완료 응답 전송
                    self.client_socket.send("complete".encode('utf-8'))
                    self.response_sent.emit("complete")
                    
                except socket.timeout:
                    # 타임아웃 (계속 실행)
                    continue
                except ConnectionResetError:
                    self.error_occurred.emit("클라이언트 연결이 리셋되었습니다.")
                    break
                except Exception as e:
                    self.error_occurred.emit(f"클라이언트 통신 오류: {str(e)}")
                    break
                    
        except Exception as e:
            self.error_occurred.emit(f"명령 처리 스레드 오류: {str(e)}")
        finally:
            # 연결 종료
            self.stop()
    
    def stop(self):
        """스레드 중지"""
        self.running = False
        if self.client_socket:
            try:
                self.client_socket.close()
                self.client_disconnected.emit(f"{self.client_addr} 연결 종료")
            except:
                pass
            self.client_socket = None
            self.client_addr = None


class SocketListenThread(QThread):
    """소켓 연결 대기 스레드"""
    server_started = pyqtSignal(str)
    server_stopped = pyqtSignal(str)
    client_accepted = pyqtSignal(tuple)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, socket_manager):
        super().__init__()
        self.socket_manager = socket_manager
        self.server_socket = None
        self.running = False
        self.server_ip = "0.0.0.0"  # 모든 인터페이스에서 연결 수신
        self.server_port = 50000    # 기본 포트
    
    def setup(self, server_ip, server_port):
        """서버 소켓 설정"""
        self.server_ip = server_ip
        self.server_port = server_port
    
    def run(self):
        """스레드 실행"""
        self.running = True
        
        try:
            # 서버 소켓 생성
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.settimeout(1.0)  # 1초 타임아웃 설정
            
            # 바인딩 및 리스닝
            self.server_socket.bind((self.server_ip, self.server_port))
            self.server_socket.listen(5)
            
            self.server_started.emit(f"서버가 {self.server_ip}:{self.server_port}에서 시작됨")
            
            # 연결 수락 대기
            while self.running and self.socket_manager.is_running():
                try:
                    client_socket, client_addr = self.server_socket.accept()
                    self.client_accepted.emit((client_socket, client_addr))
                except socket.timeout:
                    # 타임아웃 (계속 대기)
                    continue
                except Exception as e:
                    if self.running:  # 의도적인 종료가 아닌 경우에만 오류 발생
                        self.error_occurred.emit(f"연결 수락 오류: {str(e)}")
                    break
                    
        except Exception as e:
            if self.running:  # 의도적인 종료가 아닌 경우에만 오류 발생
                self.error_occurred.emit(f"서버 소켓 오류: {str(e)}")
        finally:
            # 소켓 종료
            self.stop()
    
    def stop(self):
        """스레드 중지"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
                self.server_stopped.emit("서버 소켓 종료됨")
            except:
                pass
            self.server_socket = None


class SocketManager(QObject):
    """소켓 통신 관리 클래스"""
    
    # 상태 변화 및 로그 관련 시그널
    status_changed = pyqtSignal(str)
    log_message = pyqtSignal(str)
    connection_state_changed = pyqtSignal(bool)  # 연결 상태 변경 시그널 (True: 연결됨, False: 연결 안됨)
    
    # 명령 실행 관련 시그널
    command_executed = pyqtSignal(str, bool)  # 명령, 성공여부
    
    def __init__(self, robot_controller=None, gripper_controller=None):
        super().__init__()
        
        # 컨트롤러 참조
        self.robot_controller = robot_controller
        self.gripper_controller = gripper_controller
        
        # 상태 변수
        self.connected = False
        self.server_running = False
        self._running = False
        
        # 스레드 객체
        self.listen_thread = SocketListenThread(self)
        self.command_thread = SocketCommandThread(self)
        
        # 시그널 연결
        self._connect_signals()
        
        # 명령 콜백 사전
        self.command_callbacks = {}
        
        # 명령 초기화
        self.register_default_commands()
    
    def _connect_signals(self):
        """스레드 시그널 연결"""
        # 리스닝 스레드 시그널
        self.listen_thread.server_started.connect(self._on_server_started)
        self.listen_thread.server_stopped.connect(self._on_server_stopped)
        self.listen_thread.client_accepted.connect(self._on_client_accepted)
        self.listen_thread.error_occurred.connect(self._on_error)
        
        # 명령 스레드 시그널
        self.command_thread.command_received.connect(self._on_command_received)
        self.command_thread.response_sent.connect(self._on_response_sent)
        self.command_thread.client_connected.connect(self._on_client_connected)
        self.command_thread.client_disconnected.connect(self._on_client_disconnected)
        self.command_thread.error_occurred.connect(self._on_error)
    
    def register_default_commands(self):
        """기본 명령 등록"""
        # 그리퍼 관련 명령
        self.register_command("gripinner1", self._grip_inner_1)
        self.register_command("gripinner2", self._grip_inner_2)
        self.register_command("gripinner3", self._grip_inner_3)
        self.register_command("gripouter1", self._grip_outer_1)
        self.register_command("gripouter2", self._grip_outer_2)
        self.register_command("gripouter3", self._grip_outer_3)
        
        # 배치 관련 명령
        self.register_command("place1", self._place_1)
        self.register_command("place2", self._place_2)
        self.register_command("place3", self._place_3)
        
        # 카메라 관련 명령
        self.register_command("Cam", self._camera_detection)
    
    def register_command(self, command, callback):
        """명령 등록"""
        self.command_callbacks[command] = callback
    
    def start_server(self, ip="0.0.0.0", port=50000):
        """서버 시작"""
        if self.server_running:
            self.log_message.emit("서버가 이미 실행 중입니다.")
            return False
        
        # 상태 설정
        self._running = True
        
        # 스레드 설정 및 시작
        self.listen_thread.setup(ip, port)
        self.listen_thread.start()
        
        return True
    
    def stop_server(self):
        """서버 중지"""
        # 상태 설정
        self._running = False
        
        # 명령 스레드 중지
        if self.command_thread.isRunning():
            self.command_thread.stop()
            self.command_thread.wait()
        
        # 리스닝 스레드 중지
        if self.listen_thread.isRunning():
            self.listen_thread.stop()
            self.listen_thread.wait()
        
        return True
    
    def is_running(self):
        """서버 실행 여부 반환"""
        return self._running
    
    def is_connected(self):
        """클라이언트 연결 여부 반환"""
        return self.connected
    
    def execute_command(self, command):
        """명령어 실행 (GUI에서 직접 실행)"""
        self.log_message.emit(f"명령 실행 요청: {command}")
        
        if not self.is_connected():
            self.log_message.emit("클라이언트 연결이 없습니다.")
            self.command_executed.emit(command, False)
            return False
        
        success = self.process_command(command)
        return success
    
    def process_command(self, command):
        """명령어 처리 (내부 함수)"""
        if command in self.command_callbacks:
            try:
                callback = self.command_callbacks[command]
                callback()
                self.command_executed.emit(command, True)
                return True
            except Exception as e:
                self._on_error(f"명령 '{command}' 실행 중 오류: {str(e)}")
                self.command_executed.emit(command, False)
                return False
        else:
            self._on_error(f"알 수 없는 명령: {command}")
            self.command_executed.emit(command, False)
            return False
    
    # 내부 이벤트 핸들러
    def _on_server_started(self, message):
        """서버 시작 이벤트 처리"""
        self.server_running = True
        self.status_changed.emit("대기중")
        self.log_message.emit(message)
    
    def _on_server_stopped(self, message):
        """서버 중지 이벤트 처리"""
        self.server_running = False
        self.connected = False
        self.status_changed.emit("중지됨")
        self.log_message.emit(message)
        self.connection_state_changed.emit(False)
    
    def _on_client_accepted(self, client_info):
        """클라이언트 연결 수락 이벤트 처리"""
        client_socket, client_addr = client_info
        
        # 이미 연결된 클라이언트가 있으면 새 연결 거부
        if self.connected:
            try:
                client_socket.send("Server is busy".encode('utf-8'))
                client_socket.close()
                self.log_message.emit(f"{client_addr}의 연결 요청 거부 (이미 연결됨)")
            except:
                pass
            return
        
        # 새 클라이언트 연결 처리
        self.command_thread.setup(client_socket, client_addr)
        self.command_thread.start()
    
    def _on_client_connected(self, message):
        """클라이언트 연결 이벤트 처리"""
        self.connected = True
        self.status_changed.emit("연결됨")
        self.log_message.emit(message)
        self.connection_state_changed.emit(True)
    
    def _on_client_disconnected(self, message):
        """클라이언트 연결 종료 이벤트 처리"""
        self.connected = False
        self.status_changed.emit("대기중")
        self.log_message.emit(message)
        self.connection_state_changed.emit(False)
    
    def _on_command_received(self, command):
        """명령 수신 이벤트 처리"""
        self.log_message.emit(f"명령 수신: {command}")
    
    def _on_response_sent(self, response):
        """응답 전송 이벤트 처리"""
        self.log_message.emit(f"응답 전송: {response}")
    
    def _on_error(self, message):
        """오류 이벤트 처리"""
        self.log_message.emit(f"오류: {message}")
    
    # 명령 콜백 함수들
    def _grip_inner_1(self):
        """내부 그립 1 명령"""
        self.log_message.emit("내부 그립 1 실행")
        # 실제 구현은 여기에
    
    def _grip_inner_2(self):
        """내부 그립 2 명령"""
        self.log_message.emit("내부 그립 2 실행")
        # 실제 구현은 여기에
    
    def _grip_inner_3(self):
        """내부 그립 3 명령"""
        self.log_message.emit("내부 그립 3 실행")
        # 실제 구현은 여기에
    
    def _grip_outer_1(self):
        """외부 그립 1 명령"""
        self.log_message.emit("외부 그립 1 실행")
        # 실제 구현은 여기에
    
    def _grip_outer_2(self):
        """외부 그립 2 명령"""
        self.log_message.emit("외부 그립 2 실행")
        # 실제 구현은 여기에
    
    def _grip_outer_3(self):
        """외부 그립 3 명령"""
        self.log_message.emit("외부 그립 3 실행")
        # 실제 구현은 여기에
    
    def _place_1(self):
        """배치 1 명령"""
        self.log_message.emit("배치 1 실행")
        # 실제 구현은 여기에
    
    def _place_2(self):
        """배치 2 명령"""
        self.log_message.emit("배치 2 실행")
        # 실제 구현은 여기에
    
    def _place_3(self):
        """배치 3 명령"""
        self.log_message.emit("배치 3 실행")
        # 실제 구현은 여기에
    
    def _camera_detection(self):
        """카메라 감지 명령"""
        self.log_message.emit("카메라 감지 실행")
        # 실제 구현은 여기에