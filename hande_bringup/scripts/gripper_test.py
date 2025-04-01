#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import binascii
import struct
import sys
import json
import os
from datetime import datetime

class HandEGripperTester:
    # Modbus RTU 통신 상수
    SLAVE_ID = 0x09
    REG_STATUS = 0x07D0  # 상태 레지스터 시작 주소
    REG_COMMAND = 0x03E8  # 명령 레지스터 시작 주소
    
    # 비트 마스크 (매뉴얼 참조)
    MASK_gACT = 0x01      # 활성화 상태
    MASK_gGTO = 0x02      # 이동 활성화
    MASK_gSTA = 0x0C      # 그리퍼 상태 (2비트)
    MASK_gOBJ = 0x30      # 물체 감지 상태 (2비트)
    MASK_gFLT = 0xC0      # 폴트 상태 (2비트)
    
    # 물체 감지 상태 값
    OBJ_MOVING = 0x00     # 이동 중
    OBJ_DETECTED = 0x01   # 물체 감지됨
    OBJ_MAX_OPEN = 0x02   # 최대 열림
    OBJ_MAX_CLOSE = 0x03  # 최대 닫힘
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.log_data = {
            "timestamp": datetime.now().isoformat(),
            "port": port,
            "baudrate": baudrate,
            "events": []
        }

    def log_event(self, event_type, message, data=None):
        """이벤트 로깅"""
        event = {
            "time": datetime.now().isoformat(),
            "type": event_type,
            "message": message,
        }
        if data:
            event["data"] = data
        
        self.log_data["events"].append(event)
        
        # 콘솔에 출력
        if event_type == "ERROR":
            print(f"\033[91m[ERROR] {message}\033[0m")
        elif event_type == "SUCCESS":
            print(f"\033[92m[SUCCESS] {message}\033[0m")
        elif event_type == "INFO":
            print(f"\033[94m[INFO] {message}\033[0m")
        elif event_type == "DEBUG":
            print(f"\033[90m[DEBUG] {message} {data if data else ''}\033[0m")
        else:
            print(f"[{event_type}] {message}")
    
    def save_log(self, filename=None):
        """로그 데이터를 JSON 파일로 저장"""
        if not filename:
            filename = f"hande_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.log_data, f, indent=2)
        
        print(f"로그가 {filename}에 저장되었습니다.")
    
    def calculate_crc(self, data):
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
    
    def create_command(self, function_code, address, data=None, count=None):
        """Modbus 명령 생성"""
        cmd = bytearray([self.SLAVE_ID, function_code])
        
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
        cmd.extend(self.calculate_crc(cmd))
        
        return cmd
    
    def connect(self):
        """시리얼 포트 연결"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.serial.is_open:
                self.log_event("SUCCESS", f"시리얼 포트 {self.port} 연결 성공")
                return True
            else:
                self.log_event("ERROR", f"시리얼 포트 {self.port} 열기 실패")
                return False
                
        except Exception as e:
            self.log_event("ERROR", f"시리얼 포트 연결 오류: {str(e)}")
            return False
    
    def close(self):
        """시리얼 포트 닫기"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.log_event("INFO", "시리얼 포트 연결 종료")
    
    def read_status(self):
        """그리퍼 상태 읽기"""
        if not self.serial or not self.serial.is_open:
            self.log_event("ERROR", "상태 읽기 실패: 시리얼 포트가 연결되지 않음")
            return None
        
        try:
            # 상태 레지스터 읽기 명령 생성 (3개 레지스터)
            cmd = self.create_command(0x03, self.REG_STATUS, count=3)
            
            self.log_event("DEBUG", "상태 읽기 명령 전송", {"command": binascii.hexlify(cmd).decode()})
            
            # 시리얼 버퍼 비우기
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # 명령 전송
            self.serial.write(cmd)
            
            # 응답 읽기 (11바이트 예상)
            response = self.serial.read(11)
            
            if len(response) != 11:
                self.log_event("ERROR", f"상태 응답 오류: {len(response)}바이트 받음 (11바이트 예상)", 
                              {"response": binascii.hexlify(response).decode() if response else None})
                return None
            
            self.log_event("DEBUG", "상태 응답 수신", {"response": binascii.hexlify(response).decode()})
            
            # 헤더 확인
            if response[0] != self.SLAVE_ID or response[1] != 0x03 or response[2] != 0x06:
                self.log_event("ERROR", "상태 응답 헤더 오류", 
                              {"response": binascii.hexlify(response).decode()})
                return None
            
            # CRC 확인
            received_crc = response[-2:]
            calculated_crc = self.calculate_crc(response[:-2])
            
            if received_crc != calculated_crc:
                self.log_event("ERROR", "상태 응답 CRC 오류", 
                              {"received": binascii.hexlify(received_crc).decode(),
                               "calculated": binascii.hexlify(calculated_crc).decode()})
                return None
            
            # 상태 바이트 파싱
            status_byte = response[3]
            position = response[7]    # 위치 값 (0-255)
            current = response[9]     # 전류 값 (0-255)
            
            # 상태 비트 해석
            activated = bool(status_byte & self.MASK_gACT)
            goto_pos = bool(status_byte & self.MASK_gGTO)
            
            # 그리퍼 상태 (2비트)
            gripper_status = (status_byte & self.MASK_gSTA) >> 2
            
            # 물체 감지 상태 (2비트)
            obj_status = (status_byte & self.MASK_gOBJ) >> 4
            
            # 폴트 상태 (2비트)
            fault_status = (status_byte & self.MASK_gFLT) >> 6
            
            # 물체 감지 여부
            object_detected = (obj_status == self.OBJ_DETECTED)
            
            # 상태 메시지 생성
            status_msg = {
                "activated": activated,
                "goto_position": goto_pos,
                "gripper_status": gripper_status,
                "object_status": obj_status,
                "object_detected": object_detected,
                "fault_status": fault_status,
                "position": position,
                "current": current,
                "raw_status": binascii.hexlify(response).decode()
            }
            
            # 상태 메시지 출력 및 로깅
            status_desc = f"활성화: {'예' if activated else '아니오'}, "
            status_desc += f"이동명령: {'활성' if goto_pos else '비활성'}, "
            status_desc += f"물체감지: {'예' if object_detected else '아니오'}, "
            status_desc += f"폴트: {fault_status}, "
            status_desc += f"위치: {position}/255, "
            status_desc += f"전류: {current}/255"
            
            self.log_event("INFO", f"그리퍼 상태: {status_desc}", status_msg)
            
            return status_msg
            
        except Exception as e:
            self.log_event("ERROR", f"상태 읽기 중 오류 발생: {str(e)}")
            return None
    
    def send_command(self, rACT=1, rGTO=1, rATR=0, rPR=0, rSP=255, rFR=255):
        """그리퍼 명령 전송
        
        매개변수:
        - rACT: 활성화 (0-1)
        - rGTO: 이동 활성화 (0-1)
        - rATR: 자동 해제 (0-1)
        - rPR: 위치 요청 (0-255) - 0: 열림, 255: 닫힘
        - rSP: 속도 (0-255)
        - rFR: 힘 (0-255)
        """
        if not self.serial or not self.serial.is_open:
            self.log_event("ERROR", "명령 전송 실패: 시리얼 포트가 연결되지 않음")
            return False
        
        try:
            # 명령 비트 생성
            cmd_byte = 0
            if rACT:
                cmd_byte |= 0x01
            if rGTO:
                cmd_byte |= 0x02
            if rATR:
                cmd_byte |= 0x04
                
            # 명령 데이터 바이트 생성
            data = bytearray([cmd_byte, 0, 0, rPR & 0xFF, rSP & 0xFF, rFR & 0xFF])
            
            # 명령 생성 (3개 레지스터에 쓰기)
            cmd = self.create_command(0x10, self.REG_COMMAND, data, 3)
            
            cmd_desc = f"활성화: {rACT}, 이동: {rGTO}, 자동해제: {rATR}, "
            cmd_desc += f"위치: {rPR}/255, 속도: {rSP}/255, 힘: {rFR}/255"
            
            self.log_event("INFO", f"명령 전송: {cmd_desc}", 
                          {"command": binascii.hexlify(cmd).decode(),
                           "data": [rACT, rGTO, rATR, rPR, rSP, rFR]})
            
            # 시리얼 버퍼 비우기
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # 명령 전송
            self.serial.write(cmd)
            
            # 응답 읽기 (8바이트 예상)
            response = self.serial.read(8)
            
            if len(response) != 8:
                self.log_event("ERROR", f"명령 응답 오류: {len(response)}바이트 받음 (8바이트 예상)", 
                              {"response": binascii.hexlify(response).decode() if response else None})
                return False
            
            self.log_event("DEBUG", "명령 응답 수신", {"response": binascii.hexlify(response).decode()})
            
            # 헤더 확인
            if response[0] != self.SLAVE_ID or response[1] != 0x10:
                self.log_event("ERROR", "명령 응답 헤더 오류", 
                              {"response": binascii.hexlify(response).decode()})
                return False
            
            # CRC 확인
            received_crc = response[-2:]
            calculated_crc = self.calculate_crc(response[:-2])
            
            if received_crc != calculated_crc:
                self.log_event("ERROR", "명령 응답 CRC 오류", 
                              {"received": binascii.hexlify(received_crc).decode(),
                               "calculated": binascii.hexlify(calculated_crc).decode()})
                return False
            
            self.log_event("SUCCESS", "명령 전송 성공")
            return True
            
        except Exception as e:
            self.log_event("ERROR", f"명령 전송 중 오류 발생: {str(e)}")
            return False
    
    def activate_gripper(self):
        """그리퍼 활성화"""
        self.log_event("INFO", "그리퍼 활성화 시도...")
        return self.send_command(rACT=1, rGTO=0, rATR=0, rPR=0)
    
    def deactivate_gripper(self):
        """그리퍼 비활성화"""
        self.log_event("INFO", "그리퍼 비활성화 시도...")
        return self.send_command(rACT=0, rGTO=0, rATR=0, rPR=0)
    
    def open_gripper(self, speed=255, force=255):
        """그리퍼 열기"""
        self.log_event("INFO", "그리퍼 열기 시도...")
        return self.send_command(rACT=1, rGTO=1, rATR=0, rPR=0, rSP=speed, rFR=force)
    
    def close_gripper(self, speed=255, force=255):
        """그리퍼 닫기"""
        self.log_event("INFO", "그리퍼 닫기 시도...")
        return self.send_command(rACT=1, rGTO=1, rATR=0, rPR=255, rSP=speed, rFR=force)
    
    def set_position(self, position, speed=255, force=255):
        """그리퍼 위치 설정 (0-255)"""
        position = max(0, min(255, position))  # 값 범위 제한
        self.log_event("INFO", f"그리퍼 위치 설정: {position}/255...")
        return self.send_command(rACT=1, rGTO=1, rATR=0, rPR=position, rSP=speed, rFR=force)
    
    def run_test(self):
        """종합 테스트 실행"""
        try:
            self.log_event("INFO", "=== Hand-E 그리퍼 종합 테스트 시작 ===")
            
            # 시리얼 포트 연결
            if not self.connect():
                self.log_event("ERROR", "테스트 중단: 시리얼 포트 연결 실패")
                return False
            
            # 초기 상태 확인
            initial_status = self.read_status()
            if not initial_status:
                self.log_event("ERROR", "테스트 중단: 초기 상태 읽기 실패")
                return False
            
            # 그리퍼가 이미 활성화되어 있는지 확인
            if initial_status["activated"]:
                self.log_event("INFO", "그리퍼가 이미 활성화되어 있습니다.")
            else:
                # 그리퍼 활성화
                if not self.activate_gripper():
                    self.log_event("ERROR", "테스트 중단: 그리퍼 활성화 실패")
                    return False
                
                # 활성화 후 상태 확인
                time.sleep(1)
                status = self.read_status()
                if not status or not status["activated"]:
                    self.log_event("ERROR", "테스트 중단: 그리퍼 활성화 확인 실패")
                    return False
            
            # 그리퍼 열기
            self.log_event("INFO", "테스트 1: 그리퍼 열기")
            if not self.open_gripper():
                self.log_event("ERROR", "테스트 중단: 그리퍼 열기 명령 실패")
                return False
            
            # 상태 확인 및 대기
            time.sleep(2)
            status = self.read_status()
            if not status:
                self.log_event("ERROR", "테스트 중단: 그리퍼 상태 읽기 실패")
                return False
                
            # 그리퍼 닫기
            self.log_event("INFO", "테스트 2: 그리퍼 닫기")
            if not self.close_gripper():
                self.log_event("ERROR", "테스트 중단: 그리퍼 닫기 명령 실패")
                return False
            
            # 상태 확인 및 대기
            time.sleep(2)
            status = self.read_status()
            if not status:
                self.log_event("ERROR", "테스트 중단: 그리퍼 상태 읽기 실패")
                return False
            
            # 50% 위치로 이동
            self.log_event("INFO", "테스트 3: 그리퍼 50% 위치로 이동")
            if not self.set_position(128):
                self.log_event("ERROR", "테스트 중단: 그리퍼 위치 설정 명령 실패")
                return False
            
            # 상태 확인 및 대기
            time.sleep(2)
            status = self.read_status()
            if not status:
                self.log_event("ERROR", "테스트 중단: 그리퍼 상태 읽기 실패")
                return False
                
            # 다양한 위치 테스트 (10%, 30%, 70%, 90%)
            self.log_event("INFO", "테스트 4: 다양한 위치 테스트")
            positions = [25, 77, 179, 230]  # 10%, 30%, 70%, 90%
            
            for i, pos in enumerate(positions):
                self.log_event("INFO", f"테스트 4-{i+1}: 그리퍼 {pos}/255 위치로 이동")
                if not self.set_position(pos):
                    self.log_event("ERROR", f"테스트 중단: 그리퍼 위치 {pos} 설정 명령 실패")
                    return False
                
                # 상태 확인 및 대기
                time.sleep(1.5)
                status = self.read_status()
                if not status:
                    self.log_event("ERROR", f"테스트 중단: 그리퍼 상태 읽기 실패 (위치 {pos})")
                    return False
            
            # 다양한 속도 테스트
            self.log_event("INFO", "테스트 5: 다양한 속도 테스트")
            
            # 저속으로 열기
            self.log_event("INFO", "테스트 5-1: 저속으로 열기")
            if not self.open_gripper(speed=50):
                self.log_event("ERROR", "테스트 중단: 저속 열기 명령 실패")
                return False
            
            # 상태 확인 및 대기
            time.sleep(3)
            status = self.read_status()
            if not status:
                self.log_event("ERROR", "테스트 중단: 그리퍼 상태 읽기 실패")
                return False
            
            # 고속으로 닫기
            self.log_event("INFO", "테스트 5-2: 고속으로 닫기")
            if not self.close_gripper(speed=255):
                self.log_event("ERROR", "테스트 중단: 고속 닫기 명령 실패")
                return False
            
            # 상태 확인 및 대기
            time.sleep(1.5)
            status = self.read_status()
            if not status:
                self.log_event("ERROR", "테스트 중단: 그리퍼 상태 읽기 실패")
                return False
            
            # 최종 테스트: 열기로 마무리
            self.log_event("INFO", "테스트 종료: 그리퍼 열기")
            if not self.open_gripper():
                self.log_event("ERROR", "테스트 중단: 최종 열기 명령 실패")
                return False
            
            # 최종 상태 확인
            time.sleep(2)
            final_status = self.read_status()
            if not final_status:
                self.log_event("ERROR", "테스트 중단: 최종 상태 읽기 실패")
                return False
            
            self.log_event("SUCCESS", "=== 모든 테스트 완료 ===")
            return True
            
        except KeyboardInterrupt:
            self.log_event("INFO", "사용자에 의해 테스트 중단")
            return False
        except Exception as e:
            self.log_event("ERROR", f"테스트 중 예외 발생: {str(e)}")
            return False
        finally:
            self.close()
    
    def enumerate_ports(self):
        """시스템에서 사용 가능한 시리얼 포트 찾기"""
        import serial.tools.list_ports
        
        ports = list(serial.tools.list_ports.comports())
        self.log_event("INFO", f"사용 가능한 시리얼 포트 {len(ports)}개 발견")
        
        for i, port in enumerate(ports):
            self.log_event("INFO", f"포트 {i+1}: {port.device} - {port.description}")
        
        return [port.device for port in ports]

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Robotiq Hand-E 그리퍼 테스트 도구')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0',
                        help='시리얼 포트 (기본값: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='통신 속도 (기본값: 115200)')
    parser.add_argument('-l', '--list-ports', action='store_true',
                        help='사용 가능한 시리얼 포트 나열')
    parser.add_argument('-o', '--output', type=str,
                        help='로그 파일 저장 경로')
    parser.add_argument('-s', '--status', action='store_true',
                        help='그리퍼 상태만 확인')
    parser.add_argument('--open', action='store_true',
                        help='그리퍼 열기')
    parser.add_argument('--close', action='store_true',
                        help='그리퍼 닫기')
    parser.add_argument('--position', type=int,
                        help='그리퍼 위치 설정 (0-255)')
    
    args = parser.parse_args()
    
    tester = HandEGripperTester(port=args.port, baudrate=args.baudrate)
    
    if args.list_ports:
        # 사용 가능한 포트 나열만 하고 종료
        tester.enumerate_ports()
        return
    
    # 접속 시도
    if not tester.connect():
        print("시리얼 포트 연결에 실패했습니다. 포트를 확인하세요.")
        sys.exit(1)
    
    try:
        if args.status:
            # 상태만 확인
            status = tester.read_status()
            if status:
                print("\nHand-E 그리퍼 상태:")
                print(f"  활성화: {'예' if status['activated'] else '아니오'}")
                print(f"  이동명령: {'활성' if status['goto_position'] else '비활성'}")
                print(f"  그리퍼 상태: {status['gripper_status']}")
                print(f"  물체 상태: {status['object_status']} " + 
                      f"({'감지됨' if status['object_detected'] else '감지 안됨'})")
                print(f"  폴트 상태: {status['fault_status']}")
                print(f"  위치: {status['position']} / 255")
                print(f"  전류: {status['current']} / 255")
            else:
                print("그리퍼 상태를 읽을 수 없습니다.")
        elif args.open:
            # 그리퍼 열기
            print("그리퍼 열기 명령 전송...")
            success = tester.open_gripper()
            print(f"명령 {'성공' if success else '실패'}")
        elif args.close:
            # 그리퍼 닫기
            print("그리퍼 닫기 명령 전송...")
            success = tester.close_gripper()
            print(f"명령 {'성공' if success else '실패'}")
        elif args.position is not None:
            # 특정 위치로 이동
            position = max(0, min(255, args.position))
            print(f"그리퍼 위치 {position}/255 설정 명령 전송...")
            success = tester.set_position(position)
            print(f"명령 {'성공' if success else '실패'}")
        else:
            # 종합 테스트 실행
            tester.run_test()
    finally:
        # 연결 종료 및 로그 저장
        tester.close()
        if args.output:
            tester.save_log(args.output)
        else:
            tester.save_log()

if __name__ == "__main__":
    main()