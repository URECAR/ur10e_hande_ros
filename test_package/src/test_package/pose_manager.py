#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import PoseManager directly from this module
# This will make "from test_package.pose_manager import PoseManager" work

import os
import json
import rospy
from PyQt5.QtCore import QObject, pyqtSignal

class PoseManager(QObject):
    """로봇 포즈 관리 클래스"""
    
    # 신호 정의
    pose_list_updated = pyqtSignal(list)  # 포즈 목록이 업데이트되면 발생
    
    def __init__(self, config_dir=None):
        super().__init__()
        
        # 설정 파일 경로 설정
        if config_dir is None:
            # ROS 패키지 경로 사용
            package_path = os.path.expanduser('~/.ros/ur_poses')
            if not os.path.exists(package_path):
                os.makedirs(package_path)
            self.config_dir = package_path
        else:
            self.config_dir = config_dir
        
        self.config_file = os.path.join(self.config_dir, 'poses.json')
        
        # 포즈 저장소
        self.poses = {}
        
        # 설정 파일 로드
        self.load_poses()
        
        rospy.loginfo(f"포즈 관리자 초기화 완료: {len(self.poses)} 포즈 로드됨")
    
    def load_poses(self):
        """설정 파일에서 포즈 로드"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    self.poses = json.load(f)
                rospy.loginfo(f"포즈 로드 성공: {self.config_file}")
                # 기존 파일이 없거나 비어있으면 기본 포즈 추가
                if not self.poses:
                    self._add_default_poses()
            else:
                rospy.loginfo(f"포즈 파일 없음, 기본 포즈 생성: {self.config_file}")
                self._add_default_poses()
                self.save_poses()  # 기본 포즈 저장
        except Exception as e:
            rospy.logerr(f"포즈 로드 오류: {str(e)}")
            self._add_default_poses()  # 오류 발생시 기본 포즈 설정
        
        # 포즈 목록 업데이트 신호 발생
        self.pose_list_updated.emit(list(self.poses.keys()))
    
    def _add_default_poses(self):
        """기본 포즈 추가"""
        self.poses = {
            "홈 포지션": {
                "type": "joint",
                "values": [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
            },
            "대기 위치": {
                "type": "joint",
                "values": [45.0, -90.0, 90.0, -90.0, -45.0, 0.0]
            },
            "테이블 위 20cm": {
                "type": "tcp",
                "values": [400.0, 0.0, 200.0, 180.0, 0.0, 90.0]
            }
        }
    
    def save_poses(self):
        """포즈를 설정 파일에 저장"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.poses, f, ensure_ascii=False, indent=2)
            rospy.loginfo(f"포즈 저장 성공: {self.config_file}")
            return True
        except Exception as e:
            rospy.logerr(f"포즈 저장 오류: {str(e)}")
            return False
    
    def get_pose_names(self):
        """저장된 모든 포즈 이름 목록 반환"""
        return list(self.poses.keys())
    
    def get_pose(self, name):
        """지정된 이름의 포즈 반환"""
        if name in self.poses:
            return self.poses[name]
        return None
    
    def add_pose(self, name, pose_type, values):
        """새 포즈 추가"""
        if not name or name.strip() == "":
            return False, "포즈 이름을 입력해주세요."
        
        # 포즈 타입 확인
        if pose_type not in ["joint", "tcp"]:
            return False, "포즈 타입은 'joint' 또는 'tcp'여야 합니다."
        
        # 값 개수 확인
        if len(values) != 6:
            return False, "포즈 값은 6개여야 합니다."
        
        # 포즈 추가
        self.poses[name] = {
            "type": pose_type,
            "values": values
        }
        
        # 저장 및 알림
        success = self.save_poses()
        if success:
            self.pose_list_updated.emit(list(self.poses.keys()))
            return True, f"포즈 '{name}' 추가 성공"
        else:
            return False, "포즈 저장 실패"
    
    def update_pose(self, name, pose_type=None, values=None):
        """기존 포즈 업데이트"""
        if name not in self.poses:
            return False, f"포즈 '{name}'가 존재하지 않습니다."
        
        if pose_type is not None:
            self.poses[name]["type"] = pose_type
        
        if values is not None:
            self.poses[name]["values"] = values
        
        # 저장 및 알림
        success = self.save_poses()
        if success:
            self.pose_list_updated.emit(list(self.poses.keys()))
            return True, f"포즈 '{name}' 업데이트 성공"
        else:
            return False, "포즈 저장 실패"
    
    def delete_pose(self, name):
        """포즈 삭제"""
        if name not in self.poses:
            return False, f"포즈 '{name}'가 존재하지 않습니다."
        
        del self.poses[name]
        
        # 저장 및 알림
        success = self.save_poses()
        if success:
            self.pose_list_updated.emit(list(self.poses.keys()))
            return True, f"포즈 '{name}' 삭제 성공"
        else:
            return False, "포즈 저장 실패"
    
    def rename_pose(self, old_name, new_name):
        """포즈 이름 변경"""
        if old_name not in self.poses:
            return False, f"포즈 '{old_name}'가 존재하지 않습니다."
        
        if new_name in self.poses:
            return False, f"이미 '{new_name}' 포즈가 존재합니다."
        
        if not new_name or new_name.strip() == "":
            return False, "새 포즈 이름을 입력해주세요."
        
        # 포즈 복사 후 이름 변경
        self.poses[new_name] = self.poses[old_name]
        del self.poses[old_name]
        
        # 저장 및 알림
        success = self.save_poses()
        if success:
            self.pose_list_updated.emit(list(self.poses.keys()))
            return True, f"포즈 이름 변경 성공: '{old_name}' → '{new_name}'"
        else:
            return False, "포즈 저장 실패"