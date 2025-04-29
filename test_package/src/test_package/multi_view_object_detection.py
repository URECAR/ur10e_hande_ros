#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import cv2
import copy
import time
import threading
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf.transformations

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QGroupBox, QGridLayout, QPushButton, QFrame, QLineEdit,
                            QCheckBox, QDoubleSpinBox, QSpinBox, QMessageBox, QProgressBar,
                            QTabWidget, QListWidget, QComboBox)
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread

import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN

class ModelLoadingThread(QThread):
    """STL 모델을 비동기적으로 로드하는 스레드"""
    finished = pyqtSignal(dict)
    progress = pyqtSignal(int)
    error = pyqtSignal(str)
    
    def __init__(self, model_path):
        super().__init__()
        self.model_path = model_path
    
    def run(self):
        try:
            models = {}
            
            # 단일 STL 파일 경로인 경우
            if self.model_path.endswith('.stl'):
                model_name = os.path.basename(self.model_path).split('.')[0]
                mesh = o3d.io.read_triangle_mesh(self.model_path)
                # 샘플링된 포인트 클라우드 생성
                pcd = mesh.sample_points_uniformly(number_of_points=5000)
                models[model_name] = pcd
                self.progress.emit(100)
            else:
                # 디렉토리인 경우 모든 STL 파일 찾기
                stl_files = [f for f in os.listdir(self.model_path) if f.endswith('.stl')]
                
                for i, filename in enumerate(stl_files):
                    model_name = os.path.splitext(filename)[0]
                    model_full_path = os.path.join(self.model_path, filename)
                    
                    mesh = o3d.io.read_triangle_mesh(model_full_path)
                    # 샘플링된 포인트 클라우드 생성
                    pcd = mesh.sample_points_uniformly(number_of_points=5000)
                    models[model_name] = pcd
                    
                    # 진행 상황 업데이트
                    progress = int((i + 1) / len(stl_files) * 100)
                    self.progress.emit(progress)
            
            self.finished.emit(models)
        except Exception as e:
            self.error.emit(f"모델 로딩 오류: {str(e)}")


class PointCloudCaptureThread(QThread):
    """포인트 클라우드 캡처 및 처리 스레드"""
    progress = pyqtSignal(int, str)
    finished = pyqtSignal(list)
    error = pyqtSignal(str)
    
    def __init__(self, robot_controller, camera_points_topic, x_min, x_max, y_min, y_max, z_min, z_max):
        super().__init__()
        self.robot_controller = robot_controller
        self.camera_points_topic = camera_points_topic
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        
        # 포인트 클라우드 저장 변수
        self.point_clouds = []
        self.camera_positions = []
        
        # TF 관련 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def capture_point_cloud(self):
        """현재 위치에서 포인트 클라우드 캡처"""
        try:
            # 포인트 클라우드 토픽 메시지 수신 대기
            rospy.loginfo("포인트 클라우드 수신 대기 중...")
            point_cloud_msg = rospy.wait_for_message(self.camera_points_topic, PointCloud2, timeout=5.0)
            
            # 카메라에서 베이스로의 변환 가져오기
            transform = self.tf_buffer.lookup_transform("base_link", point_cloud_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            
            # 변환 행렬 생성
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # 쿼터니언을 회전 행렬로 변환
            quaternion = [rot.x, rot.y, rot.z, rot.w]
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
            
            # 변환 행렬 완성
            transform_matrix = np.identity(4)
            transform_matrix[:3, :3] = rotation_matrix[:3, :3]
            transform_matrix[0, 3] = trans.x
            transform_matrix[1, 3] = trans.y
            transform_matrix[2, 3] = trans.z
            
            # PointCloud2를 Open3D 포인트 클라우드로 변환
            points_list = []
            for p in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append(p)
            
            if len(points_list) == 0:
                rospy.logwarn("수신된 포인트 클라우드가 비어있습니다")
                return None
                
            # Open3D 포인트 클라우드 생성
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(points_list))
            
            # 월드 좌표계(베이스 링크)로 변환된 포인트 클라우드 반환
            pcd_transformed = pcd.transform(transform_matrix)
            
            # 현재 카메라 위치 저장
            camera_position = np.array([trans.x, trans.y, trans.z])
            
            # 지정된 영역으로 포인트 클라우드 필터링
            points = np.asarray(pcd_transformed.points)
            x_mask = np.logical_and(points[:, 0] >= self.x_min, points[:, 0] <= self.x_max)
            y_mask = np.logical_and(points[:, 1] >= self.y_min, points[:, 1] <= self.y_max)
            z_mask = np.logical_and(points[:, 2] >= self.z_min, points[:, 2] <= self.z_max)
            mask = np.logical_and(np.logical_and(x_mask, y_mask), z_mask)
            
            filtered_points = points[mask]
            
            if len(filtered_points) == 0:
                rospy.logwarn("지정된 영역 내에 포인트가 없습니다")
                return None
            
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
            
            return {
                'pcd': filtered_pcd,
                'camera_position': camera_position
            }
            
        except Exception as e:
            rospy.logerr(f"포인트 클라우드 캡처 오류: {str(e)}")
            return None
    
    def move_to_view_position(self, view_index):
        """다양한 관점을 위한 로봇 이동 - Z축 중심 기울임 촬영 (40도 기울기)"""
        try:
            # 영역의 중심 계산
            offset_x = -0.050
            center_x = (self.x_min + self.x_max) / 2 + offset_x
            center_y = (self.y_min + self.y_max) / 2 
            center_z = (self.z_min + self.z_max) / 2
            
            # 카메라 높이 - 중심보다 약간 위
            camera_height = center_z + 0.45
            
            # 관찰 각도 (도)
            tilt_angle = 40  # 기울기 각도
            
            # sin, cos 값 계산
            import math
            sin_tilt = math.sin(math.radians(tilt_angle))
            cos_tilt = math.cos(math.radians(tilt_angle))
            
            # 카메라와 중심 사이의 수평 거리
            radius = 0.3  # 중심으로부터 30cm 거리
            
            # 4개의 관점 위치 계산 (모두 위에서 내려다보는 각도)
            view_positions = [
                # 맨 위에서 보기 (정수직)
                [center_x, center_y, camera_height, 180, 0, -90],
                
                # 위에서 앞으로 40도 기울여 보기
                [center_x - radius * sin_tilt, center_y, camera_height - radius * (1 - cos_tilt), 180 + tilt_angle, 0, -90],
                
                # 위에서 오른쪽으로 40도 기울여 보기
                [center_x, center_y - radius * sin_tilt, camera_height - radius * (1 - cos_tilt), 180, tilt_angle, -90],
                
                # 위에서 왼쪽으로 40도 기울여 보기
                [center_x, center_y + radius * sin_tilt, camera_height - radius * (1 - cos_tilt), 180, -tilt_angle, -90]
            ]
            
            if view_index < 0 or view_index >= len(view_positions):
                self.error.emit(f"올바르지 않은 뷰 인덱스: {view_index}")
                return False
            
            # 선택된 위치로 로봇 이동
            target_pose = view_positions[view_index]
            
            # 위치 변환 (m → mm)
            x_mm = target_pose[0] * 1000
            y_mm = target_pose[1] * 1000
            z_mm = target_pose[2] * 1000
            rx = target_pose[3]
            ry = target_pose[4]
            rz = target_pose[5]
            
            # MoveIt을 사용하여 로봇 이동
            pose_values = [x_mm, y_mm, z_mm, rx, ry, rz]
            
            # 이동 전 상태 업데이트
            self.progress.emit(view_index * 25, f"위치 {view_index+1}로 이동 중...")
            
            # TCP 이동 계획
            velocity_scaling = 0.3  # 30% 속도
            acceleration_scaling = 0.2  # 20% 가속도
            
            # 중요: 시작 상태 재설정
            try:
                # 현재 로봇 상태 업데이트 - 이것이 중요!
                self.robot_controller.move_group.set_start_state_to_current_state()
                
                # 잠시 대기하여 상태 업데이트 확인
                rospy.sleep(0.5)
            except Exception as e:
                rospy.logwarn(f"로봇 상태 업데이트 오류 (무시): {e}")
            
            # 계획 시도
            self.robot_controller.plan_pose_movement(pose_values, velocity_scaling, acceleration_scaling)
            time.sleep(1.0)  # 계획 완료 대기
            
            # 계획 실행 시도
            try:
                self.robot_controller.execute_plan()
                time.sleep(3.0)  # 이동 완료 대기
            except Exception as exec_error:
                # 실행 실패 시
                rospy.logerr(f"실행 오류: {exec_error}")
                self.error.emit(f"로봇 이동 실행 오류: {str(exec_error)}")
                return False
            
            return True
            
        except Exception as e:
            self.error.emit(f"로봇 이동 오류: {str(e)}")
            return False

    def process_multi_view(self):
        """물체 감지 프로세스 실행 - 평면 기반 클러스터링 방식"""
        try:
            # 진행 상황 업데이트
            # ... 나머지 코드
            # 진행 상황 업데이트
            self.progress.emit(10, "포인트 클라우드 결합 중...")
            
            if not self.point_clouds:
                self.error.emit("포인트 클라우드가 없습니다.")
                return []
            
            # 개별적으로 각 포인트 클라우드에서 평면 위 물체 추출
            objects_from_views = []
            
            for i, point_cloud in enumerate(self.point_clouds):
                self.progress.emit(10 + i * 10, f"시점 {i+1} 처리 중...")
                
                # 각 시점에서 평면 분할 및 물체 추출
                try:
                    # 다운샘플링을 통한 노이즈 제거 (remove_statistical_outliers 대체)
                    point_cloud_downsampled = point_cloud.voxel_down_sample(voxel_size=0.005)
                    
                    # 평면 분할
                    plane_model, inliers = point_cloud_downsampled.segment_plane(
                        distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                    
                    # 평면 위 물체 추출 (평면이 아닌 점)
                    objects = point_cloud_downsampled.select_by_index(inliers, invert=True)
                    
                    # 평면 높이 계산 (평면 방정식: ax + by + cz + d = 0)
                    a, b, c, d = plane_model
                    rospy.loginfo(f"평면 방정식: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
                    
                    # 평면의 높이를 기준으로 필터링
                    points = np.asarray(objects.points)
                    if len(points) == 0:
                        rospy.logwarn(f"시점 {i+1}에서 평면 위 포인트가 없습니다")
                        continue
                    
                    # 각 점의 평면으로부터의 높이 계산
                    heights = np.abs(np.dot(points, [a, b, c]) + d) / np.sqrt(a*a + b*b + c*c)
                    
                    # 일정 높이 이상인 점만 선택 (평면 위에 있는 객체)
                    height_threshold = 0.015  # 1.5cm 이상 높이의 점만 선택
                    height_mask = heights > height_threshold
                    
                    # 높이 마스크 결과 로깅
                    rospy.loginfo(f"시점 {i+1}: 전체 포인트 {len(points)}개, 높이 임계값 {height_threshold}m 초과 포인트 {np.sum(height_mask)}개")
                    
                    if np.sum(height_mask) < 30:
                        rospy.logwarn(f"시점 {i+1}에서 높이 임계값을 초과하는 포인트가 충분하지 않습니다: {np.sum(height_mask)}개")
                        continue
                    
                    filtered_points = points[height_mask]
                    
                    # 새 포인트 클라우드 생성
                    filtered_objects = o3d.geometry.PointCloud()
                    filtered_objects.points = o3d.utility.Vector3dVector(filtered_points)
                    
                    # 이 시점의 물체 포인트 저장
                    objects_from_views.append(filtered_objects)
                    rospy.loginfo(f"시점 {i+1}에서 물체 포인트 {len(filtered_points)}개 추출됨")
                    
                except Exception as e:
                    rospy.logwarn(f"시점 {i+1} 처리 오류 (계속 진행): {e}")
                    import traceback
                    rospy.logwarn(traceback.format_exc())
                    continue
            
            # 모든 시점에서 추출한 물체 포인트 결합
            self.progress.emit(50, "모든 시점의 물체 결합 중...")
            
            if not objects_from_views:
                self.error.emit("어떤 시점에서도 물체를 찾을 수 없습니다.")
                return []
            
            # 물체 포인트 개수 기록
            for i, obj in enumerate(objects_from_views):
                rospy.loginfo(f"시점 {i+1}의 물체 포인트: {len(obj.points)}개")
            
            # 모든 물체 포인트 결합
            combined_objects = o3d.geometry.PointCloud()
            for obj in objects_from_views:
                combined_objects += obj
                
            rospy.loginfo(f"결합된 총 물체 포인트: {len(combined_objects.points)}개")
            
            # 중복 점 제거를 위한 다운샘플링
            combined_objects = combined_objects.voxel_down_sample(voxel_size=0.005)
            rospy.loginfo(f"다운샘플링 후 물체 포인트: {len(combined_objects.points)}개")
            
            # DBSCAN 클러스터링으로 개별 물체 분리
            self.progress.emit(60, "물체 클러스터링 중...")
            
            # 클러스터링 매개변수 조정
            labels = np.array(combined_objects.cluster_dbscan(
                eps=0.03,  # 3cm 클러스터링 거리 (이전: 5cm)
                min_points=20))  # 최소 포인트 수 감소 (이전: 30)
            
            # 클러스터링 결과 로깅
            if labels.max() < 0:
                rospy.logwarn("클러스터링된 물체가 없습니다. DBSCAN에서 클러스터를 찾지 못했습니다.")
                # 클러스터링 실패 시 추가 디버깅 정보
                unique, counts = np.unique(labels, return_counts=True)
                for val, count in zip(unique, counts):
                    rospy.loginfo(f"클러스터 ID {val}: {count}개 포인트")
                self.error.emit("클러스터링된 물체가 없습니다.")
                return []
            else:
                rospy.loginfo(f"감지된 클러스터 수: {labels.max() + 1}")
                unique, counts = np.unique(labels, return_counts=True)
                for val, count in zip(unique, counts):
                    rospy.loginfo(f"클러스터 ID {val}: {count}개 포인트")
                    
            # 결과 저장 목록
            detected_objects = []
            
            # 진행 상황 업데이트
            self.progress.emit(70, f"{labels.max() + 1}개 물체 분석 중...")
            
            # 각 클러스터를 개별 물체로 처리
            for i in range(labels.max() + 1):
                # 현재 클러스터에 해당하는 포인트 인덱스 찾기
                cluster_indices = np.where(labels == i)[0]
                
                # 충분한 포인트가 있는지 확인
                if len(cluster_indices) < 20:  # 최소 20개 포인트 필요
                    continue
                    
                # 클러스터 포인트 클라우드 추출
                cluster = combined_objects.select_by_index(cluster_indices)
                
                # 물체의 중심점 계산
                center = np.mean(np.asarray(cluster.points), axis=0)
                
                # 물체의 주축 방향 계산 (PCA 사용)
                points = np.asarray(cluster.points)
                points_centered = points - center
                
                # 공분산 행렬 계산
                cov = np.cov(points_centered, rowvar=False)
                
                # 고유값 분해
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                
                # 고유값이 큰 순서대로 정렬
                idx = eigenvalues.argsort()[::-1]
                eigenvalues = eigenvalues[idx]
                eigenvectors = eigenvectors[:, idx]
                
                # 주축 방향 (최대 분산 방향)
                primary_axis = eigenvectors[:, 0]
                secondary_axis = eigenvectors[:, 1]
                
                # XY 평면에서의 방향 계산 (Z축 회전)
                direction_xy = np.array([primary_axis[0], primary_axis[1], 0])
                if np.linalg.norm(direction_xy) > 1e-6:  # 0으로 나누기 방지
                    direction_xy = direction_xy / np.linalg.norm(direction_xy)
                    z_rotation = np.arctan2(direction_xy[1], direction_xy[0])
                    z_rotation_deg = np.degrees(z_rotation)
                else:
                    z_rotation_deg = 0
                
                # X, Y축 회전 계산
                x_rotation_deg = np.degrees(np.arctan2(primary_axis[2], np.sqrt(primary_axis[0]**2 + primary_axis[1]**2)))
                y_rotation_deg = np.degrees(np.arctan2(-primary_axis[0], primary_axis[1]))
                
                # 모델과 정합 시도
                model_match, match_score, transform = None, 0, None
                
                if self.reference_models:
                    # 첫 번째 모델 사용 (단일 STL 모델 가정)
                    model_name = list(self.reference_models.keys())[0]
                    model = self.reference_models[model_name]
                    
                    try:
                        # 클러스터와 모델 정합
                        # 기본 정렬 - 클러스터 중심에 모델 배치
                        init_transform = np.eye(4)
                        init_transform[:3, 3] = center
                        
                        # 대략적인 방향 정합
                        r = Rotation.from_euler('xyz', [x_rotation_deg, 0, z_rotation_deg], degrees=True)
                        init_transform[:3, :3] = r.as_matrix()
                        
                        # 복제 모델 생성 및 변환
                        model_copy = copy.deepcopy(model)
                        model_copy.transform(init_transform)
                        
                        # ICP로 정밀 정합
                        icp_result = o3d.pipelines.registration.registration_icp(
                            cluster, model_copy, 0.05, np.eye(4),
                            o3d.pipelines.registration.TransformationEstimationPointToPoint())
                        
                        # 결과 변환 행렬 (초기 변환 + ICP 변환)
                        final_transform = icp_result.transformation @ init_transform
                        match_score = icp_result.fitness
                        
                        if match_score > 0.5:  # 일정 기준 이상 매칭되면 저장
                            model_match = model_name
                            transform = final_transform
                        
                    except Exception as e:
                        rospy.logwarn(f"물체 {i} 모델 정합 오류 (무시): {e}")
                
                # 추가 속성 계산
                bbox = cluster.get_axis_aligned_bounding_box()
                bbox_size = bbox.get_extent()
                
                # 객체 정보 저장
                obj_info = {
                    'id': i,
                    'center': center,
                    'orientation': [x_rotation_deg, 0, z_rotation_deg],  # X, Y, Z축 회전
                    'size': bbox_size,
                    'points': np.asarray(cluster.points),
                    'point_count': len(cluster.points)
                }
                
                # 모델 매칭 결과가 있으면 추가
                if model_match and transform is not None:
                    # 최종 방향 계산 (변환 행렬에서)
                    rotation_matrix = transform[:3, :3]
                    r = Rotation.from_matrix(rotation_matrix)
                    euler_angles = r.as_euler('xyz', degrees=True)
                    
                    obj_info.update({
                        'model': model_match,
                        'score': match_score,
                        'transform': transform,
                        'final_orientation': euler_angles
                    })
                
                detected_objects.append(obj_info)
                
                # 진행 상황 업데이트
                progress = 70 + (i + 1) / (labels.max() + 1) * 30
                self.progress.emit(int(progress), f"물체 {i+1}/{labels.max() + 1} 분석 완료")
            
            # 결과 반환
            return detected_objects
            
        except Exception as e:
            self.error.emit(f"물체 감지 오류: {str(e)}")
            import traceback
            rospy.logerr(f"상세 오류: {traceback.format_exc()}")
            return []

    def run(self):
        """다양한 뷰에서 포인트 클라우드 캡처"""
        try:
            # 이전에 캡처된 데이터 초기화
            self.point_clouds = []
            self.camera_positions = []
            
            # 4개의 뷰 위치에서 캡처
            for view_index in range(4):
                # 해당 뷰 위치로 이동
                if not self.move_to_view_position(view_index):
                    continue
                
                # 안정화를 위한 대기
                time.sleep(1.0)
                
                # 포인트 클라우드 캡처
                result = self.capture_point_cloud()
                if result:
                    self.point_clouds.append(result['pcd'])
                    self.camera_positions.append(result['camera_position'])
                    self.progress.emit(view_index * 25 + 25, f"위치 {view_index+1} 캡처 완료")
                else:
                    self.progress.emit(view_index * 25 + 25, f"위치 {view_index+1} 캡처 실패")
            
            # 캡처된 데이터가 있는지 확인
            if len(self.point_clouds) > 0:
                # 모든 캡처 완료
                self.progress.emit(100, "모든 뷰 캡처 완료")
                self.finished.emit(self.point_clouds)
            else:
                self.error.emit("캡처된 포인트 클라우드가 없습니다")
                    
        except Exception as e:
            self.error.emit(f"포인트 클라우드 캡처 프로세스 오류: {str(e)}")

class ObjectDetectionThread(QThread):
    """물체 감지 및 정합 스레드"""
    progress = pyqtSignal(int, str)
    finished = pyqtSignal(list)
    error = pyqtSignal(str)
    
    def __init__(self, point_clouds, reference_models, x_min, x_max, y_min, y_max, z_min, z_max):
        super().__init__()
        self.point_clouds = point_clouds
        self.reference_models = reference_models
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        
        # 감지 매개변수
        self.height_threshold = 0.01  # 평면 위로 돌출된 객체의 최소 높이
        self.cluster_distance = 0.02  # 클러스터링 거리
        self.min_points = 30  # 클러스터의 최소 포인트 수
        self.icp_threshold = 0.005  # ICP 정합 임계값
        self.matching_threshold = 0.6  # 매칭 임계값 (0-1)
    
    def combine_point_clouds(self):
        """여러 포인트 클라우드를 하나로 결합"""
        if not self.point_clouds:
            return None
            
        combined = o3d.geometry.PointCloud()
        
        for pcd in self.point_clouds:
            combined += pcd
            
        # 다운샘플링 적용
        return combined.voxel_down_sample(voxel_size=0.002)
    
    def segment_plane(self, point_cloud):
        """평면 분할하여 물체 추출"""
        # RANSAC 알고리즘을 사용하여 평면 분할
        plane_model, inliers = point_cloud.segment_plane(
            distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        
        # 평면 외 포인트 추출 (물체)
        objects = point_cloud.select_by_index(inliers, invert=True)
        
        # 평면 매개변수
        a, b, c, d = plane_model
        
        return objects, plane_model
    
    def cluster_objects(self, point_cloud):
        """포인트 클라우드에서 개별 물체 클러스터링 - Open3D 호환성 수정"""
        try:
            # 구 버전과 신 버전 모두 호환되도록 예외 처리
            try:
                # 최신 버전 Open3D 방식으로 시도
                filtered, _ = point_cloud.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)
            except AttributeError:
                # 구 버전 Open3D 대응
                filtered = copy.deepcopy(point_cloud)
                # 간단한 다운샘플링으로 대체
                filtered = filtered.voxel_down_sample(voxel_size=0.005)
                rospy.logwarn("remove_statistical_outliers 함수를 사용할 수 없어 다운샘플링으로 대체합니다.")
            
            # DBSCAN 클러스터링 적용
            labels = np.array(filtered.cluster_dbscan(
                eps=self.cluster_distance, min_points=self.min_points))
            
            # 클러스터 없음
            if labels.max() < 0:
                return []
                
            # 각 클러스터를 개별 포인트 클라우드로 분리
            clusters = []
            for i in range(labels.max() + 1):
                # 현재 클러스터에 해당하는 포인트 인덱스 찾기
                cluster_indices = np.where(labels == i)[0]
                
                # 충분한 포인트가 있는지 확인
                if len(cluster_indices) < self.min_points:
                    continue
                    
                # 클러스터 포인트 클라우드 추출
                cluster = filtered.select_by_index(cluster_indices)
                
                # 클러스터 추가
                clusters.append(cluster)
                
            return clusters
        except Exception as e:
            rospy.logerr(f"클러스터링 오류: {str(e)}")
            return []
    
    def preprocess_point_cloud(self, pcd, voxel_size=0.005):
        """포인트 클라우드 전처리 (다운샘플링 및 특징 계산)"""
        # 다운샘플링
        pcd_down = pcd.voxel_down_sample(voxel_size)
        
        # 법선 계산
        radius_normal = voxel_size * 2
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        
        # FPFH 특징 계산
        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        return pcd_down, pcd_fpfh
    
    def match_with_model(self, object_cloud, reference_models):
        """물체를 참조 모델과 매칭"""
        best_match = None
        best_score = 0
        best_transform = None
        
        # 물체가 충분한 포인트를 가지고 있는지 확인
        if len(object_cloud.points) < self.min_points:
            return None, 0, None
        
        # 전처리 및 특징 계산
        object_down, object_fpfh = self.preprocess_point_cloud(object_cloud)
        
        # 각 참조 모델과 비교
        for model_name, model_cloud in reference_models.items():
            # 모델 복제
            model = copy.deepcopy(model_cloud)
            
            # 모델 전처리
            model_down, model_fpfh = self.preprocess_point_cloud(model)
            
            # RANSAC 기반 전역 등록 (초기 정렬)
            result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                object_down, model_down, object_fpfh, model_fpfh, 
                mutual_filter=True,
                max_correspondence_distance=0.05,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                ransac_n=4,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.05)
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 500))
                
            # 전역 정합이 실패하면 다음 모델로
            if result_ransac.fitness < 0.1:
                continue
                
            # ICP로 지역 정합하여 정밀도 향상
            result_icp = o3d.pipelines.registration.registration_icp(
                object_down, model_down, self.icp_threshold, result_ransac.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
                
            # 매칭 점수 계산
            score = result_icp.fitness / (1 + result_icp.inlier_rmse)
            
            if score > best_score:
                best_score = score
                best_match = model_name
                best_transform = result_icp.transformation
        
        return best_match, best_score, best_transform

    def on_detection_thread_finished(self, detected_objects):
        """ObjectDetectionThread에서 사용해야 할 수정된 콜백"""
        # ObjectDetectionThread 클래스에서 process_multi_view 대신 다음 코드 추가
        def run(self):
            """물체 감지 스레드 실행"""
            try:
                # 물체 감지 수행
                detected_objects = self.process_multi_view()
                
                # 결과 반환
                self.finished.emit(detected_objects)
                
            except Exception as e:
                self.error.emit(f"물체 감지 오류: {str(e)}")

    
    def run(self):
        """물체 감지 프로세스 실행"""
        try:
            # 진행 상황 업데이트
            self.progress.emit(10, "포인트 클라우드 결합 중...")
            
            # 포인트 클라우드 결합
            combined_cloud = self.combine_point_clouds()
            if combined_cloud is None:
                self.error.emit("유효한 포인트 클라우드가 없습니다")
                return
                
            # 진행 상황 업데이트
            self.progress.emit(30, "평면 분할 중...")
            
            # 평면 분할하여 물체 추출
            objects_cloud, plane_model = self.segment_plane(combined_cloud)
            
            # 진행 상황 업데이트
            self.progress.emit(50, "물체 클러스터링 중...")
            
            # 개별 물체로 클러스터링
            object_clusters = self.cluster_objects(objects_cloud)
            
            if not object_clusters:
                self.error.emit("감지된 물체가 없습니다")
                return
                
            # 결과 저장 목록
            detected_objects = []
            
            # 진행 상황 업데이트
            self.progress.emit(70, f"{len(object_clusters)}개 물체 분석 중...")
            
            # 각 클러스터에 대해 참조 모델과 매칭
            for i, cluster in enumerate(object_clusters):
                # 모델 매칭
                model_name, score, transform = self.match_with_model(cluster, self.reference_models)
                
                # 충분한 유사성이 있는 경우에만 추가
                if model_name and score > self.matching_threshold:
                    # 위치 추출 (변환 행렬의 이동 부분)
                    position = transform[:3, 3]
                    
                    # 방향 추출 (변환 행렬의 회전 부분)
                    rotation_matrix = transform[:3, :3]
                    r = Rotation.from_matrix(rotation_matrix)
                    euler_angles = r.as_euler('xyz', degrees=True)
                    
                    # 객체 정보 저장
                    detected_objects.append({
                        'id': i,
                        'model': model_name,
                        'score': score,
                        'position': position,
                        'orientation': euler_angles,
                        'transform': transform,
                        'points': np.asarray(cluster.points)
                    })
                    
                    # 진행 상황 업데이트
                    progress = 70 + (i + 1) / len(object_clusters) * 20
                    self.progress.emit(int(progress), f"물체 {i+1}/{len(object_clusters)} 분석 완료")
            
            # 결과 전송
            self.progress.emit(100, f"{len(detected_objects)}개 물체 감지 완료")
            self.finished.emit(detected_objects)
            
        except Exception as e:
            self.error.emit(f"물체 감지 오류: {str(e)}")


class MultiViewObjectDetectionTab(QWidget):
    """다중 시점 물체 감지 탭"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 기본 변수 초기화
        self.robot_controller = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        
        # 참조 모델 경로
        self.model_path = "/root/catkin_ws/src/UR_Control/ur10e_hande/test_package/models/target.stl"
        
        # 참조 모델 저장소
        self.reference_models = {}
        
        # 감지된 물체 목록
        self.detected_objects = []
        
        # 캡처된 포인트 클라우드
        self.point_clouds = []
        
        # 마커 발행자
        self.object_markers_pub = rospy.Publisher('/detected_objects', MarkerArray, queue_size=1)
        
        # UI 초기화
        self.init_ui()
        
        # 모델 로드 시작
        self.load_reference_models()
    
    def init_ui(self):
        """UI 초기화"""
        main_layout = QVBoxLayout(self)
        
        # 상태 표시줄
        status_frame = QFrame()
        status_frame.setFrameShape(QFrame.StyledPanel)
        status_layout = QHBoxLayout(status_frame)
        
        self.status_label = QLabel("준비")
        self.status_label.setStyleSheet("font-weight: bold;")
        status_layout.addWidget(self.status_label)
        
        main_layout.addWidget(status_frame)
        
        # 영역 설정 그룹
        region_group = QGroupBox("감지 영역 설정")
        region_layout = QGridLayout()
        
        # X 범위 (첫 번째 줄)
        region_layout.addWidget(QLabel("X 범위 (m):"), 0, 0)
        self.x_min_input = QDoubleSpinBox()
        self.x_min_input.setRange(-10.0, 10.0)
        self.x_min_input.setValue(0.46)
        self.x_min_input.setSingleStep(0.01)
        region_layout.addWidget(self.x_min_input, 0, 1)
        
        region_layout.addWidget(QLabel("~"), 0, 2)
        
        self.x_max_input = QDoubleSpinBox()
        self.x_max_input.setRange(-10.0, 10.0)
        self.x_max_input.setValue(0.63)
        self.x_max_input.setSingleStep(0.01)
        region_layout.addWidget(self.x_max_input, 0, 3)
        
        # Y 범위 (두 번째 줄)
        region_layout.addWidget(QLabel("Y 범위 (m):"), 1, 0)
        self.y_min_input = QDoubleSpinBox()
        self.y_min_input.setRange(-10.0, 10.0)
        self.y_min_input.setValue(-0.21)
        self.y_min_input.setSingleStep(0.01)
        region_layout.addWidget(self.y_min_input, 1, 1)
        
        region_layout.addWidget(QLabel("~"), 1, 2)
        
        self.y_max_input = QDoubleSpinBox()
        self.y_max_input.setRange(-10.0, 10.0)
        self.y_max_input.setValue(0.31)
        self.y_max_input.setSingleStep(0.01)
        region_layout.addWidget(self.y_max_input, 1, 3)
        
        # Z 범위 (세 번째 줄)
        region_layout.addWidget(QLabel("Z 범위 (m):"), 2, 0)
        self.z_min_input = QDoubleSpinBox()
        self.z_min_input.setRange(-10.0, 10.0)
        self.z_min_input.setValue(0.14)
        self.z_min_input.setSingleStep(0.01)
        region_layout.addWidget(self.z_min_input, 2, 1)
        
        region_layout.addWidget(QLabel("~"), 2, 2)
        
        self.z_max_input = QDoubleSpinBox()
        self.z_max_input.setRange(-10.0, 10.0)
        self.z_max_input.setValue(0.26)
        self.z_max_input.setSingleStep(0.01)
        region_layout.addWidget(self.z_max_input, 2, 3)
        
        region_group.setLayout(region_layout)
        main_layout.addWidget(region_group)
        
        # 컨트롤 그룹
        control_group = QGroupBox("작업 제어")
        control_layout = QVBoxLayout()
        
        # 진행 상황 바
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        control_layout.addWidget(self.progress_bar)
        
        # 버튼 레이아웃
        button_layout = QHBoxLayout()
        
        # 캡처 버튼
        self.capture_button = QPushButton("다중 시점 캡처")
        self.capture_button.clicked.connect(self.start_capture)
        button_layout.addWidget(self.capture_button)
        
        # 감지 버튼
        self.detect_button = QPushButton("물체 감지")
        self.detect_button.clicked.connect(self.start_detection)
        self.detect_button.setEnabled(False)
        button_layout.addWidget(self.detect_button)
        
        control_layout.addLayout(button_layout)
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # 결과 그룹
        results_group = QGroupBox("감지 결과")
        results_layout = QVBoxLayout()
        
        # 감지된 물체 목록
        self.results_list = QListWidget()
        self.results_list.itemClicked.connect(self.on_result_selected)
        results_layout.addWidget(self.results_list)
        
        # 물체 이동 버튼
        self.move_to_object_button = QPushButton("선택한 물체로 이동")
        self.move_to_object_button.clicked.connect(self.move_to_selected_object)
        self.move_to_object_button.setEnabled(False)
        results_layout.addWidget(self.move_to_object_button)
        
        results_group.setLayout(results_layout)
        main_layout.addWidget(results_group)
    
    def set_robot_controller(self, controller):
        """로봇 컨트롤러 설정"""
        self.robot_controller = controller
        
        if controller:
            self.status_label.setText("로봇 컨트롤러 연결됨")
            self.capture_button.setEnabled(True)
        else:
            self.status_label.setText("로봇 컨트롤러 연결 필요")
            self.capture_button.setEnabled(False)
    
    def load_reference_models(self):
        """참조 STL 모델 로드 또는 생성"""
        self.status_label.setText("모델 로딩 중...")
        
        try:
            # 모델 로드 시도
            try:
                # STL 파일 존재 확인
                if os.path.exists(self.model_path):
                    self.model_loading_thread = ModelLoadingThread(self.model_path)
                    self.model_loading_thread.progress.connect(self.update_model_loading_progress)
                    self.model_loading_thread.finished.connect(self.on_models_loaded)
                    self.model_loading_thread.error.connect(self.on_model_loading_error)
                    self.model_loading_thread.start()
                    return
                else:
                    rospy.logwarn(f"STL 파일이 존재하지 않습니다: {self.model_path}")
            except Exception as e:
                rospy.logwarn(f"모델 로드 스레드 시작 오류: {e}")
            
            # STL 로드 실패 시 직접 모델 생성
            rospy.loginfo("STL 로드 실패, 원통 모델 직접 생성")
            self.reference_models = create_cylinder_model()
            self.progress_bar.setValue(100)
            self.status_label.setText("원통 모델 생성 완료")
            self.capture_button.setEnabled(True)
            
        except Exception as e:
            error_msg = f"모델 생성 오류: {str(e)}"
            self.status_label.setText(f"오류: {error_msg}")
            QMessageBox.warning(self, "모델 생성 오류", error_msg)
            self.capture_button.setEnabled(False)
    def update_model_loading_progress(self, progress):
        """모델 로딩 진행 상황 업데이트"""
        self.progress_bar.setValue(progress)
        self.status_label.setText(f"모델 로딩 중... {progress}%")
    
    def on_models_loaded(self, models):
        """모델 로딩 완료 처리"""
        self.reference_models = models
        model_count = len(models)
        
        if model_count > 0:
            self.status_label.setText(f"{model_count}개 모델 로드 완료")
            self.capture_button.setEnabled(True)
        else:
            self.status_label.setText("모델 로드 실패")
            self.capture_button.setEnabled(False)
    
    def on_model_loading_error(self, error_message):
        """모델 로딩 오류 처리"""
        self.status_label.setText(f"오류: {error_message}")
        QMessageBox.warning(self, "모델 로딩 오류", error_message)
        self.capture_button.setEnabled(False)
    
    def start_capture(self):
        """다중 시점 캡처 시작"""
        if not self.robot_controller:
            QMessageBox.warning(self, "오류", "로봇 컨트롤러가 연결되지 않았습니다.")
            return
        
        # 영역 정보 가져오기
        x_min = self.x_min_input.value()
        x_max = self.x_max_input.value()
        y_min = self.y_min_input.value()
        y_max = self.y_max_input.value()
        z_min = self.z_min_input.value()
        z_max = self.z_max_input.value()
        
        # 캡처 버튼 비활성화
        self.capture_button.setEnabled(False)
        self.detect_button.setEnabled(False)
        self.status_label.setText("캡처 준비 중...")
        
        # 캡처 스레드 생성 및 시작
        self.capture_thread = PointCloudCaptureThread(
            self.robot_controller,
            '/camera/depth/color/points',
            x_min, x_max, y_min, y_max, z_min, z_max
        )
        self.capture_thread.progress.connect(self.update_capture_progress)
        self.capture_thread.finished.connect(self.on_capture_finished)
        self.capture_thread.error.connect(self.on_capture_error)
        self.capture_thread.start()
    
    def update_capture_progress(self, progress, message):
        """캡처 진행 상황 업데이트"""
        self.progress_bar.setValue(progress)
        self.status_label.setText(message)
    
    def on_capture_finished(self, point_clouds):
        """캡처 완료 처리"""
        self.point_clouds = point_clouds
        
        if len(point_clouds) > 0:
            self.status_label.setText(f"{len(point_clouds)}개 시점 캡처 완료")
            self.capture_button.setEnabled(True)
            self.detect_button.setEnabled(True)
        else:
            self.status_label.setText("캡처 실패")
            self.capture_button.setEnabled(True)
            self.detect_button.setEnabled(False)
    
    def on_capture_error(self, error_message):
        """캡처 오류 처리"""
        self.status_label.setText(f"오류: {error_message}")
        QMessageBox.warning(self, "캡처 오류", error_message)
        self.capture_button.setEnabled(True)
    
    def start_detection(self):
        """물체 감지 시작"""
        if not self.point_clouds:
            QMessageBox.warning(self, "오류", "캡처된 포인트 클라우드가 없습니다.")
            return
            
        if not self.reference_models:
            QMessageBox.warning(self, "오류", "참조 모델이 로드되지 않았습니다.")
            return
        
        # 영역 정보 가져오기
        x_min = self.x_min_input.value()
        x_max = self.x_max_input.value()
        y_min = self.y_min_input.value()
        y_max = self.y_max_input.value()
        z_min = self.z_min_input.value()
        z_max = self.z_max_input.value()
        
        # 버튼 비활성화
        self.capture_button.setEnabled(False)
        self.detect_button.setEnabled(False)
        self.status_label.setText("물체 감지 중...")
        
        # 감지 스레드 생성 및 시작
        self.detection_thread = ObjectDetectionThread(
            self.point_clouds,
            self.reference_models,
            x_min, x_max, y_min, y_max, z_min, z_max
        )
        self.detection_thread.progress.connect(self.update_detection_progress)
        self.detection_thread.finished.connect(self.on_detection_finished)
        self.detection_thread.error.connect(self.on_detection_error)
        self.detection_thread.start()
    
    def update_detection_progress(self, progress, message):
        """감지 진행 상황 업데이트"""
        self.progress_bar.setValue(progress)
        self.status_label.setText(message)
    
    def on_detection_finished(self, detected_objects):
        """감지 완료 처리"""
        self.detected_objects = detected_objects
        
        # 버튼 활성화
        self.capture_button.setEnabled(True)
        self.detect_button.setEnabled(True)
        
        # 결과 목록 업데이트
        self.update_results_list()
        
        # 마커 발행
        self.publish_object_markers(detected_objects)
        
        if len(detected_objects) > 0:
            self.status_label.setText(f"{len(detected_objects)}개 물체 감지 완료")
        else:
            self.status_label.setText("감지된 물체 없음")
    
    def on_detection_error(self, error_message):
        """감지 오류 처리"""
        self.status_label.setText(f"오류: {error_message}")
        QMessageBox.warning(self, "감지 오류", error_message)
        
        # 버튼 활성화
        self.capture_button.setEnabled(True)
        self.detect_button.setEnabled(True)
    
    def update_results_list(self):
        """감지 결과 목록 업데이트 - 평면 기반 감지 결과"""
        self.results_list.clear()
        
        for i, obj in enumerate(self.detected_objects):
            # 기본 정보 포맷팅
            center = obj['center']
            point_count = obj['point_count']
            
            # 방향 정보 포맷팅
            if 'final_orientation' in obj:
                orientation = obj['final_orientation']
                orientation_str = f"[{orientation[0]:.1f}°, {orientation[1]:.1f}°, {orientation[2]:.1f}°]"
            else:
                orientation = obj['orientation']
                orientation_str = f"[{orientation[0]:.1f}°, 0.0°, {orientation[2]:.1f}°]"
            
            # 크기 정보
            size = obj['size']
            
            # 모델 매칭 정보
            if 'model' in obj:
                model_info = f"모델: {obj['model']} (점수: {obj['score']:.3f})"
            else:
                model_info = "모델 매칭 없음"
            
            # 항목 텍스트 생성
            item_text = f"물체 {i+1}: {point_count}개 포인트 {model_info}\n"
            item_text += f"  위치: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]\n"
            item_text += f"  방향: {orientation_str}\n"
            item_text += f"  크기: [{size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}]"
            
            # 목록에 항목 추가
            self.results_list.addItem(item_text)

    
    def on_result_selected(self, item):
        """결과 목록에서 항목 선택 처리"""
        if self.robot_controller:
            self.move_to_object_button.setEnabled(True)
    
    def move_to_selected_object(self):
        """선택한 물체 위치로 로봇 이동"""
        selected_items = self.results_list.selectedItems()
        if not selected_items:
            return
            
        # 선택된 항목의 인덱스 찾기
        selected_index = self.results_list.row(selected_items[0])
        
        if selected_index < 0 or selected_index >= len(self.detected_objects):
            return
            
        # 선택된 물체 정보
        selected_object = self.detected_objects[selected_index]
        
        # 물체 위치 (미터)
        obj_x = selected_object['position'][0]
        obj_y = selected_object['position'][1]
        obj_z = selected_object['position'][2] + 0.1  # 물체 위 10cm
        
        # MoveIt에 전달하기 위해 밀리미터 단위로 변환
        x_mm = obj_x * 1000
        y_mm = obj_y * 1000
        z_mm = obj_z * 1000
        
        # 기본 방향 (물체 위에서 내려다보는 방향)
        rx, ry, rz = 180.0, 0.0, -90.0
        
        # 이동 실행
        try:
            # 로봇 제어기에 이동 명령 전송
            pose_values = [x_mm, y_mm, z_mm, rx, ry, rz]
            
            # 속도 및 가속도 설정
            velocity_scaling = 0.3  # 30% 속도
            acceleration_scaling = 0.2  # 20% 가속도
            
            # 계획 생성
            self.status_label.setText("물체 위치로 이동 계획 중...")
            self.robot_controller.plan_pose_movement(pose_values, velocity_scaling, acceleration_scaling)
            
            # 잠시 대기
            time.sleep(1.0)
            
            # 계획 실행
            self.status_label.setText("물체 위치로 이동 중...")
            self.robot_controller.execute_plan()
            
            self.status_label.setText(f"물체 {selected_index+1} 위치로 이동 완료")
            
        except Exception as e:
            self.status_label.setText(f"이동 오류: {str(e)}")
            QMessageBox.warning(self, "이동 오류", str(e))
    
    def publish_object_markers(self, objects):
        """감지된 물체에 대한 시각화 마커 발행 - 방향 포함"""
        marker_array = MarkerArray()
        
        # 기존 마커 삭제 마커 추가
        delete_marker = Marker()
        delete_marker.header.frame_id = "base_link"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "detected_objects"
        delete_marker.id = 9999  # 특수 ID
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 각 물체마다 마커 생성
        for i, obj in enumerate(objects):
            # 1. 중심점 마커
            center_marker = Marker()
            center_marker.header.frame_id = "base_link"
            center_marker.header.stamp = rospy.Time.now()
            center_marker.ns = "detected_objects"
            center_marker.id = i * 3  # 각 물체마다 여러 마커를 사용하므로 ID 간격을 둠
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            
            # 위치 설정
            center = obj['center']
            center_marker.pose.position.x = center[0]
            center_marker.pose.position.y = center[1]
            center_marker.pose.position.z = center[2]
            
            # 방향 설정 (기본값)
            center_marker.pose.orientation.w = 1.0
            
            # 크기 설정
            center_marker.scale.x = 0.03
            center_marker.scale.y = 0.03
            center_marker.scale.z = 0.03
            
            # 색상 설정 (빨간색)
            center_marker.color.r = 1.0
            center_marker.color.g = 0.0
            center_marker.color.b = 0.0
            center_marker.color.a = 0.8
            
            marker_array.markers.append(center_marker)
            
            # 2. 방향 화살표 마커
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "base_link"
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "detected_objects"
            arrow_marker.id = i * 3 + 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # 위치 설정
            arrow_marker.pose.position.x = center[0]
            arrow_marker.pose.position.y = center[1]
            arrow_marker.pose.position.z = center[2]
            
            # 방향 설정
            if 'final_orientation' in obj:
                orientation = obj['final_orientation']
            else:
                orientation = obj['orientation']
                
            # 오일러 각도를 쿼터니언으로 변환
            quat = tf.transformations.quaternion_from_euler(
                np.radians(orientation[0]),
                np.radians(orientation[1]),
                np.radians(orientation[2])
            )
            
            arrow_marker.pose.orientation.x = quat[0]
            arrow_marker.pose.orientation.y = quat[1]
            arrow_marker.pose.orientation.z = quat[2]
            arrow_marker.pose.orientation.w = quat[3]
            
            # 크기 설정 (화살표 길이와 너비)
            arrow_marker.scale.x = 0.1  # 화살표 길이
            arrow_marker.scale.y = 0.01  # 화살표 머리 너비
            arrow_marker.scale.z = 0.01  # 화살표 머리 높이
            
            # 색상 설정 (녹색)
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 0.8
            
            marker_array.markers.append(arrow_marker)
            
            # 3. 경계 상자 마커
            if 'size' in obj:
                bbox_marker = Marker()
                bbox_marker.header.frame_id = "base_link"
                bbox_marker.header.stamp = rospy.Time.now()
                bbox_marker.ns = "detected_objects"
                bbox_marker.id = i * 3 + 2
                bbox_marker.type = Marker.CUBE
                bbox_marker.action = Marker.ADD
                
                # 위치 설정
                bbox_marker.pose.position.x = center[0]
                bbox_marker.pose.position.y = center[1]
                bbox_marker.pose.position.z = center[2]
                
                # 방향 설정 - 화살표와 동일한 방향
                bbox_marker.pose.orientation.x = quat[0]
                bbox_marker.pose.orientation.y = quat[1]
                bbox_marker.pose.orientation.z = quat[2]
                bbox_marker.pose.orientation.w = quat[3]
                
                # 크기 설정
                size = obj['size']
                bbox_marker.scale.x = size[0]
                bbox_marker.scale.y = size[1]
                bbox_marker.scale.z = size[2]
                
                # 색상 설정 (반투명 파란색)
                bbox_marker.color.r = 0.0
                bbox_marker.color.g = 0.0
                bbox_marker.color.b = 1.0
                bbox_marker.color.a = 0.3
                
                marker_array.markers.append(bbox_marker)
        
        # 마커 발행
        self.object_markers_pub.publish(marker_array)