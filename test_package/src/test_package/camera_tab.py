#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                           QGroupBox, QGridLayout, QPushButton, QFrame)
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint, QSize, QRect


class ImageDisplayWidget(QLabel):
    """이미지 표시 및 마우스 이벤트 처리 위젯"""
    mouse_moved = pyqtSignal(QPoint)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setFixedSize(640, 480)  # 이미지 크기 고정
        self.setStyleSheet("background-color: black;")
        self.setMouseTracking(True)  # 마우스 움직임 추적 활성화
        self.setCursor(Qt.CrossCursor)  # 십자형 커서로 변경
        self.scaled_image_width = 0
        self.scaled_image_height = 0
        self.image_offset_x = 0
        self.image_offset_y = 0
        self.original_width = 0
        self.original_height = 0
    
    def setPixmap(self, pixmap):
        """픽스맵 설정 및 표시 영역 계산"""
        if pixmap:
            self.original_width = pixmap.width()
            self.original_height = pixmap.height()
            widget_rect = self.rect()
            scaled_pixmap = pixmap.scaled(
                widget_rect.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.scaled_image_width = scaled_pixmap.width()
            self.scaled_image_height = scaled_pixmap.height()
            self.image_offset_x = (widget_rect.width() - self.scaled_image_width) // 2
            self.image_offset_y = (widget_rect.height() - self.scaled_image_height) // 2
            super().setPixmap(scaled_pixmap)
        else:
            super().setPixmap(pixmap)
    
    def mouseMoveEvent(self, event):
        """마우스 움직임 이벤트 처리"""
        self.mouse_moved.emit(event.pos())
        super().mouseMoveEvent(event)
    
    def getImageCoordinates(self, mouse_pos):
        """마우스 화면 좌표를 이미지 원본 좌표로 변환"""
        if self.scaled_image_width == 0 or self.scaled_image_height == 0:
            return None
        image_x = mouse_pos.x() - self.image_offset_x
        image_y = mouse_pos.y() - self.image_offset_y
        if (image_x < 0 or image_x >= self.scaled_image_width or
            image_y < 0 or image_y >= self.scaled_image_height):
            return None
        original_x = int(image_x * self.original_width / self.scaled_image_width)
        original_y = int(image_y * self.original_height / self.scaled_image_height)
        return (original_x, original_y)


class CameraTab(QWidget):
    """카메라 탭 위젯 - 개선된 버전"""
    
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.last_mouse_pos = QPoint(0, 0)
        self.pixel_depth = None
        self.world_point = None
        
        # 이미지 표시 모드 (0: 컬러, 1: 깊이)
        self.display_mode = 0
        
        self.init_ui()
        self.setup_subscribers()
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(100)
        rospy.loginfo("카메라 탭 초기화 완료 - 개선된 인터페이스")
    
    def init_ui(self):
        """UI 초기화"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 이미지 표시 그룹
        image_group = QGroupBox("카메라 이미지")
        image_layout = QVBoxLayout()
        
        # 이미지 표시 위젯
        self.image_view = ImageDisplayWidget()
        self.image_view.mouse_moved.connect(self.on_mouse_moved)
        image_layout.addWidget(self.image_view)
        
        # 이미지 라벨 (현재 표시 중인 이미지 타입)
        self.image_type_label = QLabel("컬러 이미지")
        self.image_type_label.setStyleSheet("font-weight: bold; color: #3366cc;")
        self.image_type_label.setAlignment(Qt.AlignCenter)
        image_layout.addWidget(self.image_type_label)
        
        image_group.setLayout(image_layout)
        main_layout.addWidget(image_group)
        
        # 좌표 및 제어 패널 (하단에 한 줄로 배치)
        control_frame = QFrame()
        control_frame.setFrameShape(QFrame.StyledPanel)
        control_frame.setFrameShadow(QFrame.Raised)
        control_layout = QHBoxLayout(control_frame)
        control_layout.setContentsMargins(5, 5, 5, 5)
        
        # 이미지 좌표 표시
        self.img_coord_label = QLabel("이미지: (-,-)")
        control_layout.addWidget(self.img_coord_label)
        
        # 월드 좌표 표시
        self.world_coord_label = QLabel("월드: (-,-,-)")
        self.world_coord_label.setStyleSheet("font-weight: bold;")
        control_layout.addWidget(self.world_coord_label)
        
        # 깊이 값 표시
        self.depth_label = QLabel("깊이: - mm")
        control_layout.addWidget(self.depth_label)
        
        # 공간 확보를 위한 스페이서
        control_layout.addStretch(1)
        
        # 필터 변환 버튼
        self.filter_button = QPushButton("깊이 이미지 보기")
        self.filter_button.clicked.connect(self.toggle_display_mode)
        control_layout.addWidget(self.filter_button)
        
        main_layout.addWidget(control_frame)
        
        # 전체 크기 설정
        self.setMinimumSize(660, 600)
        self.setMaximumSize(800, 700)
    
    def setup_subscribers(self):
        """ROS 토픽 구독 설정"""
        rospy.loginfo("구독할 카메라 토픽:")
        rospy.loginfo("  컬러: /camera/color/image_raw")
        rospy.loginfo("  깊이: /camera/aligned_depth_to_color/image_raw")
        rospy.loginfo("  정보: /camera/color/camera_info")
        
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_callback)
        self.frame_count = 0
        self.last_fps_time = rospy.Time.now()
    
    def color_callback(self, msg):
        """컬러 이미지 콜백"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1
            current_time = rospy.Time.now()
            time_diff = (current_time - self.last_fps_time).to_sec()
            if time_diff > 1.0:
                self.frame_count = 0
                self.last_fps_time = current_time
        except CvBridgeError as e:
            rospy.logerr(f"컬러 이미지 변환 오류: {e}")
    
    def depth_callback(self, msg):
        """정렬된 깊이 이미지 콜백"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.update_depth_value()
        except CvBridgeError as e:
            rospy.logerr(f"깊이 이미지 변환 오류: {e}")
    
    def info_callback(self, msg):
        """카메라 정보 콜백"""
        self.camera_info = msg
    
    def on_mouse_moved(self, pos):
        """마우스 이동 이벤트 처리"""
        self.last_mouse_pos = pos
        QTimer.singleShot(0, self.update_depth_value)
    
    def toggle_display_mode(self):
        """이미지 표시 모드 전환"""
        self.display_mode = 1 - self.display_mode  # 0과 1 사이 전환
        
        if self.display_mode == 0:
            self.image_type_label.setText("컬러 이미지")
            self.filter_button.setText("깊이 이미지 보기")
        else:
            self.image_type_label.setText("깊이 이미지")
            self.filter_button.setText("컬러 이미지 보기")
    
    def pixel_to_world(self, x, y, depth):
        """픽셀 좌표와 깊이를 월드 좌표로 변환"""
        if self.camera_info is None or depth <= 0:
            return None
        try:
            k_matrix = np.array(self.camera_info.K).reshape(3, 3)
            fx = k_matrix[0, 0]
            fy = k_matrix[1, 1]
            cx = k_matrix[0, 2]
            cy = k_matrix[1, 2]
            z = depth
            x_cam = (x - cx) * z / fx
            y_cam = (y - cy) * z / fy
            camera_point = PointStamped()
            camera_point.header.frame_id = self.camera_info.header.frame_id
            camera_point.header.stamp = rospy.Time.now()
            camera_point.point.x = z
            camera_point.point.y = -x_cam
            camera_point.point.z = -y_cam
            try:
                world_point = self.tf_buffer.transform(camera_point, "base_link", rospy.Duration(0.1))
                return world_point.point
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                return camera_point.point
        except Exception as e:
            rospy.logerr(f"좌표 변환 오류: {e}")
            return None
    
    def update_depth_value(self):
        """마우스 위치의 깊이 값 및 월드 좌표 업데이트"""
        if self.depth_image is None or self.color_image is None:
            return
        img_coords = self.image_view.getImageCoordinates(self.last_mouse_pos)
        if img_coords is None:
            self.pixel_depth = None
            self.world_point = None
            self.world_coord_label.setText("월드: (-,-,-)")
            self.img_coord_label.setText("이미지: (-,-)")
            self.depth_label.setText("깊이: - mm")
            return
        img_x, img_y = img_coords
        if (img_x < 0 or img_x >= self.depth_image.shape[1] or
            img_y < 0 or img_y >= self.depth_image.shape[0]):
            self.pixel_depth = None
            self.world_point = None
            self.world_coord_label.setText("월드: (-,-,-)")
            self.img_coord_label.setText(f"이미지: ({img_x},{img_y})")
            self.depth_label.setText("깊이: - mm")
            return
        depth_val = self.depth_image[img_y, img_x]
        depth_m = depth_val / 1000.0 if depth_val > 0 else 0.0
        if depth_m > 0 and depth_m < 10:
            self.pixel_depth = depth_m
            self.img_coord_label.setText(f"이미지: ({img_x},{img_y})")
            self.depth_label.setText(f"깊이: {int(depth_val)} mm")
            
            world_point = self.pixel_to_world(img_x, img_y, depth_m)
            self.world_point = world_point
            if world_point:
                x_mm = world_point.x * 1000
                y_mm = world_point.y * 1000
                z_mm = world_point.z * 1000
                self.world_coord_label.setText(f"월드: ({x_mm:.0f},{y_mm:.0f},{z_mm:.0f})mm")
            else:
                self.world_coord_label.setText("월드: 변환 실패")
        else:
            self.pixel_depth = None
            self.world_point = None
            self.world_coord_label.setText("월드: (-,-,-)")
            self.img_coord_label.setText(f"이미지: ({img_x},{img_y})")
            self.depth_label.setText("깊이: - mm")
            
    def update_displays(self):
        """디스플레이 업데이트"""
        if self.display_mode == 0 and self.color_image is not None:
            # 컬러 이미지 표시
            rgb_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_view.setPixmap(pixmap)
            
        elif self.display_mode == 1 and self.depth_image is not None:
            # 깊이 이미지 표시
            try:
                depth_norm = self.depth_image.astype(np.float32) / 1000.0
                mask = (depth_norm > 0.1) & (depth_norm < 5.0)
                depth_norm_masked = np.zeros_like(depth_norm)
                depth_norm_masked[mask] = depth_norm[mask]
                min_depth = 0.1
                max_depth = 5.0
                normalized = np.zeros_like(depth_norm_masked)
                normalized[mask] = ((depth_norm_masked[mask] - min_depth) / (max_depth - min_depth) * 255)
                depth_visualized = normalized.astype(np.uint8)
                depth_colormap = cv2.applyColorMap(depth_visualized, cv2.COLORMAP_JET)
                h, w, ch = depth_colormap.shape
                bytes_per_line = ch * w
                qt_depth = QImage(depth_colormap.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_depth)
                self.image_view.setPixmap(pixmap)
            except Exception as e:
                rospy.logerr(f"깊이 이미지 시각화 오류: {e}")
    
    def closeEvent(self, event):
        """닫기 이벤트 처리"""
        if hasattr(self, 'color_sub'):
            self.color_sub.unregister()
        if hasattr(self, 'depth_sub'):
            self.depth_sub.unregister()
        if hasattr(self, 'info_sub'):
            self.info_sub.unregister()
        if self.update_timer.isActive():
            self.update_timer.stop()