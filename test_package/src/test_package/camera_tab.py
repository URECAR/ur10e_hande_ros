#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                           QGroupBox, QGridLayout, QScrollArea)
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint, QSize, QRect


class ImageDisplayWidget(QLabel):
    """이미지 표시 및 마우스 이벤트 처리 위젯"""
    mouse_moved = pyqtSignal(QPoint)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(400, 300)
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
            # 원본 이미지 크기 저장
            self.original_width = pixmap.width()
            self.original_height = pixmap.height()
            
            # 위젯 크기에 맞게 비율 유지하며 스케일링
            widget_rect = self.rect()
            scaled_pixmap = pixmap.scaled(
                widget_rect.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # 스케일링된 이미지 크기 저장
            self.scaled_image_width = scaled_pixmap.width()
            self.scaled_image_height = scaled_pixmap.height()
            
            # 중앙 정렬 오프셋 계산
            self.image_offset_x = (widget_rect.width() - self.scaled_image_width) // 2
            self.image_offset_y = (widget_rect.height() - self.scaled_image_height) // 2
            
            # 부모 클래스의 setPixmap 호출
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
        
        # 마우스 위치를 스케일링된 이미지 좌표계로 변환
        image_x = mouse_pos.x() - self.image_offset_x
        image_y = mouse_pos.y() - self.image_offset_y
        
        # 이미지 영역 내에 있는지 확인
        if (image_x < 0 or image_x >= self.scaled_image_width or
            image_y < 0 or image_y >= self.scaled_image_height):
            return None
        
        # 스케일링된 좌표를 원본 이미지 좌표로 변환
        original_x = int(image_x * self.original_width / self.scaled_image_width)
        original_y = int(image_y * self.original_height / self.scaled_image_height)
        
        return (original_x, original_y)


class CameraTab(QWidget):
    """카메라 탭 위젯"""
    
    def __init__(self):
        super().__init__()
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 이미지 및 정보 저장 변수
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        
        # 마지막 마우스 위치
        self.last_mouse_pos = QPoint(0, 0)
        self.pixel_depth = None
        
        # UI 초기화
        self.init_ui()
        
        # ROS 구독 설정
        self.setup_subscribers()
        
        # 타이머 설정 (이미지 업데이트용)
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(100)  # 10Hz
        
        rospy.loginfo("카메라 탭 초기화 완료 - 정렬된 깊이 이미지 사용")
    
    def init_ui(self):
        """UI 초기화"""
        main_layout = QVBoxLayout(self)
        
        # 화면 분할 레이아웃
        split_layout = QHBoxLayout()
        
        # 왼쪽: 이미지 표시 영역
        image_layout = QVBoxLayout()
        
        # 컬러 이미지 그룹
        color_group = QGroupBox("컬러 이미지")
        color_layout = QVBoxLayout()
        self.color_view = ImageDisplayWidget()
        self.color_view.mouse_moved.connect(self.on_mouse_moved)
        color_layout.addWidget(self.color_view)
        color_group.setLayout(color_layout)
        image_layout.addWidget(color_group)
        
        # 깊이 이미지 그룹
        depth_group = QGroupBox("깊이 이미지")
        depth_layout = QVBoxLayout()
        self.depth_view = QLabel()
        self.depth_view.setAlignment(Qt.AlignCenter)
        self.depth_view.setMinimumSize(400, 300)
        self.depth_view.setStyleSheet("background-color: black;")
        depth_layout.addWidget(self.depth_view)
        depth_group.setLayout(depth_layout)
        image_layout.addWidget(depth_group)
        
        split_layout.addLayout(image_layout, 3)  # 3:1 비율로 표시
        
        # 오른쪽: 정보 표시 영역
        info_layout = QVBoxLayout()
        
        # 마우스 정보 그룹
        mouse_group = QGroupBox("포인터 정보")
        mouse_layout = QGridLayout()
        
        mouse_layout.addWidget(QLabel("좌표:"), 0, 0)
        self.mouse_pos_label = QLabel("(-, -)")
        mouse_layout.addWidget(self.mouse_pos_label, 0, 1)
        
        mouse_layout.addWidget(QLabel("깊이:"), 1, 0)
        self.depth_value_label = QLabel("-")
        self.depth_value_label.setStyleSheet("font-weight: bold; color: blue;")
        mouse_layout.addWidget(self.depth_value_label, 1, 1)
        
        # 컬러 값 표시 추가
        mouse_layout.addWidget(QLabel("컬러:"), 2, 0)
        self.color_value_label = QLabel("RGB: (-,-,-)")
        mouse_layout.addWidget(self.color_value_label, 2, 1)
        
        mouse_group.setLayout(mouse_layout)
        info_layout.addWidget(mouse_group)
        
        # 카메라 정보 그룹
        camera_info_group = QGroupBox("카메라 정보")
        camera_info_layout = QVBoxLayout()
        
        # 스크롤 영역 추가
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QGridLayout(scroll_content)
        
        # 카메라 정보 레이블들
        self.resolution_label = QLabel("해상도: -")
        scroll_layout.addWidget(self.resolution_label, 0, 0)
        
        self.frame_id_label = QLabel("프레임 ID: -")
        scroll_layout.addWidget(self.frame_id_label, 1, 0)
        
        self.distortion_label = QLabel("왜곡 계수: -")
        scroll_layout.addWidget(self.distortion_label, 2, 0)
        
        self.k_label = QLabel("K 행렬: -")
        scroll_layout.addWidget(self.k_label, 3, 0)
        
        self.p_label = QLabel("P 행렬: -")
        scroll_layout.addWidget(self.p_label, 4, 0)
        
        self.fps_label = QLabel("FPS: -")
        scroll_layout.addWidget(self.fps_label, 5, 0)
        
        # 좌표 변환 정보 추가
        self.alignment_label = QLabel("정렬 상태: -")
        scroll_layout.addWidget(self.alignment_label, 6, 0)
        
        scroll_content.setLayout(scroll_layout)
        scroll_area.setWidget(scroll_content)
        camera_info_layout.addWidget(scroll_area)
        
        camera_info_group.setLayout(camera_info_layout)
        info_layout.addWidget(camera_info_group)
        
        split_layout.addLayout(info_layout, 1)
        
        main_layout.addLayout(split_layout)
    
    def setup_subscribers(self):
        """ROS 토픽 구독 설정"""
        rospy.loginfo("구독할 카메라 토픽:")
        rospy.loginfo("  컬러: /camera/color/image_raw")
        rospy.loginfo("  깊이: /camera/aligned_depth_to_color/image_raw")
        rospy.loginfo("  정보: /camera/color/camera_info")
        
        # 컬러 이미지 구독
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        
        # 컬러 이미지에 정렬된 깊이 이미지 구독
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        
        # 카메라 정보 구독
        self.info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_callback)
        
        # 프레임 카운터 및 시간 측정 변수
        self.frame_count = 0
        self.last_fps_time = rospy.Time.now()
    
    def color_callback(self, msg):
        """컬러 이미지 콜백"""
        try:
            # 이미지 변환
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # FPS 계산
            self.frame_count += 1
            current_time = rospy.Time.now()
            time_diff = (current_time - self.last_fps_time).to_sec()
            
            if time_diff > 1.0:  # 1초마다 FPS 업데이트
                fps = self.frame_count / time_diff
                self.fps_label.setText(f"FPS: {fps:.2f}")
                self.frame_count = 0
                self.last_fps_time = current_time
            
        except CvBridgeError as e:
            rospy.logerr(f"컬러 이미지 변환 오류: {e}")
    
    def depth_callback(self, msg):
        """정렬된 깊이 이미지 콜백"""
        try:
            # 깊이 이미지 변환 (16UC1 형식)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            
            # 마우스 위치에 따른 깊이 값 업데이트
            self.update_depth_value()
            
        except CvBridgeError as e:
            rospy.logerr(f"깊이 이미지 변환 오류: {e}")
    
    def info_callback(self, msg):
        """카메라 정보 콜백"""
        self.camera_info = msg
        
        # 카메라 정보 업데이트
        self.resolution_label.setText(f"해상도: {msg.width} x {msg.height}")
        self.frame_id_label.setText(f"프레임 ID: {msg.header.frame_id}")
        
        # 왜곡 계수
        distortion_str = ", ".join([f"{d:.4f}" for d in msg.D[:5]])
        self.distortion_label.setText(f"왜곡 계수: [{distortion_str}]")
        
        # K 행렬 (내부 파라미터)
        k_matrix = np.array(msg.K).reshape(3, 3)
        k_str = f"fx: {k_matrix[0, 0]:.2f}, fy: {k_matrix[1, 1]:.2f}\n"
        k_str += f"cx: {k_matrix[0, 2]:.2f}, cy: {k_matrix[1, 2]:.2f}"
        self.k_label.setText(f"K 행렬:\n{k_str}")
        
        # P 행렬 (투영 행렬)
        p_matrix = np.array(msg.P).reshape(3, 4)
        p_str = f"fx: {p_matrix[0, 0]:.2f}, fy: {p_matrix[1, 1]:.2f}\n"
        p_str += f"cx: {p_matrix[0, 2]:.2f}, cy: {p_matrix[1, 2]:.2f}\n"
        p_str += f"Tx: {p_matrix[0, 3]:.2f}, Ty: {p_matrix[1, 3]:.2f}"
        self.p_label.setText(f"P 행렬:\n{p_str}")
        
        # 정렬 상태 업데이트
        if self.depth_image is not None and self.color_image is not None:
            color_res = f"{self.color_image.shape[1]}x{self.color_image.shape[0]}"
            depth_res = f"{self.depth_image.shape[1]}x{self.depth_image.shape[0]}"
            
            if (self.depth_image.shape[1] == self.color_image.shape[1] and 
                self.depth_image.shape[0] == self.color_image.shape[0]):
                alignment = "정렬됨 (same resolution)"
            else:
                alignment = f"미정렬 (color: {color_res}, depth: {depth_res})"
            
            self.alignment_label.setText(f"정렬 상태: {alignment}")
    
    def on_mouse_moved(self, pos):
        """마우스 이동 이벤트 처리"""
        # 마우스 위치 저장
        self.last_mouse_pos = pos
        
        # 깊이 값 업데이트
        QTimer.singleShot(0, self.update_depth_value)  # 즉시 처리하나 이벤트 루프에서 실행
    
    def update_depth_value(self):
        """마우스 위치의 깊이 값 업데이트 - 정렬된 깊이 이미지 사용"""
        if self.depth_image is None or self.color_image is None:
            return
        
        # 이미지 내 마우스 좌표 계산
        img_coords = self.color_view.getImageCoordinates(self.last_mouse_pos)
        
        if img_coords is None:
            # 이미지 영역 밖
            self.pixel_depth = None
            self.depth_value_label.setText("-")
            self.mouse_pos_label.setText("이미지 영역 밖")
            return
        
        img_x, img_y = img_coords
        
        # 좌표값 범위 체크
        if (img_x < 0 or img_x >= self.depth_image.shape[1] or
            img_y < 0 or img_y >= self.depth_image.shape[0]):
            self.pixel_depth = None
            self.depth_value_label.setText("범위 초과")
            self.mouse_pos_label.setText(f"({img_x}, {img_y}) - 범위 초과")
            return
        
        # 깊이 값 추출 (밀리미터 단위)
        depth_val = self.depth_image[img_y, img_x]
        
        # 깊이 값 단위 변환 (mm -> m)
        depth_m = depth_val / 1000.0 if depth_val > 0 else 0.0
        
        # 깊이 값이 유효한지 확인 (예: 0m 초과, 10m 미만)
        if depth_m > 0 and depth_m < 10:
            self.pixel_depth = depth_m
            self.depth_value_label.setText(f"{depth_m:.3f} m")
            self.mouse_pos_label.setText(f"({img_x}, {img_y})")
            
            # 컬러 값도 함께 표시
            if hasattr(self, 'color_value_label'):
                b, g, r = self.color_image[img_y, img_x]
                self.color_value_label.setText(f"RGB: ({r},{g},{b})")
        else:
            self.pixel_depth = None
            self.depth_value_label.setText("유효하지 않은 깊이")
            self.mouse_pos_label.setText(f"({img_x}, {img_y})")
    def update_displays(self):
        """디스플레이 업데이트"""
        # 컬러 이미지 업데이트
        if self.color_image is not None:
            # BGR에서 RGB로 변환
            rgb_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # QImage 및 QPixmap으로 변환
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            
            # 표시 영역 크기에 맞게 조정
            scaled_pixmap = pixmap.scaled(self.color_view.width(), self.color_view.height(), 
                                  Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            self.color_view.setPixmap(scaled_pixmap)
            
            # 이미지 크기 표시 정보 업데이트
            if hasattr(self, 'resolution_label') and self.depth_image is not None:
                self.resolution_label.setText(f"컬러: {w}x{h}, 깊이: "
                                            f"{self.depth_image.shape[1]}x{self.depth_image.shape[0]}")
        
        # 깊이 이미지 업데이트
        if self.depth_image is not None:
            try:
                # 깊이 이미지 시각화
                # 유효 깊이 범위 설정 (0.1m ~ 5m)
                depth_norm = self.depth_image.copy()
                
                if depth_norm.dtype != np.float32:
                    # 밀리미터 단위를 미터로 변환
                    depth_norm = depth_norm.astype(np.float32) / 1000.0
                
                # 범위 밖 값을 0으로 설정 (마스킹)
                mask = (depth_norm > 0.1) & (depth_norm < 5.0)
                depth_norm_masked = np.zeros_like(depth_norm)
                depth_norm_masked[mask] = depth_norm[mask]
                
                # 0.1m ~ 5m 범위를 0~255로 정규화
                min_depth = 0.1
                max_depth = 5.0
                normalized = np.zeros_like(depth_norm_masked)
                normalized[mask] = ((depth_norm_masked[mask] - min_depth) / (max_depth - min_depth) * 255)
                depth_visualized = normalized.astype(np.uint8)
                
                # 컬러맵 적용 (JET: 파란색-녹색-빨간색 순으로 깊이 표시)
                depth_colormap = cv2.applyColorMap(depth_visualized, cv2.COLORMAP_JET)
                
                # 이미지 변환 및 표시
                h, w, ch = depth_colormap.shape
                bytes_per_line = ch * w
                qt_depth = QImage(depth_colormap.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_depth)
                
                # 표시 영역 크기에 맞게 조정
                scaled_pixmap = pixmap.scaled(self.depth_view.width(), self.depth_view.height(), 
                                         Qt.KeepAspectRatio, Qt.SmoothTransformation)
                
                self.depth_view.setPixmap(scaled_pixmap)
            except Exception as e:
                rospy.logerr(f"깊이 이미지 시각화 오류: {e}")
                
    
    def closeEvent(self, event):
        """닫기 이벤트 처리"""
        # 구독 해제
        if hasattr(self, 'color_sub'):
            self.color_sub.unregister()
        if hasattr(self, 'depth_sub'):
            self.depth_sub.unregister()
        if hasattr(self, 'info_sub'):
            self.info_sub.unregister()
        
        # 타이머 정지
        if self.update_timer.isActive():
            self.update_timer.stop()