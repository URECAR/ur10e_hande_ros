#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                           QGroupBox, QGridLayout, QPushButton, QFrame, QLineEdit,
                           QCheckBox, QDoubleSpinBox, QColorDialog, QSpinBox, QMessageBox)
from PyQt5.QtGui import QImage, QPixmap, QColor, QPainter, QPen, QFont
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint, QSize, QRect
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN


class ImageDisplayWidget(QLabel):
    """이미지 표시 및 마우스 이벤트 처리 위젯"""
    mouse_moved = pyqtSignal(QPoint)
    mouse_clicked = pyqtSignal(QPoint)  # 클릭 이벤트 추가
    
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
        
        # 객체 오버레이를 위한 변수
        self.object_positions = []  # 이미지 상의 객체 위치 목록
        self.highlighted_object = -1  # 현재 강조된 객체 인덱스
    
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
    
    def setObjects(self, object_positions):
        """감지된 객체 위치 설정"""
        self.object_positions = object_positions
        self.update()  # 위젯 다시 그리기
    
    def paintEvent(self, event):
        """위젯 페인트 이벤트 - 픽스맵 위에 객체 오버레이 그리기"""
        super().paintEvent(event)
        
        if not self.object_positions:
            return
            
        # QPainter 생성
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 각 객체 위치에 원 그리기
        for i, pos in enumerate(self.object_positions):
            # 강조 여부에 따라 색상 설정
            if i == self.highlighted_object:
                painter.setPen(QPen(QColor(255, 50, 50), 2))  # 빨간색, 두께 2
                painter.setBrush(QColor(255, 100, 100, 100))  # 반투명 빨간색
                size = 24  # 강조된 객체는 더 크게
            else:
                painter.setPen(QPen(QColor(50, 255, 50), 2))  # 녹색, 두께 2
                painter.setBrush(QColor(100, 255, 100, 100))  # 반투명 녹색
                size = 18
                
            # 원 그리기
            painter.drawEllipse(pos.x() - size/2, pos.y() - size/2, size, size)
            
            # 번호 표시
            painter.setPen(QColor(0, 0, 0))
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(QRect(pos.x() - size/2, pos.y() - size/2, size, size), 
                            Qt.AlignCenter, str(i+1))
        
        painter.end()
    
    def mouseMoveEvent(self, event):
        """마우스 움직임 이벤트 처리"""
        self.mouse_moved.emit(event.pos())
        
        # 마우스가 객체 위에 있는지 확인
        old_highlight = self.highlighted_object
        self.highlighted_object = -1
        
        for i, pos in enumerate(self.object_positions):
            # 마우스와 객체 중심 사이의 거리 계산
            distance = ((event.x() - pos.x())**2 + (event.y() - pos.y())**2)**0.5
            if distance < 15:  # 15픽셀 이내면 강조
                self.highlighted_object = i
                self.setCursor(Qt.PointingHandCursor)  # 손가락 커서로 변경
                break
        
        # 강조된 객체가 변경되었으면 다시 그리기
        if old_highlight != self.highlighted_object:
            self.update()
            
        # 객체 위에 없으면 기본 커서로 돌아가기
        if self.highlighted_object == -1:
            self.setCursor(Qt.CrossCursor)
            
        super().mouseMoveEvent(event)
    
    def mouseReleaseEvent(self, event):
        """마우스 클릭 이벤트 처리"""
        # 왼쪽 버튼 클릭 & 객체 위에 있을 때만 처리
        if event.button() == Qt.LeftButton and self.highlighted_object != -1:
            self.mouse_clicked.emit(event.pos())
        super().mouseReleaseEvent(event)
    
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
    """카메라 탭 위젯 - 물체 감지 기능 추가"""
    
    # 로봇 이동 명령 신호 추가
    move_to_object = pyqtSignal(float, float, float, float, float, float)
    
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
        
        # 영역 파라미터
        self.region_enabled = False
        self.x_min = 0.41
        self.x_max = 0.67
        self.y_min = -0.10
        self.y_max = 0.32
        self.z_min = 0.14
        self.z_max = 0.30
        
        # 로봇 제어 관련 변수
        self.robot_controller = None  # 나중에 설정됨
        self.current_robot_z = 0.2    # 기본값
        
        # 물체 감지 파라미터
        self.height_threshold = 0.015  # 평면 위로 튀어나온 물체로 감지할 높이 임계값(m)
        self.cluster_distance = 0.035  # 클러스터링할 때 최대 거리(m)
        self.min_points = 25  # 클러스터로 인정할 최소 포인트 수
        
        # 영역 박스 시각화 파라미터
        self.visualize_region = True
        self.marker_color = [0.2, 0.6, 1.0, 0.3]  # RGBA (반투명 파란색)
        
        # 물체 마커를 위한 퍼블리셔
        self.object_markers_pub = rospy.Publisher('/detected_objects', MarkerArray, queue_size=1)
        
        # 영역 박스 시각화를 위한 마커 퍼블리셔
        self.region_marker_pub = rospy.Publisher('/region_box_marker', Marker, queue_size=1)
        
        # 포인트클라우드 저장 변수
        self.point_cloud = None
        self.detected_objects = []
        
        self.init_ui()
        self.setup_subscribers()
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(100)  # 10Hz로 디스플레이 업데이트
        
        self.detection_timer = QTimer()
        self.detection_timer.timeout.connect(self.maybe_detect_objects)
        self.detection_timer.start(500)  # 500ms마다 물체 감지

        # 영역 박스 시각화 타이머 (느리게 업데이트)
        self.marker_timer = QTimer(self)
        self.marker_timer.timeout.connect(self.publish_region_box_marker)
        self.marker_timer.start(500)  # 2Hz로 마커 업데이트
        
        rospy.loginfo("카메라 탭 초기화 완료 - 물체 감지 기능 추가")
    
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
        self.image_view.mouse_clicked.connect(self.on_image_clicked)
        image_layout.addWidget(self.image_view)
        
        # 이미지 라벨 (현재 표시 중인 이미지 타입)
        self.image_type_label = QLabel("컬러 이미지")
        self.image_type_label.setStyleSheet("font-weight: bold; color: #3366cc;")
        self.image_type_label.setAlignment(Qt.AlignCenter)
        image_layout.addWidget(self.image_type_label)
        
        # 표시 모드 전환 버튼
        button_layout = QHBoxLayout()
        
        self.display_button = QPushButton("깊이 이미지 보기")
        self.display_button.clicked.connect(self.toggle_display_mode)
        button_layout.addWidget(self.display_button)
        
        # 로봇 제어 팁 추가
        self.control_tip = QLabel("물체를 클릭하면 로봇이 해당 위치로 이동합니다")
        self.control_tip.setStyleSheet("color: #008800; font-style: italic;")
        button_layout.addWidget(self.control_tip)
        
        image_layout.addLayout(button_layout)

        image_group.setLayout(image_layout)
        main_layout.addWidget(image_group)
        
        # 좌표 및 제어 패널 (중간에 한 줄로 배치)
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
        
        # 감지된 물체 수 표시
        self.objects_count_label = QLabel("감지된 물체: 0개")
        self.objects_count_label.setStyleSheet("font-weight: bold; color: green;")
        control_layout.addWidget(self.objects_count_label)
        
        # 공간 확보를 위한 스페이서
        control_layout.addStretch(1)
        
        main_layout.addWidget(control_frame)
        
        # 물체 감지 영역 설정 그룹
        region_group = QGroupBox("물체 감지 영역 설정")
        region_layout = QGridLayout()
        
        # 영역 활성화 체크박스 및 시각화 체크박스
        checkbox_layout = QHBoxLayout()
        self.region_checkbox = QCheckBox("영역 활성화")
        self.region_checkbox.toggled.connect(self.toggle_region)
        checkbox_layout.addWidget(self.region_checkbox)
        
        self.visualize_checkbox = QCheckBox("영역 시각화")
        self.visualize_checkbox.setChecked(self.visualize_region)
        self.visualize_checkbox.toggled.connect(self.toggle_visualization)
        checkbox_layout.addWidget(self.visualize_checkbox)
        
        # 색상 버튼 추가
        self.color_button = QPushButton("색상 변경")
        self.color_button.clicked.connect(self.change_marker_color)
        self.color_button.setStyleSheet(f"background-color: rgba({int(self.marker_color[0]*255)}, {int(self.marker_color[1]*255)}, {int(self.marker_color[2]*255)}, {int(self.marker_color[3]*255)})")
        checkbox_layout.addWidget(self.color_button)
        
        region_layout.addLayout(checkbox_layout, 0, 0, 1, 4)
        
        # 영역 정보 표시
        self.region_info_label = QLabel("영역: 비활성화")
        region_layout.addWidget(self.region_info_label, 1, 0, 1, 4)
        
        # X 범위 (첫 번째 줄)
        region_layout.addWidget(QLabel("X 범위 (m):"), 2, 0)
        self.x_min_input = QDoubleSpinBox()
        self.x_min_input.setRange(-10.0, 10.0)
        self.x_min_input.setValue(self.x_min)
        self.x_min_input.setSingleStep(0.01)
        self.x_min_input.valueChanged.connect(self.update_region_params)
        region_layout.addWidget(self.x_min_input, 2, 1)
        
        region_layout.addWidget(QLabel("~"), 2, 2)
        
        self.x_max_input = QDoubleSpinBox()
        self.x_max_input.setRange(-10.0, 10.0)
        self.x_max_input.setValue(self.x_max)
        self.x_max_input.setSingleStep(0.01)
        self.x_max_input.valueChanged.connect(self.update_region_params)
        region_layout.addWidget(self.x_max_input, 2, 3)
        
        # Y 범위 (두 번째 줄)
        region_layout.addWidget(QLabel("Y 범위 (m):"), 3, 0)
        self.y_min_input = QDoubleSpinBox()
        self.y_min_input.setRange(-10.0, 10.0)
        self.y_min_input.setValue(self.y_min)
        self.y_min_input.setSingleStep(0.01)
        self.y_min_input.valueChanged.connect(self.update_region_params)
        region_layout.addWidget(self.y_min_input, 3, 1)
        
        region_layout.addWidget(QLabel("~"), 3, 2)
        
        self.y_max_input = QDoubleSpinBox()
        self.y_max_input.setRange(-10.0, 10.0)
        self.y_max_input.setValue(self.y_max)
        self.y_max_input.setSingleStep(0.01)
        self.y_max_input.valueChanged.connect(self.update_region_params)
        region_layout.addWidget(self.y_max_input, 3, 3)
        
        # Z 범위 (세 번째 줄)
        region_layout.addWidget(QLabel("Z 범위 (m):"), 4, 0)
        self.z_min_input = QDoubleSpinBox()
        self.z_min_input.setRange(-10.0, 10.0)
        self.z_min_input.setValue(self.z_min)
        self.z_min_input.setSingleStep(0.01)
        self.z_min_input.valueChanged.connect(self.update_region_params)
        region_layout.addWidget(self.z_min_input, 4, 1)
        
        region_layout.addWidget(QLabel("~"), 4, 2)
        
        self.z_max_input = QDoubleSpinBox()
        self.z_max_input.setRange(-10.0, 10.0)
        self.z_max_input.setValue(self.z_max)
        self.z_max_input.setSingleStep(0.01)
        self.z_max_input.valueChanged.connect(self.update_region_params)
        region_layout.addWidget(self.z_max_input, 4, 3)
        
        # 물체 감지 파라미터 설정
        detection_params_layout = QGridLayout()
        
        # 높이 임계값
        detection_params_layout.addWidget(QLabel("높이 임계값 (cm):"), 0, 0)
        self.height_threshold_input = QDoubleSpinBox()
        self.height_threshold_input.setRange(0.1, 50.0)
        self.height_threshold_input.setValue(self.height_threshold * 100)  # m → cm 변환
        self.height_threshold_input.setSingleStep(0.5)
        self.height_threshold_input.valueChanged.connect(self.update_detection_params)
        detection_params_layout.addWidget(self.height_threshold_input, 0, 1)
        
        # 클러스터링 거리
        detection_params_layout.addWidget(QLabel("클러스터 거리 (cm):"), 1, 0)
        self.cluster_distance_input = QDoubleSpinBox()
        self.cluster_distance_input.setRange(0.5, 50.0)
        self.cluster_distance_input.setValue(self.cluster_distance * 100)  # m → cm 변환
        self.cluster_distance_input.setSingleStep(0.5)
        self.cluster_distance_input.valueChanged.connect(self.update_detection_params)
        detection_params_layout.addWidget(self.cluster_distance_input, 1, 1)
        
        # 최소 포인트 수
        detection_params_layout.addWidget(QLabel("최소 포인트 수:"), 2, 0)
        self.min_points_input = QSpinBox()
        self.min_points_input.setRange(3, 1000)
        self.min_points_input.setValue(self.min_points)
        self.min_points_input.valueChanged.connect(self.update_detection_params)
        detection_params_layout.addWidget(self.min_points_input, 2, 1)
        
        # 물체 감지 파라미터 그룹 추가
        detection_params_group = QGroupBox("물체 감지 파라미터")
        detection_params_group.setLayout(detection_params_layout)
        
        # 두 그룹을 하나의 레이아웃으로 결합
        region_layout.addWidget(detection_params_group, 5, 0, 1, 4)
        
        region_group.setLayout(region_layout)
        main_layout.addWidget(region_group)
        
        # 전체 크기 설정 - 이미지와 영역의 비율 조정
        image_ratio = 7
        region_ratio = 3
        main_layout.setStretch(0, image_ratio)  # 이미지 그룹 (인덱스 0)
        main_layout.setStretch(2, region_ratio)  # 영역 그룹 (인덱스 2)
    
    def setup_subscribers(self):
        """ROS 토픽 구독 설정"""
        rospy.loginfo("구독할 카메라 토픽:")
        rospy.loginfo("  컬러: /camera/color/image_raw")
        rospy.loginfo("  깊이: /camera/aligned_depth_to_color/image_raw")
        rospy.loginfo("  정보: /camera/color/camera_info")
        rospy.loginfo("  포인트클라우드: /camera/depth/color/points")
        
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_callback)
        
        # 포인트클라우드 구독 추가
        self.cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        
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
    
    def pointcloud_callback(self, msg):
        """포인트클라우드 콜백"""
        self.point_cloud = msg
    
    def toggle_region(self, enabled):
        """영역 활성화/비활성화 토글"""
        self.region_enabled = enabled
        
        if enabled:
            region_text = self.get_region_range_text()
            self.region_info_label.setText(f"영역: 활성화 ({region_text})")
            # 영역 박스 마커 즉시 발행
            if self.visualize_region:
                self.publish_region_box_marker()
        else:
            self.region_info_label.setText("영역: 비활성화")
            # 영역 비활성화 시 마커 제거
            if self.visualize_region:
                self.publish_region_box_marker(delete=True)
            # 물체 마커 제거
            self.publish_object_markers([], delete=True)
            self.objects_count_label.setText("감지된 물체: 0개")
    
    def toggle_visualization(self, enabled):
        """영역 시각화 활성화/비활성화 토글"""
        self.visualize_region = enabled
        
        if enabled and self.region_enabled:
            # 시각화 활성화 & 영역 활성화된 경우 마커 발행
            self.publish_region_box_marker()
        elif not enabled:
            # 시각화 비활성화 시 마커 제거
            self.publish_region_box_marker(delete=True)
    
    def change_marker_color(self):
        """마커 색상 변경"""
        current_color = QColor(
            int(self.marker_color[0] * 255),
            int(self.marker_color[1] * 255),
            int(self.marker_color[2] * 255),
            int(self.marker_color[3] * 255)
        )
        
        color = QColorDialog.getColor(current_color, self, "영역 박스 색상 선택", QColorDialog.ShowAlphaChannel)
        
        if color.isValid():
            self.marker_color = [
                color.red() / 255.0,
                color.green() / 255.0,
                color.blue() / 255.0,
                color.alpha() / 255.0
            ]
            
            # 색상 버튼 배경색 업데이트
            self.color_button.setStyleSheet(f"background-color: rgba({color.red()}, {color.green()}, {color.blue()}, {color.alpha()})")
            
            # 마커 색상 업데이트
            if self.visualize_region and self.region_enabled:
                self.publish_region_box_marker()
    
    def get_region_range_text(self):
        """현재 영역 범위 텍스트 생성"""
        return f"X[{self.x_min:.2f}~{self.x_max:.2f}], Y[{self.y_min:.2f}~{self.y_max:.2f}], Z[{self.z_min:.2f}~{self.z_max:.2f}]"
    
    def update_region_params(self):
        """영역 파라미터 업데이트"""
        self.x_min = self.x_min_input.value()
        self.x_max = self.x_max_input.value()
        self.y_min = self.y_min_input.value()
        self.y_max = self.y_max_input.value()
        self.z_min = self.z_min_input.value()
        self.z_max = self.z_max_input.value()
        
        if self.region_enabled:
            region_text = self.get_region_range_text()
            self.region_info_label.setText(f"영역: 활성화 ({region_text})")
            
            # 영역 박스 시각화 업데이트
            if self.visualize_region:
                self.publish_region_box_marker()
    
    def update_detection_params(self):
        """물체 감지 파라미터 업데이트"""
        # cm → m 변환
        self.height_threshold = self.height_threshold_input.value() / 100.0
        self.cluster_distance = self.cluster_distance_input.value() / 100.0
        self.min_points = self.min_points_input.value()
        
        rospy.loginfo(f"물체 감지 파라미터 업데이트: 높이 임계값={self.height_threshold:.3f}m, "
                     f"클러스터 거리={self.cluster_distance:.3f}m, "
                     f"최소 포인트={self.min_points}")
    
    def maybe_detect_objects(self):
        """영역이 활성화된 경우에만 물체 감지 실행"""
        if self.region_enabled and self.point_cloud is not None:
            self.detect_objects()
    
    def detect_objects(self):
        """지정된 영역 내에서 물체 감지"""
        if self.point_cloud is None:
            return

        try:
            import tf.transformations

            # TF 변환 획득
            tf_stamped = self.tf_buffer.lookup_transform(
                "base_link", self.point_cloud.header.frame_id,
                rospy.Time(0), rospy.Duration(1.0)
            )

            trans = tf_stamped.transform.translation
            rot = tf_stamped.transform.rotation
            quaternion = [rot.x, rot.y, rot.z, rot.w]
            T = tf.transformations.quaternion_matrix(quaternion)
            T[0:3, 3] = [trans.x, trans.y, trans.z]

            # PointCloud2 → NumPy array 변환
            cloud = np.array(list(pc2.read_points(self.point_cloud,
                                                  field_names=("x", "y", "z", "rgb"),
                                                  skip_nans=True)),
                             dtype=[('x', np.float32), ('y', np.float32),
                                    ('z', np.float32), ('rgb', np.float32)])
            if len(cloud) == 0:
                self.region_info_label.setText("영역: 활성화 (포인트 없음)")
                return

            # Nx4 좌표 → 월드 좌표 변환
            xyz = np.vstack((cloud['x'], cloud['y'], cloud['z'], np.ones(len(cloud)))).T
            world_xyz = (T @ xyz.T).T[:, :3]

            # 지정 영역 내의 포인트 선택
            x_cond = (self.x_min <= world_xyz[:, 0]) & (world_xyz[:, 0] <= self.x_max)
            y_cond = (self.y_min <= world_xyz[:, 1]) & (world_xyz[:, 1] <= self.y_max)
            z_cond = (self.z_min <= world_xyz[:, 2]) & (world_xyz[:, 2] <= self.z_max)
            region_mask = x_cond & y_cond & z_cond

            # 영역 내 포인트가 충분한지 확인
            if np.sum(region_mask) < 10:
                self.region_info_label.setText(f"영역: 활성화 (포인트 부족: {np.sum(region_mask)}개)")
                self.publish_object_markers([], delete=True)
                self.objects_count_label.setText("감지된 물체: 0개")
                return

            # 영역 내 포인트 추출
            region_points = world_xyz[region_mask]
            region_colors = cloud['rgb'][region_mask]

            # 평면(바닥) Z 높이 추정 - 히스토그램 방식
            z_values = region_points[:, 2]
            hist, bin_edges = np.histogram(z_values, bins=50)
            dominant_bin_idx = np.argmax(hist)
            ground_z = (bin_edges[dominant_bin_idx] + bin_edges[dominant_bin_idx + 1]) / 2

            # 평면보다 높은 포인트 선택 (물체 후보)
            object_mask = z_values > (ground_z + self.height_threshold)
            if np.sum(object_mask) < self.min_points:
                self.region_info_label.setText(f"영역: 활성화 (물체 포인트 부족: {np.sum(object_mask)}개)")
                self.publish_object_markers([], delete=True)
                self.objects_count_label.setText("감지된 물체: 0개")
                return

            # 물체 후보 포인트 추출
            object_points = region_points[object_mask]

            # 클러스터링으로 개별 물체 감지 (DBSCAN 알고리즘)
            clustering = DBSCAN(eps=self.cluster_distance, min_samples=self.min_points).fit(object_points)
            labels = clustering.labels_

            # 클러스터 수 계산 (-1은 노이즈 포인트)
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

            # 각 클러스터의 중심점 계산
            object_centers = []
            for i in range(n_clusters):
                cluster_points = object_points[labels == i]
                if len(cluster_points) >= self.min_points:
                    # 물체의 중심점 계산
                    center = np.mean(cluster_points, axis=0)
                    # 물체의 높이 계산
                    height = np.max(cluster_points[:, 2]) - ground_z
                    # 물체 정보 저장
                    object_centers.append({
                        'center': center,
                        'height': height,
                        'num_points': len(cluster_points)
                    })

            # 감지된 물체 정보 업데이트
            self.detected_objects = object_centers
            self.objects_count_label.setText(f"감지된 물체: {len(object_centers)}개")
            
            # 영역 내 물체 정보 업데이트
            self.region_info_label.setText(f"영역: 활성화 ({len(object_centers)}개 물체 감지, 기준 Z: {ground_z:.3f}m)")

            # 물체 중심 마커 발행
            self.publish_object_markers(object_centers)

        except Exception as e:
            rospy.logerr(f"[detect_objects] 오류: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def publish_object_markers(self, objects, delete=False):
        """감지된 물체 중심에 마커 발행"""
        marker_array = MarkerArray()
        
        if delete:
            # 기존 마커 삭제
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "detected_objects"
            marker.id = 0
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            self.object_markers_pub.publish(marker_array)
            
            # 이미지 상의 객체 표시도 제거
            self.image_view.setObjects([])
            return
        
        # 각 물체마다 마커 생성
        for i, obj in enumerate(objects):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "detected_objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 물체 중심점 설정
            marker.pose.position.x = obj['center'][0]
            marker.pose.position.y = obj['center'][1]
            marker.pose.position.z = obj['center'][2]
            
            # 마커 방향 (기본값)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 마커 크기 (높이에 비례하게 설정)
            size = max(0.02, min(0.05, obj['height'] * 0.8))  # 물체 높이의 80%로 하되 최소 2cm, 최대 5cm
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            
            # 마커 색상 (초록색)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            # 마커 유지 시간
            marker.lifetime = rospy.Duration(1.0)  # 1초
            
            marker_array.markers.append(marker)
            
            # 월드 좌표를 이미지 좌표로 변환하여 저장
            pixel_pos = self.world_to_pixel(obj['center'][0], obj['center'][1], obj['center'][2])
            if pixel_pos:
                obj['pixel_pos'] = pixel_pos  # 이미지 상의 위치 저장
        
        # 마커 배열 발행
        if marker_array.markers:
            self.object_markers_pub.publish(marker_array)
            
            # 이미지 상에 물체 위치 표시 업데이트
            self.update_image_objects(objects)
    
    def publish_region_box_marker(self, delete=False):
        """영역을 표시하는 마커 발행 (월드 좌표계 기준)"""
        if not self.visualize_region and not delete:
            return
        
        marker = Marker()
        # 월드 좌표계로 명시적 설정
        marker.header.frame_id = "base_link"  # 월드 좌표계
        marker.header.stamp = rospy.Time.now()
        marker.ns = "region_box"
        marker.id = 0
        
        if delete or not self.region_enabled:
            marker.action = Marker.DELETE
        else:
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            
            # 영역의 중심 계산 (월드 좌표계 기준)
            center_x = (self.x_min + self.x_max) / 2
            center_y = (self.y_min + self.y_max) / 2
            center_z = (self.z_min + self.z_max) / 2
            
            # 마커 위치 설정 (월드 좌표계)
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = center_z
            
            # 마커 방향 (회전 없음 - 항등 쿼터니언)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 마커 크기 (영역의 크기)
            marker.scale.x = max(0.001, abs(self.x_max - self.x_min))
            marker.scale.y = max(0.001, abs(self.y_max - self.y_min))
            marker.scale.z = max(0.001, abs(self.z_max - self.z_min))
            
            # 마커 색상 (RGBA, 반투명)
            marker.color.r = self.marker_color[0]
            marker.color.g = self.marker_color[1]
            marker.color.b = self.marker_color[2]
            marker.color.a = self.marker_color[3]  # 알파값 (투명도)
            
            # 마커가 계속 표시되도록 설정
            marker.lifetime = rospy.Duration(0)
        
        # 마커 발행
        self.region_marker_pub.publish(marker)
    
    def on_mouse_moved(self, pos):
        """마우스 이동 이벤트 처리 - 스로틀링 적용"""
        current_time = rospy.Time.now()
        if not hasattr(self, 'last_mouse_update_time') or \
           (current_time - self.last_mouse_update_time).to_sec() > 0.1:  # 100ms마다만 업데이트
            self.last_mouse_pos = pos
            self.last_mouse_update_time = current_time
            QTimer.singleShot(0, self.update_depth_value)
    
    def toggle_display_mode(self):
        """이미지 표시 모드 전환"""
        self.display_mode = 1 - self.display_mode  # 0과 1 사이 전환
        
        if self.display_mode == 0:
            self.image_type_label.setText("컬러 이미지")
            self.display_button.setText("깊이 이미지 보기")
        else:
            self.image_type_label.setText("깊이 이미지")
            self.display_button.setText("컬러 이미지 보기")
    
    def pixel_to_world(self, x, y, depth):
        """이미지 좌표를 월드 좌표로 변환"""
        if self.camera_info is None or depth <= 0:
            return None
        try:
            # intrinsic matrix
            k_matrix = np.array(self.camera_info.K).reshape(3, 3)
            fx = k_matrix[0, 0]
            fy = k_matrix[1, 1]
            cx = k_matrix[0, 2]
            cy = k_matrix[1, 2]

            # pixel → camera coords
            x_cam = (x - cx) * depth / fx
            y_cam = (y - cy) * depth / fy
            z_cam = depth

            # 카메라 기준 좌표 (회전 없이 순수한 좌표)
            camera_point = PointStamped()
            camera_point.header.frame_id = self.camera_info.header.frame_id
            camera_point.header.stamp = rospy.Time(0)
            camera_point.point.x = x_cam
            camera_point.point.y = y_cam
            camera_point.point.z = z_cam

            # base_link 기준으로 transform
            world_point = self.tf_buffer.transform(camera_point, "base_link", rospy.Duration(0.2))
            return world_point.point

        except Exception as e:
            rospy.logerr(f"[pixel_to_world] 변환 실패: {e}")
            return None
            
    def world_to_pixel(self, x, y, z):
        """월드 좌표를 이미지 좌표로 변환"""
        if self.camera_info is None or self.color_image is None:
            return None
            
        try:
            # 월드 좌표 포인트 생성
            world_point = PointStamped()
            world_point.header.frame_id = "base_link"
            world_point.header.stamp = rospy.Time(0)
            world_point.point.x = x
            world_point.point.y = y
            world_point.point.z = z
            
            # 카메라 프레임으로 변환
            camera_point = self.tf_buffer.transform(world_point, self.camera_info.header.frame_id, rospy.Duration(0.2))
            
            # 카메라 내부 파라미터
            k_matrix = np.array(self.camera_info.K).reshape(3, 3)
            fx = k_matrix[0, 0]
            fy = k_matrix[1, 1]
            cx = k_matrix[0, 2]
            cy = k_matrix[1, 2]
            
            # 카메라 좌표계에서 이미지 좌표계로 변환
            if camera_point.point.z <= 0:
                return None
                
            pixel_x = int(fx * camera_point.point.x / camera_point.point.z + cx)
            pixel_y = int(fy * camera_point.point.y / camera_point.point.z + cy)
            
            # 이미지 범위 확인
            h, w = self.color_image.shape[:2]
            if 0 <= pixel_x < w and 0 <= pixel_y < h:
                # UI 좌표로 변환 (이미지 좌표 → 위젯 좌표)
                widget_x = int(pixel_x * self.image_view.scaled_image_width / w) + self.image_view.image_offset_x
                widget_y = int(pixel_y * self.image_view.scaled_image_height / h) + self.image_view.image_offset_y
                return QPoint(widget_x, widget_y)
            
            return None
            
        except Exception as e:
            rospy.logerr(f"[world_to_pixel] 변환 실패: {e}")
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
            self.img_coord_label.setText(f"img:({img_x},{img_y})")
            self.depth_label.setText(f"depth:{int(depth_val)}")
            
            world_point = self.pixel_to_world(img_x, img_y, depth_m)
            self.world_point = world_point
            if world_point:
                x_mm = world_point.x * 1000
                y_mm = world_point.y * 1000
                z_mm = world_point.z * 1000
                self.world_coord_label.setText(f"wld:({x_mm:.0f},{y_mm:.0f},{z_mm:.0f})")
                
                # 현재 포인트가 영역 범위 내에 있는지 확인
                if (self.region_enabled and 
                    self.x_min <= world_point.x <= self.x_max and
                    self.y_min <= world_point.y <= self.y_max and
                    self.z_min <= world_point.z <= self.z_max):
                    # 영역 범위 내 포인트 강조
                    self.world_coord_label.setStyleSheet("font-weight: bold; color: green;")
                else:
                    self.world_coord_label.setStyleSheet("font-weight: bold;")
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
            
            # 감지된 물체가 있으면 이미지 위에 표시 업데이트
            if hasattr(self, 'detected_objects') and self.detected_objects:
                self.update_image_objects(self.detected_objects)
            
        elif self.display_mode == 1 and self.depth_image is not None:
            # 깊이 이미지 표시
            try:
                depth_norm = self.depth_image.astype(np.float32) / 1000.0
                mask = (depth_norm > 0.1) & (depth_norm < 5.0)
                depth_norm_masked = np.zeros_like(depth_norm)
                depth_norm_masked[mask] = depth_norm[mask]
                min_depth = 0.5
                max_depth = 1.7
                normalized = np.zeros_like(depth_norm_masked)
                normalized[mask] = ((depth_norm_masked[mask] - min_depth) / (max_depth - min_depth) * 255)
                depth_visualized = normalized.astype(np.uint8)
                depth_colormap = cv2.applyColorMap(depth_visualized, cv2.COLORMAP_JET)
                h, w, ch = depth_colormap.shape
                bytes_per_line = ch * w
                qt_depth = QImage(depth_colormap.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_depth)
                self.image_view.setPixmap(pixmap)
                
                # 감지된 물체가 있으면 이미지 위에 표시 업데이트
                if hasattr(self, 'detected_objects') and self.detected_objects:
                    self.update_image_objects(self.detected_objects)
            except Exception as e:
                rospy.logerr(f"깊이 이미지 시각화 오류: {e}")
                
    def update_image_objects(self, objects):
        """감지된 물체 위치를 이미지에 표시"""
        if self.color_image is None:
            return
            
        # 각 물체의 이미지 상 위치 계산
        object_positions = []
        for obj in objects:
            # 이미 계산된 pixel_pos가 있는지 확인
            if 'pixel_pos' in obj and obj['pixel_pos']:
                object_positions.append(obj['pixel_pos'])
            else:
                # 월드 좌표를 이미지 좌표로 변환
                pixel_pos = self.world_to_pixel(obj['center'][0], obj['center'][1], obj['center'][2])
                if pixel_pos:
                    obj['pixel_pos'] = pixel_pos
                    object_positions.append(pixel_pos)
        
        # 이미지에 객체 위치 설정
        self.image_view.setObjects(object_positions)
    
    def on_image_clicked(self, pos):
        """이미지 클릭 이벤트 처리 - 물체 선택"""
        # 강조된 물체가 있는지 확인
        if self.image_view.highlighted_object == -1 or not hasattr(self, 'detected_objects'):
            return
            
        try:
            # 선택된 물체 정보 가져오기
            obj_idx = self.image_view.highlighted_object
            if obj_idx >= len(self.detected_objects):
                return
                
            obj = self.detected_objects[obj_idx]
            
            # 로봇을 물체 위치로 이동시키기 위한 파라미터 계산
            x_mm = obj['center'][0] * 1000  # m -> mm
            y_mm = obj['center'][1] * 1000  # m -> mm
            
            # Z 위치는 현재 로봇의 Z 값 유지
            # 로봇 컨트롤러가 설정되어 있지 않으면 현재 위치를 알 수 없으므로 기본값 사용
            if self.robot_controller:
                # 로봇 컨트롤러에서 현재 TCP Z 좌표 가져오기 (이미 mm 단위)
                z_mm = self.current_robot_z
            else:
                # 기본값 사용 (200mm)
                z_mm = 200.0
                
            # 방향 값은 현재 유지 (로봇 컨트롤러에서 가져오거나 기본값 사용)
            rx, ry, rz = 180.0, 0.0, 90.0  # 기본값
            
            # 로봇 이동 신호 발생
            self.move_to_object.emit(x_mm, y_mm, z_mm, rx, ry, rz)
            
            # 사용자에게 피드백
            rospy.loginfo(f"물체 {obj_idx+1}번 클릭됨: 위치 ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f})")
            QMessageBox.information(self, "물체 선택됨", 
                                  f"물체 {obj_idx+1}번 위치로 로봇 이동 계획 중...\n"
                                  f"위치: X={x_mm:.1f}mm, Y={y_mm:.1f}mm, Z={z_mm:.1f}mm")
            
        except Exception as e:
            rospy.logerr(f"물체 클릭 처리 오류: {e}")
            QMessageBox.warning(self, "오류", f"물체 위치로 이동 중 오류 발생: {str(e)}")
    
    def set_robot_controller(self, controller, current_z=None):
        """로봇 컨트롤러 설정"""
        self.robot_controller = controller
        
        if current_z is not None:
            self.current_robot_z = current_z
    
    def closeEvent(self, event):
        """닫기 이벤트 처리"""
        if hasattr(self, 'color_sub'):
            self.color_sub.unregister()
        if hasattr(self, 'depth_sub'):
            self.depth_sub.unregister()
        if hasattr(self, 'info_sub'):
            self.info_sub.unregister()
        if hasattr(self, 'cloud_sub'):
            self.cloud_sub.unregister()
        if self.update_timer.isActive():
            self.update_timer.stop()
        if self.marker_timer.isActive():
            self.marker_timer.stop()
        
        # 마지막으로 마커 제거 (삭제)
        self.publish_region_box_marker(delete=True)
        self.publish_object_markers([], delete=True)