#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                           QGroupBox, QGridLayout, QPushButton, QFrame, QLineEdit,
                           QCheckBox, QDoubleSpinBox, QColorDialog)
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint, QSize, QRect
import sensor_msgs.point_cloud2 as pc2


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
        
        # 공간 필터 파라미터
        self.filter_enabled = False
        self.x_min = 0.41
        self.x_max = 0.87
        self.y_min = -0.01
        self.y_max = 0.37
        self.z_min = 0.18
        self.z_max = 0.30
        
        # 필터 시각화 파라미터
        self.visualize_filter = True
        self.marker_color = [0.2, 0.6, 1.0, 0.3]  # RGBA (반투명 파란색)
        
        # 필터링된 포인트클라우드를 위한 퍼블리셔
        self.filtered_cloud_pub = rospy.Publisher('/filtered_pointcloud', PointCloud2, queue_size=1)
        
        # 필터 영역 시각화를 위한 마커 퍼블리셔
        self.marker_pub = rospy.Publisher('/filter_box_marker', Marker, queue_size=1)
        
        # 포인트클라우드 저장 변수
        self.point_cloud = None
        
        self.init_ui()
        self.setup_subscribers()
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(100)  # 10Hz로 디스플레이 업데이트
        
        self.filter_timer = QTimer()
        self.filter_timer.timeout.connect(self.maybe_apply_filter)
        self.filter_timer.start(300)  # 300ms마다 검사

        # 필터 박스 시각화 타이머 (느리게 업데이트)
        self.marker_timer = QTimer(self)
        self.marker_timer.timeout.connect(self.publish_filter_box_marker)
        self.marker_timer.start(500)  # 2Hz로 마커 업데이트
        
        rospy.loginfo("카메라 탭 초기화 완료 - 필터링 시각화 기능 추가")
    
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
        
        # 표시 모드 전환 버튼
        self.display_button = QPushButton("깊이 이미지 보기")
        self.display_button.clicked.connect(self.toggle_display_mode)
        image_layout.addWidget(self.display_button)

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
        
        # 공간 확보를 위한 스페이서
        control_layout.addStretch(1)
        
        main_layout.addWidget(control_frame)
        
        # 공간 필터 설정 그룹
        filter_group = QGroupBox("공간 필터 설정")
        filter_layout = QGridLayout()
        
        # 필터 활성화 체크박스 및 시각화 체크박스
        checkbox_layout = QHBoxLayout()
        self.filter_checkbox = QCheckBox("필터 활성화")
        self.filter_checkbox.toggled.connect(self.toggle_filter)
        checkbox_layout.addWidget(self.filter_checkbox)
        
        self.visualize_checkbox = QCheckBox("필터 영역 시각화")
        self.visualize_checkbox.setChecked(self.visualize_filter)
        self.visualize_checkbox.toggled.connect(self.toggle_visualization)
        checkbox_layout.addWidget(self.visualize_checkbox)
        
        # 색상 버튼 추가
        self.color_button = QPushButton("색상 변경")
        self.color_button.clicked.connect(self.change_marker_color)
        self.color_button.setStyleSheet(f"background-color: rgba({int(self.marker_color[0]*255)}, {int(self.marker_color[1]*255)}, {int(self.marker_color[2]*255)}, {int(self.marker_color[3]*255)})")
        checkbox_layout.addWidget(self.color_button)
        
        filter_layout.addLayout(checkbox_layout, 0, 0, 1, 4)
        
        # 필터 정보 표시
        self.filter_info_label = QLabel("필터: 비활성화")
        filter_layout.addWidget(self.filter_info_label, 1, 0, 1, 4)
        
        # X 범위 (첫 번째 줄)
        filter_layout.addWidget(QLabel("X 범위 (m):"), 2, 0)
        self.x_min_input = QDoubleSpinBox()
        self.x_min_input.setRange(-10.0, 10.0)
        self.x_min_input.setValue(self.x_min)
        self.x_min_input.setSingleStep(0.01)
        self.x_min_input.valueChanged.connect(self.update_filter_params)
        filter_layout.addWidget(self.x_min_input, 2, 1)
        
        filter_layout.addWidget(QLabel("~"), 2, 2)
        
        self.x_max_input = QDoubleSpinBox()
        self.x_max_input.setRange(-10.0, 10.0)
        self.x_max_input.setValue(self.x_max)
        self.x_max_input.setSingleStep(0.01)
        self.x_max_input.valueChanged.connect(self.update_filter_params)
        filter_layout.addWidget(self.x_max_input, 2, 3)
        
        # Y 범위 (두 번째 줄)
        filter_layout.addWidget(QLabel("Y 범위 (m):"), 3, 0)
        self.y_min_input = QDoubleSpinBox()
        self.y_min_input.setRange(-10.0, 10.0)
        self.y_min_input.setValue(self.y_min)
        self.y_min_input.setSingleStep(0.01)
        self.y_min_input.valueChanged.connect(self.update_filter_params)
        filter_layout.addWidget(self.y_min_input, 3, 1)
        
        filter_layout.addWidget(QLabel("~"), 3, 2)
        
        self.y_max_input = QDoubleSpinBox()
        self.y_max_input.setRange(-10.0, 10.0)
        self.y_max_input.setValue(self.y_max)
        self.y_max_input.setSingleStep(0.01)
        self.y_max_input.valueChanged.connect(self.update_filter_params)
        filter_layout.addWidget(self.y_max_input, 3, 3)
        
        # Z 범위 (세 번째 줄)
        filter_layout.addWidget(QLabel("Z 범위 (m):"), 4, 0)
        self.z_min_input = QDoubleSpinBox()
        self.z_min_input.setRange(-10.0, 10.0)
        self.z_min_input.setValue(self.z_min)
        self.z_min_input.setSingleStep(0.01)
        self.z_min_input.valueChanged.connect(self.update_filter_params)
        filter_layout.addWidget(self.z_min_input, 4, 1)
        
        filter_layout.addWidget(QLabel("~"), 4, 2)
        
        self.z_max_input = QDoubleSpinBox()
        self.z_max_input.setRange(-10.0, 10.0)
        self.z_max_input.setValue(self.z_max)
        self.z_max_input.setSingleStep(0.01)
        self.z_max_input.valueChanged.connect(self.update_filter_params)
        filter_layout.addWidget(self.z_max_input, 4, 3)
        
        filter_group.setLayout(filter_layout)
        main_layout.addWidget(filter_group)
        
        # 전체 크기 설정 - 이미지와 필터 영역의 비율 조정
        image_ratio = 7
        filter_ratio = 3
        main_layout.setStretch(0, image_ratio)  # 이미지 그룹 (인덱스 0)
        main_layout.setStretch(2, filter_ratio)  # 필터 그룹 (인덱스 2)
    
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
        """포인트클라우드 콜백 - 필터링은 타이머 기반으로 분리"""
        self.point_cloud = msg
        self.filter_pending = True  # 새로운 데이터 수신 플래그

    
    def toggle_filter(self, enabled):
        """필터 활성화/비활성화 토글"""
        self.filter_enabled = enabled
        
        if enabled:
            self.filter_info_label.setText(f"필터: 활성화")
            # 필터 활성화 시 바로 적용
            self.apply_filter()
            # 필터링 박스 마커 즉시 발행
            if self.visualize_filter:
                self.publish_filter_box_marker()
        else:
            self.filter_info_label.setText("필터: 비활성화")
            # 필터 비활성화 시 마커 제거
            if self.visualize_filter:
                self.publish_filter_box_marker(delete=True)
    
    def toggle_visualization(self, enabled):
        """필터 시각화 활성화/비활성화 토글"""
        self.visualize_filter = enabled
        
        if enabled and self.filter_enabled:
            # 시각화 활성화 & 필터 활성화된 경우 마커 발행
            self.publish_filter_box_marker()
        elif not enabled:
            # 시각화 비활성화 시 마커 제거
            self.publish_filter_box_marker(delete=True)
    
    def change_marker_color(self):
        """마커 색상 변경"""
        current_color = QColor(
            int(self.marker_color[0] * 255),
            int(self.marker_color[1] * 255),
            int(self.marker_color[2] * 255),
            int(self.marker_color[3] * 255)
        )
        
        color = QColorDialog.getColor(current_color, self, "필터 박스 색상 선택", QColorDialog.ShowAlphaChannel)
        
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
            if self.visualize_filter and self.filter_enabled:
                self.publish_filter_box_marker()
    
    def get_filter_range_text(self):
        """현재 필터 범위 텍스트 생성"""
        return f"X[{self.x_min:.2f}~{self.x_max:.2f}], Y[{self.y_min:.2f}~{self.y_max:.2f}], Z[{self.z_min:.2f}~{self.z_max:.2f}]"
    
    def update_filter_params(self):
        """필터 파라미터 업데이트 - 스로틀링 적용"""
        self.x_min = self.x_min_input.value()
        self.x_max = self.x_max_input.value()
        self.y_min = self.y_min_input.value()
        self.y_max = self.y_max_input.value()
        self.z_min = self.z_min_input.value()
        self.z_max = self.z_max_input.value()
        
        if self.filter_enabled:
            self.filter_info_label.setText(f"필터: 활성화")
            
            # 파라미터 변경 후 일정 시간 후에만 필터 적용
            self.update_after_filter_change()

    
    def update_after_filter_change(self):
        """필터 파라미터 변경 후 업데이트 작업"""
        # 필터 적용
        # self.apply_filter()
        
        # 필터 시각화 업데이트
        if self.visualize_filter:
            self.publish_filter_box_marker()
    
    def maybe_apply_filter(self):
        if self.filter_enabled:
            self.apply_filter()



    def apply_filter(self):
        """PointCloud2를 NumPy 기반으로 필터링하고 재발행"""
        if not self.point_cloud or not self.filter_enabled:
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
                self.filter_info_label.setText("필터: 활성화 (포인트 없음)")
                return

            # Nx4 좌표 → 월드 좌표 변환
            xyz = np.vstack((cloud['x'], cloud['y'], cloud['z'], np.ones(len(cloud)))).T
            world_xyz = (T @ xyz.T).T[:, :3]

            # 필터링 조건 (월드 좌표 기준)
            x_cond = (self.x_min <= world_xyz[:, 0]) & (world_xyz[:, 0] <= self.x_max)
            y_cond = (self.y_min <= world_xyz[:, 1]) & (world_xyz[:, 1] <= self.y_max)
            z_cond = (self.z_min <= world_xyz[:, 2]) & (world_xyz[:, 2] <= self.z_max)
            mask = x_cond & y_cond & z_cond

            filtered_points = np.empty(np.sum(mask), dtype=cloud.dtype)
            filtered_points['x'] = world_xyz[mask, 0]
            filtered_points['y'] = world_xyz[mask, 1]
            filtered_points['z'] = world_xyz[mask, 2]
            filtered_points['rgb'] = cloud['rgb'][mask]

            self.filter_info_label.setText(f"필터: 활성화 ({len(filtered_points)} 포인트)")

            if len(filtered_points) > 0:
                self.publish_filtered_cloud(filtered_points)
            else:
                rospy.logwarn("필터링된 포인트 없음")

        except Exception as e:
            rospy.logerr(f"[apply_filter] 오류: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def publish_filtered_cloud(self, filtered_points):
        """NumPy 기반 포인트클라우드 발행"""
        from std_msgs.msg import Header

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.filtered_cloud_pub.publish(pc2.create_cloud(header, fields, filtered_points))

    def publish_filter_box_marker(self, delete=False):
        """필터 영역을 표시하는 마커 발행 (월드 좌표계 기준)"""
        if not self.visualize_filter and not delete:
            return
        
        marker = Marker()
        # 월드 좌표계로 명시적 설정
        marker.header.frame_id = "base_link"  # 월드 좌표계
        marker.header.stamp = rospy.Time.now()
        marker.ns = "filter_box"
        marker.id = 0
        
        if delete or not self.filter_enabled:
            marker.action = Marker.DELETE
        else:
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            
            # 필터 영역의 중심 계산 (월드 좌표계 기준)
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
            
            # 마커 크기 (필터 영역의 크기)
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
            
            # 디버깅을 위한 로그 출력
            # rospy.loginfo_throttle(5.0, f"필터 박스 마커 발행: 중심({center_x:.3f}, {center_y:.3f}, {center_z:.3f}), " + 
            #               f"크기({marker.scale.x:.3f} x {marker.scale.y:.3f} x {marker.scale.z:.3f})")
        
        # 마커 발행
        self.marker_pub.publish(marker)
    
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
            self.img_coord_label.setText(f"img:({img_x},{img_y})")
            self.depth_label.setText(f"depth:{int(depth_val)}")
            
            world_point = self.pixel_to_world(img_x, img_y, depth_m)
            self.world_point = world_point
            if world_point:
                x_mm = world_point.x * 1000
                y_mm = world_point.y * 1000
                z_mm = world_point.z * 1000
                self.world_coord_label.setText(f"wld:({x_mm:.0f},{y_mm:.0f},{z_mm:.0f})")
                
                # 현재 포인트가 필터 범위 내에 있는지 확인
                if (self.filter_enabled and 
                    self.x_min <= world_point.x <= self.x_max and
                    self.y_min <= world_point.y <= self.y_max and
                    self.z_min <= world_point.z <= self.z_max):
                    # 필터 범위 내 포인트 강조
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
        if hasattr(self, 'cloud_sub'):
            self.cloud_sub.unregister()
        if self.update_timer.isActive():
            self.update_timer.stop()
        if self.marker_timer.isActive():
            self.marker_timer.stop()
        
        # 마지막으로 마커 제거 (삭제)
        self.publish_filter_box_marker(delete=True)