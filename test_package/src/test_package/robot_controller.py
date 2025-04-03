#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import copy
from math import degrees, radians
from PyQt5.QtCore import QObject, QTimer, pyqtSignal, QThread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_commander


class PlanningThread(QThread):
    """로봇 움직임 계획을 위한 스레드"""
    planning_finished = pyqtSignal(bool, str, object)  # 성공 여부, 메시지, 계획 객체
    
    def __init__(self, move_group, plan_type, values, velocity_scaling, acceleration_scaling):
        super().__init__()
        self.move_group = move_group
        self.plan_type = plan_type  # 'joint', 'pose', 또는 'cartesian'
        self.values = values
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.move_group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
    def execute_cartesian_path(self, waypoints):
        """카테시안 경로 실행 - 여러 번 시도하는 로직"""
        max_attempts = 5
        
        # 속도에 따른 eef_step 조정 (더 느린 속도일 경우 더 작은 스텝으로)
        base_eef_step = 0.004  # 기본 스텝 크기
        scaled_step = base_eef_step * max(0.2, self.velocity_scaling)  # 최소 스텝 크기 보장
        
        for attempt in range(max_attempts):
            for eef_step in [scaled_step, scaled_step*2]:  # 조정된 스텝 크기 사용
                for jump_threshold in [0, 1.57]:  # 두 가지 점프 임계값 시도
                    (plan, fraction) = self.move_group.compute_cartesian_path(
                        waypoints, 
                        eef_step, 
                        jump_threshold, 
                        avoid_collisions=True  # 충돌 회피 사용
                    )
                    if fraction == 1.0:  # 100% 경로 생성 성공
                        rospy.loginfo(f"카테시안 경로 계획 성공: {attempt+1}회 시도, eef_step: {eef_step}, jump_threshold: {jump_threshold}")
                        return (plan, fraction)
                    elif fraction > 0.95:  # 95% 이상도 허용
                        rospy.loginfo(f"카테시안 경로 부분 계획: {attempt+1}회 시도, {fraction:.2%} 커버, eef_step: {eef_step}, jump_threshold: {jump_threshold}")
                        return (plan, fraction)
            time.sleep(0.05)  # 다음 시도 전 잠시 대기
        
        # 모든 시도 실패
        rospy.logerr("카테시안 경로 생성 실패: 목표 위치가 도달 가능한지 확인하세요.")
        return (None, 0)
    
    def run(self):
        """스레드 실행 함수"""
        try:
            # 속도 및 가속도 스케일링 설정
            self.move_group.set_max_velocity_scaling_factor(self.velocity_scaling)
            self.move_group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
            
            if self.plan_type == 'joint':
                # 각도를 라디안으로 변환
                joint_goal = [radians(float(value)) for value in self.values]
                
                # 계획 그룹에 목표 조인트 값 설정
                self.move_group.set_joint_value_target(joint_goal)
                
                # 계획 생성
                plan = self.move_group.plan()
                
            elif self.plan_type in ['pose', 'cartesian']:
                # 위치와 방향 추출 (mm -> m 변환)
                x, y, z, rx, ry, rz = [float(val) for val in self.values]
                x /= 1000.0
                y /= 1000.0
                z /= 1000.0
                
                # 오일러 각도를 쿼터니언으로 변환 (도 -> 라디안)
                rx_rad, ry_rad, rz_rad = radians(rx), radians(ry), radians(rz)
                quat = quaternion_from_euler(rx_rad, ry_rad, rz_rad)
                
                # 목표 포즈 설정
                target_pose = Pose()
                target_pose.position.x = x
                target_pose.position.y = y
                target_pose.position.z = z
                target_pose.orientation.x = quat[0]
                target_pose.orientation.y = quat[1]
                target_pose.orientation.z = quat[2]
                target_pose.orientation.w = quat[3]
                
                if self.plan_type == 'pose':
                    # 일반 포즈 이동 계획 - 목표 포즈 설정
                    self.move_group.set_pose_target(target_pose)
                    plan = self.move_group.plan()
                else:
                    # Cartesian 경로 계획 - 직선 경로 계획
                    # 현재 포즈에서 목표 포즈까지의 직선 경로 계획
                    waypoints = [target_pose]
                    
                    # 향상된 Cartesian 경로 계획 기법 사용
                    (plan, fraction) = self.execute_cartesian_path(waypoints)
                    
                    # 직선 경로 계획 성공 여부 판단
                    if fraction >= 0.95:  # 95% 이상 경로를 계획했으면 성공
                        success = True
                        message = f"Cartesian 경로 계획 성공 ({fraction:.2%} 경로 커버)"
                        self.planning_finished.emit(success, message, plan)
                        return
                    else:
                        success = False
                        message = f"Cartesian 경로 계획 실패 ({fraction:.2%} 경로만 커버)"
                        self.planning_finished.emit(success, message, None)
                        return
            
            # 계획 성공 확인 (반환 형식은 ROS 버전에 따라 다름)
            if isinstance(plan, tuple):
                # ROS Melodic 이상에서는 (success, trajectory_msg) 형식으로 반환
                success = plan[0]
                trajectory = plan[1]
                plan = trajectory  # 추후 실행을 위해 trajectory 부분만 저장
            else:
                # 이전 버전에서는 trajectory_msg만 반환하며, 비어있지 않으면 성공으로 간주
                success = len(plan.joint_trajectory.points) > 0
            
            # 결과 메시지 생성
            if self.plan_type == 'joint':
                message = f"조인트 이동 계획 " + ("성공" if success else "실패")
            elif self.plan_type == 'pose':
                message = f"TCP 이동 계획 " + ("성공" if success else "실패")
            else:
                message = f"알 수 없는 계획 유형 ({self.plan_type})"
                
            self.planning_finished.emit(success, message, plan)
            
        except Exception as e:
            # 계획 타입에 맞는 에러 메시지 생성
            if self.plan_type == 'joint':
                error_msg = f"조인트 이동 계획 오류: {str(e)}"
            elif self.plan_type == 'pose':
                error_msg = f"TCP 이동 계획 오류: {str(e)}"
            elif self.plan_type == 'cartesian':
                error_msg = f"Cartesian 직선 이동 계획 오류: {str(e)}"
            else:
                error_msg = f"{self.plan_type.capitalize()} 이동 계획 오류: {str(e)}"
                
            self.planning_finished.emit(False, error_msg, None)

class ExecutionThread(QThread):
    """로봇 계획 실행을 위한 스레드"""
    execution_finished = pyqtSignal(bool, str)  # 실행 성공 여부 및 메시지
    
    def __init__(self, move_group, plan):
        super().__init__()
        self.move_group = move_group
        self.plan = plan
    
    def run(self):
        """스레드 실행 함수"""
        try:
            # 계획 실행 (wait=True로 설정하여 완료될 때까지 기다림)
            success = self.move_group.execute(self.plan, wait=True)
            
            # 성공 여부에 따라 신호 발생
            if success:
                self.execution_finished.emit(True, "계획 실행 성공")
            else:
                self.execution_finished.emit(False, "계획 실행 실패")
        
        except Exception as e:
            self.execution_finished.emit(False, f"계획 실행 오류: {str(e)}")
            rospy.logerr(f"계획 실행 오류: {e}")


class URRobotController(QObject):
    """UR 로봇 상태 모니터링 및 제어 클래스 (MoveIt 사용)"""
    pose_updated = pyqtSignal(float, float, float, float, float, float)  # x, y, z, rx, ry, rz
    joints_updated = pyqtSignal(list)  # 6개 조인트 값 리스트
    program_state_updated = pyqtSignal(str)  # 프로그램 상태
    planning_result = pyqtSignal(bool, str)  # 계획 성공 여부 및 메시지
    
    def __init__(self):
        super().__init__()
        self.initialize_moveit()
        
        # 현재 계획 저장 변수
        self.current_plan = None
        
        # 조인트 상태 구독
        self.joint_states = None
        self.joint_states_time = None
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # UR 프로그램 상태 구독
        self.program_state = "UNKNOWN"
        rospy.Subscriber('/ur_hardware_interface/robot_program_running', Bool, self.program_state_callback)
        
        # 모니터링 타이머
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_robot_state)
        self.update_timer.start(100)  # 10Hz
        
        rospy.loginfo(f"UR 로봇 제어기 초기화 완료 - 계획 프레임: {self.base_frame}, 엔드 이펙터: {self.ee_frame}")
    
    def initialize_moveit(self):
        """MoveIt 초기화"""
        try:
            # MoveIt 초기화 (robot commander 및 move_group)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group = moveit_commander.MoveGroupCommander("manipulator")  # UR 로봇의 플래닝 그룹 이름
            
            # 기준 프레임과 엔드 이펙터 프레임
            self.base_frame = self.move_group.get_planning_frame()
            self.ee_frame = self.move_group.get_end_effector_link()
            
            # 추가된 상자 목록 초기화
            self.added_boxes = []
            
            # 로봇 작업 영역에 기본 상자 추가
            self.add_box('workspace_box', [0, 0, 0], [0.795, 0.6, 1.0], center=True)
            self.add_box('workspace_box2', [0.66, 0.055, 0.151], [0.50, 0.50, 1.151], center=True)
        except Exception as e:
            rospy.logerr(f"MoveIt 초기화 오류: {e}")
            raise
    
    def joint_states_callback(self, msg):
        """조인트 상태 콜백 함수"""
        self.joint_states = msg
        self.joint_states_time = rospy.Time.now()
        
    def program_state_callback(self, msg):
        """UR 프로그램 상태 콜백 함수"""
        if msg.data:
            self.program_state = "RUNNING"
        else:
            self.program_state = "STOPPED"
        self.program_state_updated.emit(self.program_state)
    
    def update_robot_state(self):
        """로봇 상태 업데이트 및 신호 전송 (MoveIt 사용)"""
        try:
            # 타임스탬프 확인 (1초 이상 지난 데이터는 무시)
            if self.joint_states_time is not None:
                time_diff = (rospy.Time.now() - self.joint_states_time).to_sec()
                if time_diff > 1.0:
                    rospy.logwarn(f"데이터가 오래됨: {time_diff:.2f}초")
                    return
            
            # 엔드 이펙터 포즈 가져오기 (MoveIt 사용)
            current_pose = self.move_group.get_current_pose().pose
            
            # 포즈에서 위치와 방향 추출
            x = current_pose.position.x * 1000  # 미터 -> 밀리미터
            y = current_pose.position.y * 1000
            z = current_pose.position.z * 1000
            
            # 쿼터니언에서 오일러 각도로 변환 (RPY)
            quat = [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            ]
            euler = euler_from_quaternion(quat)
            rx, ry, rz = degrees(euler[0]), degrees(euler[1]), degrees(euler[2])
            
            # 신호 발생
            self.pose_updated.emit(x, y, z, rx, ry, rz)
            
            # 조인트 값 가져오기 (MoveIt에서 직접 가져오기)
            current_joints = self.move_group.get_current_joint_values()
            joint_values = [degrees(angle) for angle in current_joints]
            
            # 모든 조인트 값이 준비되었으면 신호 발생
            if len(joint_values) == 6:
                self.joints_updated.emit(joint_values)
        
        except Exception as e:
            rospy.logerr(f"상태 업데이트 오류: {e}")
    
    def plan_joint_movement(self, joint_values, velocity_scaling=0.25, acceleration_scaling=0.25):
        """조인트 공간에서 로봇 움직임 계획 (스레드 사용)"""
        # 계획 스레드 생성 및 시작
        self.planning_thread = PlanningThread(
            self.move_group, 'joint', joint_values, velocity_scaling, acceleration_scaling
        )
        self.planning_thread.planning_finished.connect(self._on_planning_finished)
        self.planning_thread.start()
        
        # 계획 진행 중임을 알림
        self.planning_result.emit(False, "조인트 이동 계획 진행 중...")
    
    def plan_pose_movement(self, pose_values, velocity_scaling=0.25, acceleration_scaling=0.25, cartesian=False):
        """TCP 위치로 로봇 움직임 계획 (스레드 사용)
        
        Args:
            pose_values: 목표 포즈 값 (x, y, z, rx, ry, rz)
            velocity_scaling: 속도 스케일링 (0-1)
            acceleration_scaling: 가속도 스케일링 (0-1)
            cartesian: True이면 직선 이동(Cartesian), False이면 일반 이동
        """
        # 계획 타입 설정 (cartesian 또는 pose)
        plan_type = 'cartesian' if cartesian else 'pose'
        
        # Cartesian 속도 제한 설정 (직선 이동의 경우)
        if cartesian:
            # max_cartesian_speed를 m/s 단위로 설정 (예: 0.1 m/s)
            # velocity_scaling은 0-1 사이의 값이므로 적절한 최대값을 곱합니다
            max_cartesian_speed = 0.25 * velocity_scaling  # 최대 0.25 m/s
            self.move_group.limit_max_cartesian_link_speed(max_cartesian_speed)
        
        # 계획 스레드 생성 및 시작
        self.planning_thread = PlanningThread(
            self.move_group, plan_type, pose_values, velocity_scaling, acceleration_scaling
        )
        self.planning_thread.planning_finished.connect(self._on_planning_finished)
        self.planning_thread.start()
        
        # 계획 진행 중임을 알림
        self.planning_result.emit(False, "Cartesian 직선 이동 계획 진행 중..." if cartesian else "TCP 이동 계획 진행 중...")
        
    def _on_planning_finished(self, success, message, plan):
        """계획 스레드 완료 콜백"""
        if success:
            self.current_plan = plan
        else:
            self.current_plan = None
        
        # 결과 전달
        self.planning_result.emit(success, message)
    
    def execute_plan(self):
        """현재 계획 실행 (스레드 사용)"""
        if self.current_plan is None:
            self.planning_result.emit(False, "실행할 계획이 없습니다. 먼저 계획을 생성하세요.")
            return False
        
        try:
            # 실행 스레드 생성 및 시작
            self.execution_thread = ExecutionThread(self.move_group, self.current_plan)
            self.execution_thread.execution_finished.connect(self._on_execution_finished)
            self.execution_thread.start()
            
            # 실행 중임을 알림
            self.planning_result.emit(False, "계획 실행 중...")
            return True
            
        except Exception as e:
            self.planning_result.emit(False, f"계획 실행 오류: {str(e)}")
            rospy.logerr(f"계획 실행 오류: {e}")
            return False
    
    def _on_execution_finished(self, success, message):
        """실행 스레드 완료 콜백"""
        # 결과 전달
        self.planning_result.emit(success, message)

    def add_box(self, name, position, size, center=True):
        """
        작업 영역에 상자 추가
        
        Args:
            name (str): 상자 이름
            position (list): 상자 위치 [x, y, z]
            size (list): 상자 크기 [x, y, z]
            center (bool): 위치가 상자의 중심인지 여부 (기본값: True)
        
        Returns:
            bool: 상자 추가 성공 여부
        """
        try:
            # 상자 포즈 설정
            box_pose = PoseStamped()
            box_pose.header.frame_id = self.base_frame
            box_pose.header.stamp = rospy.Time.now()
            
            # 위치 설정 (center 옵션에 따라 계산)
            if center:
                # 위치를 중심점으로 설정
                box_pose.pose.position.x = position[0]
                box_pose.pose.position.y = position[1]
                box_pose.pose.position.z = position[2] - size[2]/2
            else:
                # 위치를 좌하단 모서리로 설정 (중심점 계산)
                box_pose.pose.position.x = position[0] + size[0]/2
                box_pose.pose.position.y = position[1] + size[1]/2
                box_pose.pose.position.z = position[2] + size[2]/2
            
            # 방향은 기본값 (회전 없음)
            box_pose.pose.orientation.w = 1.0
            
            # 상자 추가
            self.scene.add_box(name, box_pose, size=size)
            self.added_boxes.append(name)  # 추가된 상자 이름 저장
            
            rospy.loginfo(f"상자 추가: 이름={name}, 크기={size[0]*1000:.0f}x{size[1]*1000:.0f}x{size[2]*1000:.0f}mm")
            return True
            
        except Exception as e:
            rospy.logerr(f"상자 추가 오류: {e}")
            return False
    def cleanup(self):
        """종료 시 리소스 정리"""
        rospy.loginfo("추가된 상자 제거 중...")
        
        # 추가했던 모든 상자 제거
        for box_name in self.added_boxes:
            try:
                self.scene.remove_world_object(box_name)
                rospy.loginfo(f"상자 제거: {box_name}")
            except Exception as e:
                rospy.logerr(f"상자 제거 오류 ({box_name}): {e}")
        
        # 추가된 상자 목록 초기화
        self.added_boxes = []