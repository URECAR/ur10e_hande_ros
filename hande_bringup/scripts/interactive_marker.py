#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

class CameraInteractiveMarker:
    def __init__(self):
        # Initialize the node
        rospy.init_node('camera_interactive_marker')
        
        # Create an interactive marker server
        self.server = InteractiveMarkerServer("camera_control")
        
        # Create a menu handler
        self.menu_handler = MenuHandler()
        
        # Add menu entries
        self.menu_handler.insert("Reset Position", callback=self.process_feedback)
        self.menu_handler.insert("Publish Transform", callback=self.process_feedback)
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Initial camera position
        self.camera_pose = geometry_msgs.msg.Pose()
        self.camera_pose.position.x = 0.01
        self.camera_pose.position.y = -0.06
        self.camera_pose.position.z = 0.046
        
        # Initial camera orientation (specified as quaternion)
        # Values from the static transform publisher in ur10e_hande_bringup.launch
        # args="0.01 -0.06 0.046 1.57 -1.55 -3.24 tool0 camera_link"
        quaternion = tf.transformations.quaternion_from_euler(1.57, -1.55, -3.24)
        self.camera_pose.orientation.x = quaternion[0]
        self.camera_pose.orientation.y = quaternion[1]
        self.camera_pose.orientation.z = quaternion[2]
        self.camera_pose.orientation.w = quaternion[3]
        
        # Create the interactive marker
        self.make_6dof_marker()
        
        # 'Commit' changes and send to all clients
        self.server.applyChanges()
        
        # Start publishing the camera transform
        rospy.Timer(rospy.Duration(0.1), self.publish_camera_tf)
        
        rospy.loginfo("Camera interactive marker initialized")
        
    def make_6dof_marker(self):
        """Create a 6DOF interactive marker for camera_link"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "tool0"
        int_marker.pose = self.camera_pose
        int_marker.scale = 0.1
        int_marker.name = "camera_link_control"
        int_marker.description = "Camera Link Control\n(6DOF)"
        
        # Create a sphere marker to represent the camera
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.02
        sphere_marker.scale.y = 0.02
        sphere_marker.scale.z = 0.02
        sphere_marker.color.r = 0.5
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.5
        sphere_marker.color.a = 1.0
        
        # Create a small arrow to show camera's viewing direction
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.03  # shaft length
        arrow_marker.scale.y = 0.01  # shaft diameter
        arrow_marker.scale.z = 0.01  # head diameter
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        arrow_marker.pose.position.x = 0.0  # offset the arrow a bit
        
        # Create basic controls
        self.add_box_control(int_marker)
        
        # Create control to move in X axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Create control to move in Y axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Create control to move in Z axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Create control to rotate around X axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Create control to rotate around Y axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Create control to rotate around Z axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Add the interactive marker to the server
        self.server.insert(int_marker, self.process_feedback)
        
        # Apply the menu handler to the marker
        self.menu_handler.apply(self.server, int_marker.name)

    def add_box_control(self, int_marker):
        """Add a box visualization control to the interactive marker"""
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(self.make_camera_marker())
        int_marker.controls.append(box_control)
    
    def make_camera_marker(self):
        """Create a camera-shaped marker"""
        # Create a basic marker for the camera body
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.02
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker

    def process_feedback(self, feedback):
        """Handle the feedback from the interactive marker"""
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update the camera pose
            self.camera_pose = feedback.pose
            rospy.loginfo(f"New camera pose: Position({feedback.pose.position.x:.3f}, {feedback.pose.position.y:.3f}, {feedback.pose.position.z:.3f})")
            
            # Extract Euler angles for more readable output
            quaternion = (
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll, pitch, yaw = euler
            rospy.loginfo(f"Orientation (RPY): ({roll:.3f}, {pitch:.3f}, {yaw:.3f})")
            
            # Output the static transform publisher command for this pose
            transform_cmd = f"rosrun tf2_ros static_transform_publisher {feedback.pose.position.x} {feedback.pose.position.y} {feedback.pose.position.z} {roll} {pitch} {yaw} tool0 camera_link"
            rospy.loginfo(f"Use this command to set this transform: {transform_cmd}")
            
        elif feedback.menu_entry_id == 1:  # Reset Position
            # Reset the camera pose to its initial value
            self.camera_pose.position.x = 0.01
            self.camera_pose.position.y = -0.06
            self.camera_pose.position.z = 0.046
            
            quaternion = tf.transformations.quaternion_from_euler(1.57, -1.55, -3.24)
            self.camera_pose.orientation.x = quaternion[0]
            self.camera_pose.orientation.y = quaternion[1]
            self.camera_pose.orientation.z = quaternion[2]
            self.camera_pose.orientation.w = quaternion[3]
            
            # Update the marker pose
            self.server.setPose("camera_link_control", self.camera_pose)
            self.server.applyChanges()
            
            rospy.loginfo("Camera pose reset to default")
            
        elif feedback.menu_entry_id == 2:  # Publish Transform
            quaternion = (
                self.camera_pose.orientation.x,
                self.camera_pose.orientation.y,
                self.camera_pose.orientation.z,
                self.camera_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll, pitch, yaw = euler
            
            transform_cmd = f"rosrun tf2_ros static_transform_publisher {self.camera_pose.position.x} {self.camera_pose.position.y} {self.camera_pose.position.z} {roll} {pitch} {yaw} tool0 camera_link"
            rospy.loginfo(f"Static transform command: {transform_cmd}")
            
    def publish_camera_tf(self, event):
        """Publish the camera transform based on the interactive marker position"""
        # Create a transform message
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "tool0"
        transform.child_frame_id = "camera_link"
        
        # Copy position
        transform.transform.translation.x = self.camera_pose.position.x
        transform.transform.translation.y = self.camera_pose.position.y
        transform.transform.translation.z = self.camera_pose.position.z
        
        # Copy orientation
        transform.transform.rotation = self.camera_pose.orientation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        camera_marker = CameraInteractiveMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass