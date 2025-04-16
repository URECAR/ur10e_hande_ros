#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import signal
import rospy
import moveit_commander
from PyQt5.QtWidgets import QApplication, QMessageBox

# Add path to the package source
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))

# Import required modules
from test_package.robot_controller import URRobotController
from test_package.gripper_controller import GripperController
from test_package.pose_manager import PoseManager
from test_package.gui import URControlGUI  # Assuming you'll update this to remove socket-related code


def init_ros_node():
    """Initialize ROS node"""
    if not rospy.core.is_initialized():
        rospy.init_node('ur_gripper_controller', anonymous=True)


def init_moveit():
    """Initialize MoveIt"""
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("MoveIt Commander initialized successfully")
        return True
    except Exception as e:
        rospy.logerr(f"MoveIt Commander initialization failed: {e}")
        return False


def check_gripper_service():
    """Check if gripper service is available"""
    try:
        # Wait for service with short timeout
        rospy.wait_for_service('hande_gripper/control', timeout=2.0)
        return True
    except rospy.ROSException:
        return False


def check_camera_topics():
    """Check if camera topics are available"""
    topics = ['/camera/color/image_raw', '/camera/depth/image_rect_raw']
    # Wait briefly to allow topic discovery
    rospy.sleep(0.5)
    
    # Get published topics
    published_topics = dict(rospy.get_published_topics())
    
    # Check if required topics are available
    missing_topics = [t for t in topics if t not in published_topics]
    
    if missing_topics:
        rospy.logwarn(f"Camera topics not found: {', '.join(missing_topics)}")
        return False
    
    return True


def cleanup_resources(robot_controller=None, gripper_controller=None):
    """Clean up resources when the program exits"""
    if robot_controller:
        try:
            robot_controller.cleanup()  # Remove workspace boxes
        except Exception as e:
            rospy.logerr(f"Error cleaning up robot controller: {e}")
    
    if gripper_controller:
        try:
            gripper_controller.close()
        except Exception as e:
            rospy.logerr(f"Error cleaning up gripper controller: {e}")
    
    # Shutdown ROS if it's still running
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Application closed")
    
    # Shutdown MoveIt
    try:
        moveit_commander.roscpp_shutdown()
    except:
        pass


def main():
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Set SIGINT handler (allow Ctrl+C to terminate)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Initialize ROS node
    init_ros_node()
    
    # Initialize MoveIt
    if not init_moveit():
        rospy.logerr("MoveIt initialization failed, exiting.")
        sys.exit(1)
    
    # Process command line arguments
    robot_ip = "192.168.56.101"  # Default IP (virtual mode)
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    
    # Check gripper service
    if not check_gripper_service():
        # Show warning dialog
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText("Gripper service: '/hande_gripper/control' not found.")
        msg_box.setInformativeText("Check if hande_driver.py is running. Continue anyway?")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.Yes)
        
        # Continue or exit based on user choice
        if msg_box.exec() == QMessageBox.No:
            sys.exit(0)
    
    # Check camera topics
    if not check_camera_topics():
        # Show warning dialog
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setText("Camera topics not found.")
        msg_box.setInformativeText("Check if the Realsense camera node is running.\nContinue without camera functionality?")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.Yes)
        
        # Continue or exit based on user choice
        if msg_box.exec() == QMessageBox.No:
            sys.exit(0)
    
    try:
        # Initialize robot controller
        robot_controller = URRobotController()
        robot_controller.add_box('workspace_box', [0, 0, 0], [0.795, 0.6, 1.0], center=True)
        robot_controller.add_box('workspace_box2', [0.6, 0.066, 0.151], [0.50, 0.50, 1.151], center=True)

        # Initialize gripper controller
        gripper_controller = GripperController(robot_ip)
        
        # Create and show GUI
        gui = URControlGUI(robot_controller, gripper_controller)
        gui.show()

        # Register cleanup function
        app.aboutToQuit.connect(lambda: cleanup_resources(robot_controller, gripper_controller))

        # Run QApplication event loop
        sys.exit(app.exec_())
    
    except Exception as e:
        rospy.logerr(f"Initialization error: {e}")
        cleanup_resources()
        sys.exit(1)
    
    finally:
        # Ensure resources are cleaned up
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Application closed")
        try:
            moveit_commander.roscpp_shutdown()
        except:
            pass


if __name__ == '__main__':
    main()