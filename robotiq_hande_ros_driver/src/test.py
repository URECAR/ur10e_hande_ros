#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from robotiq_hande_ros_driver.srv import gripper_service

if __name__ == '__main__':
    rospy.init_node('gripper_test_node')

    # ensure the service is available
    rospy.loginfo('Waiting for gripper_service...')
    rospy.wait_for_service('gripper_service')

    # create proxy to the ROS service
    gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)
    rospy.loginfo('Connected to gripper_service')

    # open gripper
    response = gripper_srv(position=0, speed=255, force=255)
    rospy.loginfo(f'Open command response: {response}')

    # close gripper
    response = gripper_srv(position=255, speed=255, force=255)
    rospy.loginfo(f'Close command response: {response}')

    # open gripper at slow speed
    response = gripper_srv(position=0, speed=55, force=255)
    rospy.loginfo(f'Open slow response: {response}')

    # close gripper at slow speed
    response = gripper_srv(position=255, speed=55, force=255)
    rospy.loginfo(f'Close slow response: {response}')

    # open with low speed and force
    response = gripper_srv(position=100, speed=5, force=5)
    rospy.loginfo(f'Open low speed/force response: {response}')

    # close with low speed and force
    response = gripper_srv(position=150, speed=5, force=5)
    rospy.loginfo(f'Close low speed/force response: {response}')