#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import robotiq_gripper
from robotiq_hande_ros_driver.srv import gripper_service, gripper_serviceResponse

class HandEGripper:
    def __init__(self):
        rospy.init_node('hand_e_gripper_node', anonymous=False)

        # get robot IP from parameter server
        ip = rospy.get_param('~robot_ip')

        # initialize the gripper driver
        self.gripper = robotiq_gripper.RobotiqGripper()
        rospy.loginfo('Connecting to the gripper...')
        self.gripper.connect(ip, 63352)

        rospy.loginfo('Activating the gripper...')
        self.gripper.activate(auto_calibrate=False)

        # advertise the gripper_service
        self.gripper_server = rospy.Service(
            'gripper_service',
            gripper_service,
            self.serverCallback
        )
        rospy.loginfo('Gripper service ready to receive requests')

    def serverCallback(self, request):
        pos = request.position
        speed = request.speed
        force = request.force

        # validate inputs
        if speed <= 0 or speed > 255:
            return gripper_serviceResponse('invalid speed value. Valid range (0,255]')
        if force <= 0 or force > 255:
            return gripper_serviceResponse('invalid force value. Valid range (0,255]')
        if pos < 0 or pos > 255:
            return gripper_serviceResponse('invalid position value. Valid range [0,255]')

        rospy.loginfo(f'Moving gripper: position={pos}, speed={speed}, force={force}')
        self.gripper.move_and_wait_for_pos(pos, speed, force)

        return gripper_serviceResponse('Done')

if __name__ == '__main__':
    gripper_obj = HandEGripper()
    rospy.spin()