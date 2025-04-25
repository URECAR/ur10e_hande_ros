#!/usr/bin/env python3

import rospy
import socket
import threading
import time
import sys
from std_msgs.msg import Float64
from robotiq_hande_ros_driver.srv import HandECmd, HandECmdResponse

class RobotiqHandEDriver:
    def __init__(self):
        self.HEADER_SIZE = 3
        self.lock = threading.Lock()
        self.statusBusy = 0
        self.statusOBJ = 0
        self.pos = 0
        self.cur = 0
        self.s_get = None
        self.s_set = None
        self.data = None
        self.statusUpdate = False
        self.controllerVersion = 0
        self.initFlag = False
        self.gripSetFlag = False
        
        try:
            rospy.init_node('robotiq_hande_driver')
        except:
            print('Node already initialized')
            
        # Get parameters
        self.gripper_ip = rospy.get_param('~gripper_ip', '192.168.1.140')
        
        # Publishers
        self.pub_position = rospy.Publisher('gripper/position', Float64, queue_size=1)
        self.pub_current = rospy.Publisher('gripper/current', Float64, queue_size=1)
        
        # Service
        self.gripper_service = rospy.Service('gripper/cmd', HandECmd, self.gripper_command_callback)
        
        # Connect to gripper
        if not self.connect(self.gripper_ip):
            rospy.logerr("Failed to connect to gripper at {}".format(self.gripper_ip))
            return
            
        rospy.loginfo("Connected to Robotiq Hand-E gripper at {}".format(self.gripper_ip))
        self.initFlag = True
        
        # Start status polling thread
        self.running = True
        self.status_thread = threading.Thread(target=self.status_polling)
        self.status_thread.daemon = True
        self.status_thread.start()
        
    def float_to_hex(self, f):
        return hex(int(f * 255))[2:]
    
    def set_pos(self, req):
        self.req_pos = req.rPR
        self.req_vel = req.rSP
        self.req_force = req.rFR
        
        # gripper position range 0(closed) to 255(open)
        # usually 0 to 100 actuation in mm
        # speed range 0 to 255
        self.p = int(self.req_pos * 255)
        self.s = int(self.req_vel * 255)
        self.f = int(self.req_force * 255)
        
        hhex = self.float_to_hex(1.0)
        phex = self.float_to_hex(self.req_pos)
        shex = self.float_to_hex(self.req_vel)
        fhex = self.float_to_hex(self.req_force)
        
        if len(phex) == 1:
            phex = '0' + phex
        if len(shex) == 1:
            shex = '0' + shex
        if len(fhex) == 1:
            fhex = '0' + fhex    
        
        r = '091001' + hhex + phex + shex + fhex
        rospy.logdebug(r)
        br = bytes.fromhex(r)
        checksum = 0
        for b in br:
            checksum = checksum ^ b
        
        bc = bytes([checksum])
        
        msg = br + bc
        self.statusUpdate = False
        ret = self.s_set.send(msg)
        
        rospy.logdebug(f'Sent {ret} bytes. msg = {msg}')
        
        return HandECmdResponse()
    
    def getTLV(self, data):
        # Takes a byte array of the TLV format, returns a list of the tag, length, and value as decimal values
        tag = int(data[0])
        val_len = int(data[1])
        val = []
        
        for i in range(val_len):
            val.append(data[2 + i])
        
        return [tag, val_len, val]
    
    def get_status(self):
        try:
            if not self.initFlag:
                return
            
            self.statusUpdate = False
            
            if self.s_get is None:
                return
            
            # Request
            self.s_get.send(b'\x09\x03\x00\x00\x00')
            
            header = self.s_get.recv(self.HEADER_SIZE)
            
            # Verify response
            if header[0] != 0x09:
                rospy.logwarn('Bad header: {}'.format(header))
                return
                
            msg_len = header[1]
            payload = self.s_get.recv(msg_len)
            
            data_bytes = header + payload
            
            tlv_list = []
            idx = 3
            
            while idx < len(data_bytes):
                tlv = self.getTLV(data_bytes[idx:])
                tlv_list.append(tlv)
                idx = idx + 2 + tlv[1]
                
            for tlv in tlv_list:
                if tlv[0] == 1:
                    self.gripperStatus = tlv[2][0]
                    # fault status
                    self.gripperFault = tlv[2][1]
                    # position request echo
                    self.posEcho = tlv[2][2]
                    # position
                    self.pos = tlv[2][3]
                    # current
                    self.cur = tlv[2][4]
            
            if self.pos > 0:
                self.pub_position.publish(self.pos / 255.0)
                self.pub_current.publish(self.cur)
                self.statusUpdate = True
                
        except Exception as e:
            rospy.logerr("Exception in get_status: {}".format(e))
    
    def gripper_command_callback(self, req):
        if not self.initFlag:
            rospy.logwarn('Wait for the gripper to initialize')
            return HandECmdResponse()
            
        with self.lock:
            ret = self.set_pos(req)
            
        return HandECmdResponse()
    
    def connect(self, host):
        self.s_get = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            status = self.s_get.connect_ex((host, 1000))
            if status != 0:
                rospy.logerr('Cannot connect to the gripper at {}, status: {}'.format(host, status))
                self.s_get.close()
                self.s_get = None
                return False
                
            self.s_set = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            status = self.s_set.connect_ex((host, 2000))
            
            if status != 0:
                rospy.logerr('Cannot connect to the gripper at {}, status: {}'.format(host, status))
                self.s_get.close()
                self.s_get = None
                return False
                
            rospy.loginfo('Connected to the hand-e gripper')
            return True
            
        except Exception as e:
            if self.s_get is not None:
                self.s_get.close()
                self.s_get = None
            rospy.logerr('Error: {}'.format(e))
            return False
    
    def status_polling(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while self.running and not rospy.is_shutdown():
            self.get_status()
            rate.sleep()
    
    def shutdown(self):
        self.running = False
        if self.status_thread.is_alive():
            self.status_thread.join(1.0)
        
        if self.s_get:
            self.s_get.close()
        if self.s_set:
            self.s_set.close()


if __name__ == "__main__":
    try:
        driver = RobotiqHandEDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'driver' in locals():
            driver.shutdown()