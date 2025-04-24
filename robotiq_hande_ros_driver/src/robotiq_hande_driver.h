#ifndef ROBOTIQ_HANDE_DRIVER_H
#define ROBOTIQ_HANDE_DRIVER_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>

#include "robotiq_hande_ros_driver/HandEStatus.h"
#include "robotiq_hande_ros_driver/HandECommand.h"
#include "robotiq_hande_ros_driver/HandEGrip.h"
#include "robotiq_hande_ros_driver/HandERelease.h"

#include <string>
#include <vector>
#include <mutex>

// Modbus RTU communication constants
const uint8_t SLAVE_ID = 0x09;
const uint8_t READ_REGISTERS = 0x03;
const uint8_t WRITE_REGISTER = 0x06;
const uint8_t WRITE_MULTIPLE_REGISTERS = 0x10;

// Register addresses
const uint16_t REG_ACTION_REQUEST = 0x03E8;    // 1000
const uint16_t REG_GRIPPER_STATUS = 0x07D0;    // 2000
const uint16_t REG_FAULT_STATUS = 0x07D1;      // 2001
const uint16_t REG_POS_REQUEST = 0x03E9;       // 1001
const uint16_t REG_SPEED = 0x03EA;             // 1002
const uint16_t REG_FORCE = 0x03EB;             // 1003
const uint16_t REG_POSITION = 0x07D2;          // 2002

// Action requests
const uint8_t ACTION_STOP = 0x00;
const uint8_t ACTION_CLOSE = 0x09;
const uint8_t ACTION_OPEN = 0x0A;
const uint8_t ACTION_GOTO_POS = 0x0B;

// Status bits
const uint8_t GRIPPER_ACTIVATED = 0x01;
const uint8_t OBJECT_DETECTED = 0x04;
const uint8_t GRIPPER_AT_REQUESTED_POS = 0x08;

class RobotiqHandEDriver {
public:
    RobotiqHandEDriver(ros::NodeHandle& nh);
    ~RobotiqHandEDriver();

    bool init(const std::string& port_name);
    void run();
    void shutdown();

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher status_pub_;
    ros::Publisher position_pub_;
    ros::Subscriber command_sub_;
    ros::ServiceServer grip_service_;
    ros::ServiceServer release_service_;
    
    // Serial communication
    int serial_fd_;
    std::string port_name_;
    bool is_connected_;
    
    // Gripper state
    robotiq_hande_ros_driver::HandEStatus status_msg_;
    std::mutex gripper_mutex_;
    
    // Command parameters
    uint8_t gripper_speed_; // 0-255
    uint8_t gripper_force_; // 0-255
    
    // Methods
    bool openSerialPort();
    void closeSerialPort();
    
    // Communication methods
    bool sendModbusRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response);
    std::vector<uint8_t> buildModbusRequest(uint8_t function_code, uint16_t start_addr, uint16_t num_registers = 1, const std::vector<uint16_t>& data = {});
    uint16_t calculateCRC(const std::vector<uint8_t>& data);

    // Gripper control
    bool activateGripper();
    bool deactivateGripper();
    bool readGripperStatus();
    bool sendGripperCommand(uint8_t action_request, uint8_t position = 0);
    bool setGripperPosition(uint8_t position);
    
    // ROS callbacks
    void commandCallback(const robotiq_hande_ros_driver::HandECommand::ConstPtr& msg);
    bool gripCallback(robotiq_hande_ros_driver::HandEGrip::Request& req, robotiq_hande_ros_driver::HandEGrip::Response& res);
    bool releaseCallback(robotiq_hande_ros_driver::HandERelease::Request& req, robotiq_hande_ros_driver::HandERelease::Response& res);
    bool convertPositionToWidth(uint8_t position, float& width_mm);
    uint8_t convertWidthToPosition(float width_mm);
};

#endif // ROBOTIQ_HANDE_DRIVER_H