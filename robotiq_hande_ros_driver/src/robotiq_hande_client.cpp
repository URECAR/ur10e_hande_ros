#include "robotiq_hande_client.h"

RobotiqHandEClient::RobotiqHandEClient(ros::NodeHandle& nh) : nh_(nh), status_received_(false) {
    // Initialize ROS communication
    command_pub_ = nh_.advertise<robotiq_hande_ros_driver::HandECommand>("robotiq_hande/command", 10);
    status_sub_ = nh_.subscribe("robotiq_hande/status", 10, &RobotiqHandEClient::statusCallback, this);
    grip_client_ = nh_.serviceClient<robotiq_hande_ros_driver::HandEGrip>("robotiq_hande/grip");
    release_client_ = nh_.serviceClient<robotiq_hande_ros_driver::HandERelease>("robotiq_hande/release");
    
    // Wait for services to be available
    ros::Duration timeout(5.0);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time < timeout)) {
        if (grip_client_.exists() && release_client_.exists()) {
            ROS_INFO("RobotiqHandEClient: Services are available");
            break;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    // Wait for status message
    start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time < timeout)) {
        if (status_received_) {
            ROS_INFO("RobotiqHandEClient: Status received");
            break;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    if (!status_received_) {
        ROS_WARN("RobotiqHandEClient: No status received yet");
    }
}

void RobotiqHandEClient::statusCallback(const robotiq_hande_ros_driver::HandEStatus::ConstPtr& msg) {
    status_ = *msg;
    status_received_ = true;
}

bool RobotiqHandEClient::open(uint8_t speed) {
    robotiq_hande_ros_driver::HandERelease srv;
    srv.request.speed = speed;
    
    if (!release_client_.call(srv)) {
        ROS_ERROR("RobotiqHandEClient: Failed to call release service");
        return false;
    }
    
    return srv.response.success;
}

bool RobotiqHandEClient::close(uint8_t speed, uint8_t force) {
    robotiq_hande_ros_driver::HandEGrip srv;
    srv.request.speed = speed;
    srv.request.force = force;
    
    if (!grip_client_.call(srv)) {
        ROS_ERROR("RobotiqHandEClient: Failed to call grip service");
        return false;
    }
    
    return srv.response.success;
}

bool RobotiqHandEClient::gotoPosition(float position_mm, uint8_t speed, uint8_t force) {
    robotiq_hande_ros_driver::HandECommand cmd_msg;
    cmd_msg.command = "goto";
    cmd_msg.position = position_mm;
    cmd_msg.speed = speed;
    cmd_msg.force = force;
    
    command_pub_.publish(cmd_msg);
    return true;
}

bool RobotiqHandEClient::stop() {
    robotiq_hande_ros_driver::HandECommand cmd_msg;
    cmd_msg.command = "stop";
    
    command_pub_.publish(cmd_msg);
    return true;
}

float RobotiqHandEClient::getCurrentPosition() const {
    if (!status_received_) {
        return -1.0f;
    }
    
    // Convert position (0-255) to mm (5-85mm)
    return 5.0f + (status_.position / 255.0f) * 80.0f;
}

bool RobotiqHandEClient::isMoving() const {
    if (!status_received_) {
        return false;
    }
    
    return status_.is_moving;
}

bool RobotiqHandEClient::isActivated() const {
    if (!status_received_) {
        return false;
    }
    
    return status_.is_activated;
}

bool RobotiqHandEClient::isObjectDetected() const {
    if (!status_received_) {
        return false;
    }
    
    return status_.object_detected;
}

uint8_t RobotiqHandEClient::getFault() const {
    if (!status_received_) {
        return 0;
    }
    
    return status_.fault;
}

bool RobotiqHandEClient::waitForMotionComplete(double timeout) {
    if (!status_received_) {
        ROS_ERROR("RobotiqHandEClient: Cannot wait for motion, no status received");
        return false;
    }
    
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10); // 10Hz
    
    while (ros::ok() && (ros::Time::now() - start_time < ros::Duration(timeout))) {
        ros::spinOnce();
        
        if (!status_.is_moving) {
            return true;
        }
        
        rate.sleep();
    }
    
    ROS_WARN("RobotiqHandEClient: Timeout waiting for motion to complete");
    return false;
}