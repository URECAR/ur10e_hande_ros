#ifndef ROBOTIQ_HANDE_CLIENT_H
#define ROBOTIQ_HANDE_CLIENT_H

#include <ros/ros.h>
#include <string>
#include "robotiq_hande_ros_driver/HandEStatus.h"
#include "robotiq_hande_ros_driver/HandECommand.h"
#include "robotiq_hande_ros_driver/HandEGrip.h"
#include "robotiq_hande_ros_driver/HandERelease.h"

class RobotiqHandEClient {
public:
    RobotiqHandEClient(ros::NodeHandle& nh);
    ~RobotiqHandEClient() = default;
    
    // Commands
    bool open(uint8_t speed = 0);
    bool close(uint8_t speed = 0, uint8_t force = 0);
    bool gotoPosition(float position_mm, uint8_t speed = 0, uint8_t force = 0);
    bool stop();
    
    // Position & status
    float getCurrentPosition() const;
    bool isMoving() const;
    bool isActivated() const;
    bool isObjectDetected() const;
    uint8_t getFault() const;
    
    // Wait for actions to complete
    bool waitForMotionComplete(double timeout = 5.0);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
    ros::Subscriber status_sub_;
    ros::ServiceClient grip_client_;
    ros::ServiceClient release_client_;
    
    // Status information
    robotiq_hande_ros_driver::HandEStatus status_;
    bool status_received_;
    
    // Status callback
    void statusCallback(const robotiq_hande_ros_driver::HandEStatus::ConstPtr& msg);
};

#endif // ROBOTIQ_HANDE_CLIENT_H