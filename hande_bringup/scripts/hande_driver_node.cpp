#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>
#include <hande_bringup/HandEGripperCommand.h>
#include <hande_bringup/HandEGripperStatus.h>
#include <hande_bringup/GripperControl.h>
#include <std_msgs/Float64.h>

class HandEGripperDriver {
public:
    HandEGripperDriver() : position_(0.0), target_position_(0.0), speed_(0.5), force_(0.5),
                          activated_(false), moving_(false), object_detected_(false), current_(0.0) {
        // 노드 핸들 초기화
        ros::NodeHandle nh;
        
        // 퍼블리셔 및 서브스크라이버 설정
        status_pub_ = nh.advertise<hande_bringup::HandEGripperStatus>("hande_gripper/status", 10);
        joint_state_pub_ = nh.advertise<std_msgs::Float64>("hande_gripper/joint_state", 10);
        cmd_sub_ = nh.subscribe("hande_gripper/command", 10, &HandEGripperDriver::commandCallback, this);
        
        // 서비스 서버 설정
        control_service_ = nh.advertiseService("hande_gripper/control", &HandEGripperDriver::handleControlService, this);
        
        // 마지막 업데이트 시간 초기화
        last_update_time_ = ros::Time::now();
        
        ROS_INFO("Robotiq Hand-E Gripper virtual driver (C++) initialized");
    }
    
    // 명령 콜백 함수
    void commandCallback(const hande_bringup::HandEGripperCommand::ConstPtr& msg) {
        // 활성화 명령 처리
        if (msg->rACT && !activated_) {
            activated_ = true;
            ROS_INFO("Gripper activated");
        } else if (!msg->rACT && activated_) {
            activated_ = false;
            ROS_INFO("Gripper deactivated");
        }
        
        // 위치, 속도, 힘 명령 처리
        if (activated_ && msg->rGTO) {
            target_position_ = static_cast<double>(msg->rPR) / 255.0;
            speed_ = static_cast<double>(msg->rSP) / 255.0;
            force_ = static_cast<double>(msg->rFR) / 255.0;
            moving_ = true;
            ROS_INFO_STREAM("Moving to position: " << target_position_ 
                          << ", speed: " << speed_ 
                          << ", force: " << force_);
        }
    }
    
    // 서비스 핸들러
    bool handleControlService(hande_bringup::GripperControl::Request& req, 
                             hande_bringup::GripperControl::Response& res) {
        if (!activated_ && req.command_type != 3) {  // 활성화 명령이 아니고 현재 비활성 상태
            res.success = false;
            res.message = "Gripper is not activated";
            return true;
        }
        
        switch (req.command_type) {
            case 0:  // POSITION
                target_position_ = static_cast<double>(req.value) / 255.0;
                moving_ = true;
                res.success = true;
                res.message = "Moving to position: " + std::to_string(target_position_);
                break;
                
            case 1:  // SPEED
                speed_ = static_cast<double>(req.value) / 255.0;
                res.success = true;
                res.message = "Speed set to: " + std::to_string(speed_);
                break;
                
            case 2:  // FORCE
                force_ = static_cast<double>(req.value) / 255.0;
                res.success = true;
                res.message = "Force set to: " + std::to_string(force_);
                break;
                
            case 3:  // ACTIVATE
                activated_ = true;
                res.success = true;
                res.message = "Gripper activated";
                break;
                
            case 4:  // DEACTIVATE
                activated_ = false;
                moving_ = false;
                res.success = true;
                res.message = "Gripper deactivated";
                break;
                
            case 5:  // EMERGENCY_RELEASE
                target_position_ = 0.0;  // 완전 열기
                speed_ = 1.0;  // 최대 속도
                moving_ = true;
                res.success = true;
                res.message = "Emergency release initiated";
                break;
                
            case 6:  // STOP
                moving_ = false;
                res.success = true;
                res.message = "Gripper stopped";
                break;
                
            default:
                res.success = false;
                res.message = "Unknown command type";
        }
        
        return true;
    }
    
    // 위치 업데이트 함수
    void updatePosition() {
        // 그리퍼 이동 시뮬레이션
        if (moving_ && activated_) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_update_time_).toSec();
            last_update_time_ = now;
            
            // 현재 위치와 목표 위치 간의 거리 계산
            double distance = std::abs(target_position_ - position_);
            
            // 이동 방향 결정
            double direction = (target_position_ > position_) ? 1.0 : -1.0;
            
            // 이동 속도 계산 (속도 파라미터 적용)
            double move_step = std::min(distance, speed_ * dt);
            
            // 새 위치 계산
            if (distance > 0.001) {  // 작은 거리는 무시
                position_ += direction * move_step;
                
                // 물체 감지 시뮬레이션 (랜덤하게 물체 감지 가능)
                if (position_ > 0.7 && !object_detected_ && direction > 0) {
                    // 물체 감지 확률: 20% (시간 기반으로 랜덤 시뮬레이션)
                    bool simulate_object_detection = true;  // 파라미터에서 가져올 수 있음
                    if (simulate_object_detection && (static_cast<int>(ros::Time::now().toSec()) % 10 < 2)) {
                        object_detected_ = true;
                        current_ = 0.7;  // 물체 감지 시 전류 증가
                        ROS_INFO("Object detected");
                    }
                }
            } else {
                // 목표 위치에 도달
                position_ = target_position_;
                moving_ = false;
            }
            
            // 전류값 시뮬레이션
            if (moving_) {
                // 이동 중 전류 (0.3 ~ 0.5)
                current_ = 0.3 + 0.2 * speed_;
            } else if (object_detected_) {
                // 물체 감지 시 전류 (0.6 ~ 0.8)
                current_ = 0.6 + 0.2 * force_;
            } else {
                // 정지 상태 전류 (0.1)
                current_ = 0.1;
            }
        }
    }
    
    // 상태 발행 함수
    void publishStatus() {
        hande_bringup::HandEGripperStatus status_msg;
        
        // 상태 메시지 설정
        status_msg.gACT = activated_;
        status_msg.gGTO = moving_;
        status_msg.gSTA = activated_;
        
        // 물체 감지 상태 설정
        if (!activated_) {
            status_msg.gOBJ = 0;
        } else if (object_detected_) {
            status_msg.gOBJ = 1;
        } else if (position_ >= 0.99) {
            status_msg.gOBJ = 3;  // 최대 닫힘 위치 도달
        } else if (position_ <= 0.01) {
            status_msg.gOBJ = 2;  // 최대 열림 위치 도달
        } else {
            status_msg.gOBJ = 0;  // 이동 중
        }
        
        status_msg.gFLT = 0;  // 결함 없음
        status_msg.gPR = static_cast<uint8_t>(target_position_ * 255);
        status_msg.gPO = static_cast<uint8_t>(position_ * 255);
        status_msg.gCU = static_cast<uint8_t>(current_ * 255);
        
        // 상태 발행
        status_pub_.publish(status_msg);
        
        // 조인트 상태 발행 (시각화를 위해)
        std_msgs::Float64 joint_msg;
        joint_msg.data = position_ * 0.025;  // 최대 열림 폭 25mm
        joint_state_pub_.publish(joint_msg);
    }
    
    // 메인 루프 함수
    void run() {
        ros::Rate rate(10);  // 10Hz
        
        while (ros::ok()) {
            updatePosition();
            publishStatus();
            ros::spinOnce();
            rate.sleep();
        }
    }
    
private:
    // 그리퍼 상태 변수
    double position_;        // 현재 위치 (0.0 ~ 1.0)
    double target_position_; // 목표 위치 (0.0 ~ 1.0)
    double speed_;           // 속도 (0.0 ~ 1.0)
    double force_;           // 힘 (0.0 ~ 1.0)
    bool activated_;         // 활성화 상태
    bool moving_;            // 이동 중 상태
    bool object_detected_;   // 물체 감지 상태
    double current_;         // 현재 전류 값 (0.0 ~ 1.0)
    
    // ROS 관련 객체
    ros::Publisher status_pub_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber cmd_sub_;
    ros::ServiceServer control_service_;
    ros::Time last_update_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hande_gripper_driver_cpp");
    
    HandEGripperDriver driver;
    driver.run();
    
    return 0;
}