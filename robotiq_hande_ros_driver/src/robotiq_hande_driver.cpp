#include "robotiq_hande_driver.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

RobotiqHandEDriver::RobotiqHandEDriver(ros::NodeHandle& nh) : nh_(nh), serial_fd_(-1), is_connected_(false) {
    // Initialize default parameters
    gripper_speed_ = 255; // Default to maximum speed
    gripper_force_ = 255; // Default to maximum force
    
    // Setup ROS communication
    status_pub_ = nh_.advertise<robotiq_hande_ros_driver::HandEStatus>("robotiq_hande/status", 10);
    position_pub_ = nh_.advertise<std_msgs::Float32>("robotiq_hande/position", 10);
    command_sub_ = nh_.subscribe("robotiq_hande/command", 10, &RobotiqHandEDriver::commandCallback, this);
    
    // Setup services
    grip_service_ = nh_.advertiseService("robotiq_hande/grip", &RobotiqHandEDriver::gripCallback, this);
    release_service_ = nh_.advertiseService("robotiq_hande/release", &RobotiqHandEDriver::releaseCallback, this);
}

RobotiqHandEDriver::~RobotiqHandEDriver() {
    shutdown();
}

bool RobotiqHandEDriver::init(const std::string& port_name) {
    port_name_ = port_name;
    
    ROS_INFO("Initializing Robotiq Hand-E gripper on port %s", port_name_.c_str());
    
    // Open serial port
    if (!openSerialPort()) {
        ROS_ERROR("Failed to open serial port: %s", port_name_.c_str());
        return false;
    }
    
    // Add a small delay after opening the port
    ros::Duration(1.0).sleep();
    ROS_INFO("Serial port opened, attempting to activate gripper...");
    
    // Try activation up to 3 times
    bool activated = false;
    for (int i = 0; i < 3; i++) {
        ROS_INFO("Activation attempt %d/3", i+1);
        if (activateGripper()) {
            activated = true;
            break;
        }
        ros::Duration(1.0).sleep();
    }
    
    if (!activated) {
        ROS_ERROR("Failed to activate gripper after multiple attempts");
        closeSerialPort();
        return false;
    }
    
    ROS_INFO("Robotiq Hand-E gripper initialized successfully");
    return true;
}

void RobotiqHandEDriver::run() {
    ros::Rate rate(10); // 10Hz
    
    while (ros::ok() && is_connected_) {
        // Read gripper status
        if (readGripperStatus()) {
            // Publish status message
            status_pub_.publish(status_msg_);
            
            // Publish position in mm
            std_msgs::Float32 pos_msg;
            float width_mm;
            if (convertPositionToWidth(status_msg_.position, width_mm)) {
                pos_msg.data = width_mm;
                position_pub_.publish(pos_msg);
            }
        }
        
        rate.sleep();
    }
}

void RobotiqHandEDriver::shutdown() {
    if (is_connected_) {
        deactivateGripper();
        closeSerialPort();
    }
}

bool RobotiqHandEDriver::openSerialPort() {
    // Open serial port (removing O_NDELAY flag to make read operations blocking)
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    
    if (serial_fd_ < 0) {
        ROS_ERROR("Failed to open serial port: %s", strerror(errno));
        return false;
    }
    
    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        ROS_ERROR("Error from tcgetattr: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // Set baud rate (115200)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // 8N1 (8 bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    
    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output
    tty.c_oflag &= ~OPOST;
    
    // No software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // No special handling of bytes
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // Set timeout - increased from 10 to 30 (3 seconds)
    tty.c_cc[VMIN] = 0;  // Min number of characters to read
    tty.c_cc[VTIME] = 30; // Timeout in deciseconds
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // Flush port to clear any pending data
    tcflush(serial_fd_, TCIOFLUSH);
    
    is_connected_ = true;
    ROS_INFO("Serial port %s opened successfully", port_name_.c_str());
    return true;
}

void RobotiqHandEDriver::closeSerialPort() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        is_connected_ = false;
        ROS_INFO("Serial port closed");
    }
}

std::vector<uint8_t> RobotiqHandEDriver::buildModbusRequest(uint8_t function_code, uint16_t start_addr, uint16_t num_registers, const std::vector<uint16_t>& data) {
    std::vector<uint8_t> request;
    
    // Slave ID
    request.push_back(SLAVE_ID);
    
    // Function code
    request.push_back(function_code);
    
    // Start address (big-endian)
    request.push_back((start_addr >> 8) & 0xFF);
    request.push_back(start_addr & 0xFF);
    
    if (function_code == READ_REGISTERS) {
        // Number of registers to read (big-endian)
        request.push_back((num_registers >> 8) & 0xFF);
        request.push_back(num_registers & 0xFF);
    } else if (function_code == WRITE_REGISTER) {
        // Register value (big-endian)
        request.push_back((data[0] >> 8) & 0xFF);
        request.push_back(data[0] & 0xFF);
    } else if (function_code == WRITE_MULTIPLE_REGISTERS) {
        // Number of registers to write (big-endian)
        request.push_back((num_registers >> 8) & 0xFF);
        request.push_back(num_registers & 0xFF);
        
        // Byte count
        request.push_back(num_registers * 2);
        
        // Register values (big-endian)
        for (uint16_t value : data) {
            request.push_back((value >> 8) & 0xFF);
            request.push_back(value & 0xFF);
        }
    }
    
    // Calculate CRC
    uint16_t crc = calculateCRC(request);
    
    // Append CRC (little-endian)
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
    
    return request;
}

uint16_t RobotiqHandEDriver::calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t byte : data) {
        crc ^= byte;
        
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

bool RobotiqHandEDriver::sendModbusRequest(const std::vector<uint8_t>& request, std::vector<uint8_t>& response) {
    if (!is_connected_) {
        ROS_ERROR("Cannot send Modbus request: not connected");
        return false;
    }
    
    // Print request for debugging
    std::stringstream req_ss;
    for (uint8_t byte : request) {
        req_ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    ROS_DEBUG("Sending Modbus request: %s", req_ss.str().c_str());
    
    // Send request
    ssize_t bytes_written = write(serial_fd_, request.data(), request.size());
    
    if (bytes_written != static_cast<ssize_t>(request.size())) {
        ROS_ERROR("Failed to write complete request: %zd of %zu bytes written", bytes_written, request.size());
        return false;
    }
    
    // Wait for response (increased from 50ms to 100ms)
    usleep(100000); // 100ms
    
    // Read response with retry
    uint8_t buffer[256];
    ssize_t bytes_read = 0;
    int retry_count = 0;
    const int max_retries = 3;
    
    while (retry_count < max_retries) {
        bytes_read = read(serial_fd_, buffer, sizeof(buffer));
        
        if (bytes_read > 0) {
            break;  // Successfully read data
        } else if (bytes_read == 0) {
            ROS_WARN("No data available, retrying... (%d/%d)", retry_count + 1, max_retries);
        } else {
            ROS_WARN("Failed to read response: %s (attempt %d/%d)", strerror(errno), retry_count + 1, max_retries);
        }
        
        retry_count++;
        usleep(100000); // 100ms between retries
    }
    
    if (bytes_read <= 0) {
        ROS_ERROR("Failed to read response after %d attempts", max_retries);
        return false;
    }
    
    // Copy response
    response.assign(buffer, buffer + bytes_read);
    
    // Print response for debugging
    std::stringstream resp_ss;
    for (uint8_t byte : response) {
        resp_ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    ROS_DEBUG("Received response (%zd bytes): %s", bytes_read, resp_ss.str().c_str());
    
    // Verify response
    if (response.size() < 5) {
        ROS_ERROR("Response too short: %zu bytes", response.size());
        return false;
    }
    
    // Check slave ID
    if (response[0] != SLAVE_ID) {
        ROS_ERROR("Invalid slave ID in response: %02X (expected %02X)", response[0], SLAVE_ID);
        return false;
    }
    
    // Check function code
    if (response[1] != request[1]) {
        // Check if error response
        if (response[1] == (request[1] | 0x80)) {
            ROS_ERROR("Modbus error response: function code %02X, exception code %02X", request[1], response[2]);
        } else {
            ROS_ERROR("Invalid function code in response: %02X (expected %02X)", response[1], request[1]);
        }
        return false;
    }
    
    // Check CRC
    if (response.size() >= 4) {
        uint16_t crc_received = (response[response.size() - 1] << 8) | response[response.size() - 2];
        std::vector<uint8_t> crc_data(response.begin(), response.end() - 2);
        uint16_t crc_calculated = calculateCRC(crc_data);
        
        if (crc_received != crc_calculated) {
            ROS_ERROR("CRC error: received %04X, calculated %04X", crc_received, crc_calculated);
            return false;
        }
    }
    
    return true;
}

bool RobotiqHandEDriver::activateGripper() {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    
    // Reset gripper by sending activation command
    std::vector<uint16_t> data = {0x0100}; // Reset + Activate gripper
    auto request = buildModbusRequest(WRITE_REGISTER, REG_ACTION_REQUEST, 1, data);
    std::vector<uint8_t> response;
    
    if (!sendModbusRequest(request, response)) {
        ROS_ERROR("Failed to send activation command");
        return false;
    }
    
    // Wait for gripper to activate
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(5.0)) {
        if (readGripperStatus() && (status_msg_.status & GRIPPER_ACTIVATED)) {
            ROS_INFO("Gripper activated successfully");
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    
    ROS_ERROR("Timeout waiting for gripper activation");
    return false;
}

bool RobotiqHandEDriver::deactivateGripper() {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    
    std::vector<uint16_t> data = {0x0000}; // Deactivate gripper
    auto request = buildModbusRequest(WRITE_REGISTER, REG_ACTION_REQUEST, 1, data);
    std::vector<uint8_t> response;
    
    if (!sendModbusRequest(request, response)) {
        ROS_ERROR("Failed to send deactivation command");
        return false;
    }
    
    return true;
}

bool RobotiqHandEDriver::readGripperStatus() {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    
    auto request = buildModbusRequest(READ_REGISTERS, REG_GRIPPER_STATUS, 3); // Read 3 registers: status, fault, position
    std::vector<uint8_t> response;
    
    if (!sendModbusRequest(request, response)) {
        ROS_ERROR("Failed to read gripper status");
        return false;
    }
    
    if (response.size() < 9) {
        ROS_ERROR("Invalid response size for status read: %zu bytes", response.size());
        return false;
    }
    
    // Parse status register
    uint16_t status_reg = (response[3] << 8) | response[4];
    status_msg_.status = status_reg & 0xFF;
    status_msg_.is_activated = (status_msg_.status & GRIPPER_ACTIVATED) != 0;
    status_msg_.is_moving = (status_msg_.status & GRIPPER_AT_REQUESTED_POS) == 0;
    status_msg_.object_detected = (status_msg_.status & OBJECT_DETECTED) != 0;
    
    // Parse fault register
    uint16_t fault_reg = (response[5] << 8) | response[6];
    status_msg_.fault = fault_reg & 0xFF;
    
    // Parse position register
    uint16_t pos_reg = (response[7] << 8) | response[8];
    status_msg_.position = pos_reg & 0xFF;
    
    return true;
}

bool RobotiqHandEDriver::sendGripperCommand(uint8_t action_request, uint8_t position) {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    
    // Write 4 registers: action, position, speed, force
    std::vector<uint16_t> data = {
        static_cast<uint16_t>(action_request),
        static_cast<uint16_t>(position),
        static_cast<uint16_t>(gripper_speed_),
        static_cast<uint16_t>(gripper_force_)
    };
    
    auto request = buildModbusRequest(WRITE_MULTIPLE_REGISTERS, REG_ACTION_REQUEST, 4, data);
    std::vector<uint8_t> response;
    
    if (!sendModbusRequest(request, response)) {
        ROS_ERROR("Failed to send gripper command");
        return false;
    }
    
    return true;
}

bool RobotiqHandEDriver::setGripperPosition(uint8_t position) {
    return sendGripperCommand(ACTION_GOTO_POS, position);
}

void RobotiqHandEDriver::commandCallback(const robotiq_hande_ros_driver::HandECommand::ConstPtr& msg) {
    // Update speed and force if specified
    if (msg->speed > 0) {
        gripper_speed_ = msg->speed;
    }
    
    if (msg->force > 0) {
        gripper_force_ = msg->force;
    }
    
    // Execute command
    if (msg->command == "close") {
        sendGripperCommand(ACTION_CLOSE, 0);
    } else if (msg->command == "open") {
        sendGripperCommand(ACTION_OPEN, 0);
    } else if (msg->command == "goto") {
        uint8_t position = convertWidthToPosition(msg->position);
        setGripperPosition(position);
    } else if (msg->command == "stop") {
        sendGripperCommand(ACTION_STOP, 0);
    } else {
        ROS_WARN("Unknown command: %s", msg->command.c_str());
    }
}

bool RobotiqHandEDriver::gripCallback(robotiq_hande_ros_driver::HandEGrip::Request& req, robotiq_hande_ros_driver::HandEGrip::Response& res) {
    // Set speed and force if specified
    if (req.speed > 0) {
        gripper_speed_ = req.speed;
    }
    
    if (req.force > 0) {
        gripper_force_ = req.force;
    }
    
    // Close gripper
    bool success = sendGripperCommand(ACTION_CLOSE, 0);
    res.success = success;
    
    return true;
}

bool RobotiqHandEDriver::releaseCallback(robotiq_hande_ros_driver::HandERelease::Request& req, robotiq_hande_ros_driver::HandERelease::Response& res) {
    // Set speed if specified
    if (req.speed > 0) {
        gripper_speed_ = req.speed;
    }
    
    // Open gripper
    bool success = sendGripperCommand(ACTION_OPEN, 0);
    res.success = success;
    
    return true;
}

bool RobotiqHandEDriver::convertPositionToWidth(uint8_t position, float& width_mm) {
    // Hand-E gripper has range of 0-85mm
    // Position 0 is fully closed (~5mm opening)
    // Position 255 is fully open (~85mm opening)
    width_mm = 5.0f + (position / 255.0f) * 80.0f;
    return true;
}

uint8_t RobotiqHandEDriver::convertWidthToPosition(float width_mm) {
    // Clamp width between 5mm and 85mm
    width_mm = std::max(5.0f, std::min(85.0f, width_mm));
    
    // Convert to position
    uint8_t position = static_cast<uint8_t>((width_mm - 5.0f) / 80.0f * 255.0f);
    return position;
}