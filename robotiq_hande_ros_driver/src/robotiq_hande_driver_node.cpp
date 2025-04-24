#include "robotiq_hande_driver.h"
#include <ros/ros.h>
#include <signal.h>

// Global driver instance for signal handler
RobotiqHandEDriver* g_driver = nullptr;

// Signal handler for clean shutdown
void signalHandler(int sig) {
    ROS_INFO("Shutting down...");
    if (g_driver) {
        g_driver->shutdown();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotiq_hande_driver_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Register signal handler
    signal(SIGINT, signalHandler);
    
    // Get parameters
    std::string port_name;
    pnh.param<std::string>("port", port_name, "/tmp/ttyUR");
    
    // Create driver instance
    RobotiqHandEDriver driver(nh);
    g_driver = &driver;
    
    // Initialize driver
    if (!driver.init(port_name)) {
        ROS_ERROR("Failed to initialize Hand-E driver");
        return 1;
    }
    
    // Run driver
    driver.run();
    
    // Cleanup
    driver.shutdown();
    
    return 0;
}