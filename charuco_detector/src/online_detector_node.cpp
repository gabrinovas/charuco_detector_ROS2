#include "rclcpp/rclcpp.hpp"
#include "charuco_detector/online_detector.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ONLINE ChArUco detector (streaming mode)");
    
    auto detector = std::make_shared<ChArUcoOnlineDetector>();
    detector->setupConfigurationFromParameterServer();
    detector->startDetection();
    
    rclcpp::spin(detector);
    rclcpp::shutdown();
    
    return 0;
}
