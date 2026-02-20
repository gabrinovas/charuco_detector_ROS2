#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "charuco_detector/offline_detector.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto detector = std::make_shared<OfflineChArUcoDetector>();
    
    std::string workspace_path = "/home/drims/static/drims2_ws/";
    std::string config_path = workspace_path + "install/charuco_detector/share/charuco_detector/yaml/charuco.yaml";
    std::string calib_path = workspace_path + "install/charuco_detector/share/charuco_detector/config/camera_calibration/camera_calibration.ini";
    std::string images_path = workspace_path + "install/charuco_detector/share/charuco_detector/config/camera_calibration/pic/";
    
    if (!detector->loadConfiguration(config_path)) {
        RCLCPP_ERROR(detector->get_logger(), "Failed to load configuration");
        return -1;
    }
    
    if (!detector->loadCameraCalibration(calib_path)) {
        RCLCPP_ERROR(detector->get_logger(), "Failed to load camera calibration");
        return -1;
    }
    
    detector->processFolder(images_path);
    
    rclcpp::shutdown();
    return 0;
}
