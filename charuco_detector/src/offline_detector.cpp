#include "charuco_detector/offline_detector.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>

OfflineChArUcoDetector::OfflineChArUcoDetector() 
    : Node("offline_charuco_detector")
    , squares_sides_size_m_(0.0200)
    , markers_sides_size_m_(0.0150)
    , number_of_squares_in_x_(10)
    , number_of_squares_in_y_(14)
    , dictionary_id_(3)
    , valid_detections_(0)
    , total_images_(0)
    , use_clahe_(false)
    , use_median_blur_(false)
    , median_blur_k_size_(3) {
    
    detector_params_ = cv::aruco::DetectorParameters::create();
    RCLCPP_INFO(this->get_logger(), "OfflineChArUcoDetector initialized");
}

OfflineChArUcoDetector::~OfflineChArUcoDetector() {}

bool OfflineChArUcoDetector::loadConfiguration(const std::string& config_file) {
    RCLCPP_INFO(this->get_logger(), "Loading configuration from: %s", config_file.c_str());
    
    try {
        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open configuration file");
            return false;
        }
        
        fs["squaresSidesSizeM"] >> squares_sides_size_m_;
        fs["markersSidesSizeM"] >> markers_sides_size_m_;
        fs["numberOfSquaresInX"] >> number_of_squares_in_x_;
        fs["numberOfSquaresInY"] >> number_of_squares_in_y_;
        fs["dictionaryId"] >> dictionary_id_;
        
        fs["adaptiveThreshWinSizeMin"] >> detector_params_->adaptiveThreshWinSizeMin;
        fs["adaptiveThreshWinSizeMax"] >> detector_params_->adaptiveThreshWinSizeMax;
        fs["adaptiveThreshWinSizeStep"] >> detector_params_->adaptiveThreshWinSizeStep;
        fs["adaptiveThreshConstant"] >> detector_params_->adaptiveThreshConstant;
        fs["minMarkerPerimeterRate"] >> detector_params_->minMarkerPerimeterRate;
        fs["maxMarkerPerimeterRate"] >> detector_params_->maxMarkerPerimeterRate;
        fs["cornerRefinementWinSize"] >> detector_params_->cornerRefinementWinSize;
        
        fs.release();
        
        if (dictionary_id_ > 0) {
            dictionary_ = cv::aruco::getPredefinedDictionary(
                cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id_));
        }
        
        board_ = cv::aruco::CharucoBoard::create(
            number_of_squares_in_x_, 
            number_of_squares_in_y_,
            static_cast<float>(squares_sides_size_m_), 
            static_cast<float>(markers_sides_size_m_), 
            dictionary_
        );
        
        RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
        RCLCPP_INFO(this->get_logger(), "Board: %dx%d squares, size: %.4fm", 
                    number_of_squares_in_x_, number_of_squares_in_y_, squares_sides_size_m_);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s", e.what());
        return false;
    }
}

bool OfflineChArUcoDetector::loadCameraCalibration(const std::string& calib_file) {
    RCLCPP_INFO(this->get_logger(), "Loading camera calibration from: %s", calib_file.c_str());
    
    try {
        cv::FileStorage fs(calib_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open calibration file");
            return false;
        }
        
        fs["camera_matrix"] >> camera_matrix_;
        fs["distortion_coefficients"] >> dist_coeffs_;
        fs.release();
        
        if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Camera matrices are empty");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera calibration loaded successfully");
        RCLCPP_INFO(this->get_logger(), "  fx: %.2f, fy: %.2f", 
                    camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(1,1));
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading calibration: %s", e.what());
        return false;
    }
}

std::vector<std::string> OfflineChArUcoDetector::findImagesInFolder(const std::string& folder_path) {
    std::vector<std::string> image_files;
    
    RCLCPP_INFO(this->get_logger(), "Searching for images in: %s", folder_path.c_str());
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
                image_files.push_back(entry.path().string());
            }
        }
        
        std::sort(image_files.begin(), image_files.end());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading directory: %s", e.what());
    }
    
    RCLCPP_INFO(this->get_logger(), "Found %zu images", image_files.size());
    total_images_ = image_files.size();
    return image_files;
}

bool OfflineChArUcoDetector::detectBoardInImage(const std::string& image_path, 
                                                cv::Vec3d& rvec, 
                                                cv::Vec3d& tvec,
                                                cv::Mat& output_image) {
    
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Could not read: %s", image_path.c_str());
        return false;
    }
    
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    if (use_median_blur_) {
        cv::medianBlur(gray, gray, median_blur_k_size_);
    }
    
    if (use_clahe_) {
        auto clahe = cv::createCLAHE(2.0, cv::Size(8,8));
        clahe->apply(gray, gray);
    }
    
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_markers;
    
    cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, 
                             detector_params_, rejected_markers);
    
    if (marker_ids.empty()) {
        return false;
    }
    
    cv::aruco::refineDetectedMarkers(gray, board_, marker_corners, marker_ids, 
                                     rejected_markers);
    
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    
    int interpolated = cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board_, 
        charuco_corners, charuco_ids, camera_matrix_, dist_coeffs_);
    
    if (interpolated <= 0) {
        return false;
    }
    
    bool valid_pose = cv::aruco::estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board_, 
        camera_matrix_, dist_coeffs_, rvec, tvec);
    
    if (!valid_pose) {
        return false;
    }
    
    image.copyTo(output_image);
    cv::aruco::drawDetectedMarkers(output_image, marker_corners);
    cv::aruco::drawDetectedCornersCharuco(output_image, charuco_corners, charuco_ids);
    
    float axis_length = 0.5f * static_cast<float>(
        std::min(number_of_squares_in_x_, number_of_squares_in_y_) * squares_sides_size_m_);
    cv::aruco::drawAxis(output_image, camera_matrix_, dist_coeffs_, rvec, tvec, axis_length);
    
    return true;
}

void OfflineChArUcoDetector::processFolder(const std::string& folder_path) {
    
    valid_detections_ = 0;
    std::vector<std::string> images = findImagesInFolder(folder_path);
    
    RCLCPP_INFO(this->get_logger(), "Processing %zu images...", images.size());
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    std::string output_folder = folder_path + "/detected/";
    std::filesystem::create_directories(output_folder);
    
    for (size_t i = 0; i < images.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "[%zu/%zu] %s", 
                    i+1, images.size(), 
                    std::filesystem::path(images[i]).filename().string().c_str());
        
        cv::Vec3d rvec, tvec;
        cv::Mat output_image;
        
        if (detectBoardInImage(images[i], rvec, tvec, output_image)) {
            valid_detections_++;
            
            RCLCPP_INFO(this->get_logger(), "  ✅ DETECTED");
            RCLCPP_INFO(this->get_logger(), "     Position: [%.3f, %.3f, %.3f] m", 
                        tvec[0], tvec[1], tvec[2]);
            
            std::string filename = std::filesystem::path(images[i]).stem().string();
            std::string output_path = output_folder + filename + "_detected.jpg";
            cv::imwrite(output_path, output_image);
            RCLCPP_INFO(this->get_logger(), "     💾 Saved: %s", output_path.c_str());
            
        } else {
            RCLCPP_WARN(this->get_logger(), "  ❌ NOT DETECTED");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "📊 SUMMARY:");
    RCLCPP_INFO(this->get_logger(), "   Total images: %d", total_images_);
    RCLCPP_INFO(this->get_logger(), "   Valid detections: %d", valid_detections_);
    RCLCPP_INFO(this->get_logger(), "   Success rate: %.1f%%", 
                (valid_detections_ * 100.0 / total_images_));
    RCLCPP_INFO(this->get_logger(), "   Results saved in: %s", output_folder.c_str());
}
