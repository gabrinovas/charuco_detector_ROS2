#ifndef OFFLINE_DETECTOR_HPP_
#define OFFLINE_DETECTOR_HPP_

#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

class OfflineChArUcoDetector {
public:
    OfflineChArUcoDetector();
    ~OfflineChArUcoDetector();

    bool loadConfiguration(const std::string& config_file);
    bool loadCameraCalibration(const std::string& calib_file);
    std::vector<std::string> findImagesInFolder(const std::string& folder_path);
    bool detectBoardInImage(const std::string& image_path, 
                           cv::Vec3d& rvec, 
                           cv::Vec3d& tvec,
                           cv::Mat& output_image);
    void processFolder(const std::string& folder_path);
    
    // Getters
    int getValidDetections() const { return valid_detections_; }
    int getTotalImages() const { return total_images_; }

private:
    // ChArUco parameters
    double squares_sides_size_m_;
    double markers_sides_size_m_;
    int number_of_squares_in_x_;
    int number_of_squares_in_y_;
    int dictionary_id_;
    
    // Detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> board_;
    
    // Camera calibration
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // Statistics
    int valid_detections_;
    int total_images_;
    
    // Image processing flags
    bool use_clahe_;
    bool use_median_blur_;
    int median_blur_k_size_;
};

#endif