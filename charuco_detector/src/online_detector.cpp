/**\file online_detector.cpp
 * \brief Online detector of ChArUco patterns (streaming mode)
 */

#include "charuco_detector/online_detector.hpp"

ChArUcoOnlineDetector::ChArUcoOnlineDetector() 
    : Node("charuco_detector")
    , camera_info_received_(false)
    , transform_stamped_valid_(false)
{
    RCLCPP_INFO(this->get_logger(), "[ChArUcoOnlineDetector] Node started.");
    
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Declare parameters
    this->declare_parameter("charuco_parameters.adaptiveThreshWinSizeMin", 3);
    this->declare_parameter("charuco_parameters.adaptiveThreshWinSizeMax", 23);
    this->declare_parameter("charuco_parameters.adaptiveThreshWinSizeStep", 10);
    this->declare_parameter("charuco_parameters.adaptiveThreshConstant", 7.0);
    this->declare_parameter("charuco_parameters.minMarkerPerimeterRate", 0.03);
    this->declare_parameter("charuco_parameters.maxMarkerPerimeterRate", 4.0);
    this->declare_parameter("charuco_parameters.polygonalApproxAccuracyRate", 0.03);
    this->declare_parameter("charuco_parameters.minCornerDistanceRate", 0.05);
    this->declare_parameter("charuco_parameters.minDistanceToBorder", 3);
    this->declare_parameter("charuco_parameters.minMarkerDistanceRate", 0.05);
    this->declare_parameter("charuco_parameters.cornerRefinementWinSize", 5);
    this->declare_parameter("charuco_parameters.cornerRefinementMaxIterations", 30);
    this->declare_parameter("charuco_parameters.cornerRefinementMinAccuracy", 0.1);
    this->declare_parameter("charuco_parameters.markerBorderBits", 1);
    this->declare_parameter("charuco_parameters.perspectiveRemovePixelPerCell", 4);
    this->declare_parameter("charuco_parameters.perspectiveRemoveIgnoredMarginPerCell", 0.13);
    this->declare_parameter("charuco_parameters.maxErroneousBitsInBorderRate", 0.35);
    this->declare_parameter("charuco_parameters.minOtsuStdDev", 5.0);
    this->declare_parameter("charuco_parameters.errorCorrectionRate", 0.6);

    this->declare_parameter("charuco_parameters.squaresSidesSizeM", 0.0200);
    this->declare_parameter("charuco_parameters.markersSidesSizeM", 0.0150);
    this->declare_parameter("charuco_parameters.numberOfBitsForMarkersSides", 4);
    this->declare_parameter("charuco_parameters.numberOfMarkers", 70);
    this->declare_parameter("charuco_parameters.numberOfSquaresInX", 10);
    this->declare_parameter("charuco_parameters.numberOfSquaresInY", 14);
    this->declare_parameter("charuco_parameters.dictionaryId", 3);

    this->declare_parameter("function_parameters.use_median_blur", false);
    this->declare_parameter("function_parameters.median_blur_k_size", 3);
    this->declare_parameter("function_parameters.use_dynamic_range", false);
    this->declare_parameter("function_parameters.use_bilateral_filter", false);
    this->declare_parameter("function_parameters.bilateral_filter_pixel_neighborhood", 5);
    this->declare_parameter("function_parameters.bilateral_filter_sigma_color", 100.0);
    this->declare_parameter("function_parameters.bilateral_filter_sigma_space", 100.0);
    this->declare_parameter("function_parameters.bilateral_filter_border_type", 4);
    this->declare_parameter("function_parameters.use_clahe", false);
    this->declare_parameter("function_parameters.clahe_clip_limit", 2.0);
    this->declare_parameter("function_parameters.clahe_size_x", 8);
    this->declare_parameter("function_parameters.clahe_size_y", 8);
    this->declare_parameter("function_parameters.use_adaptive_threshold", false);
    this->declare_parameter("function_parameters.adaptive_threshold_max_value", 255.0);
    this->declare_parameter("function_parameters.adaptive_threshold_method", 1);
    this->declare_parameter("function_parameters.adaptive_threshold_type", 0);
    this->declare_parameter("function_parameters.adaptive_threshold_block_size", 65);
    this->declare_parameter("function_parameters.adaptive_threshold_constant_offset_from_mean", 0.0);

    this->declare_parameter("tf_parameters.use_static_tf_broadcaster", false);
    this->declare_parameter("tf_parameters.tf_broadcaster_republish_rate", 10.0);
    this->declare_parameter("tf_parameters.sensor_frame_override", "");
    this->declare_parameter("tf_parameters.charuco_tf_frame", "charuco");

    this->declare_parameter("subscribers.image_topic.topic", "/camera/color/image_raw");
    this->declare_parameter("subscribers.camera_info.topic", "/camera/color/camera_info");
    this->declare_parameter("publishers.image_results_publisher.queue_size", 1);
    this->declare_parameter("publishers.image_results_publisher.latch", false);
}

ChArUcoOnlineDetector::~ChArUcoOnlineDetector() {}

void ChArUcoOnlineDetector::setupConfigurationFromParameterServer() {
    RCLCPP_INFO(this->get_logger(), "Loading configuration from parameter server...");
    
    detector_parameters_ = cv::aruco::DetectorParameters::create();
    
    this->get_parameter("charuco_parameters.adaptiveThreshWinSizeMin", detector_parameters_->adaptiveThreshWinSizeMin);
    this->get_parameter("charuco_parameters.adaptiveThreshWinSizeMax", detector_parameters_->adaptiveThreshWinSizeMax);
    this->get_parameter("charuco_parameters.adaptiveThreshWinSizeStep", detector_parameters_->adaptiveThreshWinSizeStep);
    this->get_parameter("charuco_parameters.adaptiveThreshConstant", detector_parameters_->adaptiveThreshConstant);
    this->get_parameter("charuco_parameters.minMarkerPerimeterRate", detector_parameters_->minMarkerPerimeterRate);
    this->get_parameter("charuco_parameters.maxMarkerPerimeterRate", detector_parameters_->maxMarkerPerimeterRate);
    this->get_parameter("charuco_parameters.polygonalApproxAccuracyRate", detector_parameters_->polygonalApproxAccuracyRate);
    this->get_parameter("charuco_parameters.minCornerDistanceRate", detector_parameters_->minCornerDistanceRate);
    this->get_parameter("charuco_parameters.minDistanceToBorder", detector_parameters_->minDistanceToBorder);
    this->get_parameter("charuco_parameters.minMarkerDistanceRate", detector_parameters_->minMarkerDistanceRate);
    this->get_parameter("charuco_parameters.cornerRefinementWinSize", detector_parameters_->cornerRefinementWinSize);
    this->get_parameter("charuco_parameters.cornerRefinementMaxIterations", detector_parameters_->cornerRefinementMaxIterations);
    this->get_parameter("charuco_parameters.cornerRefinementMinAccuracy", detector_parameters_->cornerRefinementMinAccuracy);
    this->get_parameter("charuco_parameters.markerBorderBits", detector_parameters_->markerBorderBits);
    this->get_parameter("charuco_parameters.perspectiveRemovePixelPerCell", detector_parameters_->perspectiveRemovePixelPerCell);
    this->get_parameter("charuco_parameters.perspectiveRemoveIgnoredMarginPerCell", detector_parameters_->perspectiveRemoveIgnoredMarginPerCell);
    this->get_parameter("charuco_parameters.maxErroneousBitsInBorderRate", detector_parameters_->maxErroneousBitsInBorderRate);
    this->get_parameter("charuco_parameters.minOtsuStdDev", detector_parameters_->minOtsuStdDev);
    this->get_parameter("charuco_parameters.errorCorrectionRate", detector_parameters_->errorCorrectionRate);

    this->get_parameter("charuco_parameters.squaresSidesSizeM", squares_sides_size_m_);
    this->get_parameter("charuco_parameters.markersSidesSizeM", markers_sides_size_m_);
    this->get_parameter("charuco_parameters.numberOfBitsForMarkersSides", number_of_bits_for_markers_sides_);
    this->get_parameter("charuco_parameters.numberOfMarkers", number_of_markers_);
    this->get_parameter("charuco_parameters.numberOfSquaresInX", number_of_squares_in_x_);
    this->get_parameter("charuco_parameters.numberOfSquaresInY", number_of_squares_in_y_);
    this->get_parameter("charuco_parameters.dictionaryId", dictionary_id_);

    this->get_parameter("function_parameters.use_median_blur", use_median_blur_);
    this->get_parameter("function_parameters.median_blur_k_size", median_blur_k_size_);
    this->get_parameter("function_parameters.use_dynamic_range", use_dynamic_range_);
    this->get_parameter("function_parameters.use_bilateral_filter", use_bilateral_filter_);
    this->get_parameter("function_parameters.bilateral_filter_pixel_neighborhood", bilateral_filter_pixel_neighborhood_);
    this->get_parameter("function_parameters.bilateral_filter_sigma_color", bilateral_filter_sigma_color_);
    this->get_parameter("function_parameters.bilateral_filter_sigma_space", bilateral_filter_sigma_space_);
    this->get_parameter("function_parameters.bilateral_filter_border_type", bilateral_filter_border_type_);
    this->get_parameter("function_parameters.use_clahe", use_clahe_);
    this->get_parameter("function_parameters.clahe_clip_limit", clahe_clip_limit_);
    this->get_parameter("function_parameters.clahe_size_x", clahe_size_x_);
    this->get_parameter("function_parameters.clahe_size_y", clahe_size_y_);
    this->get_parameter("function_parameters.use_adaptive_threshold", use_adaptive_threshold_);
    this->get_parameter("function_parameters.adaptive_threshold_max_value", adaptive_threshold_max_value_);
    this->get_parameter("function_parameters.adaptive_threshold_method", adaptive_threshold_method_);
    this->get_parameter("function_parameters.adaptive_threshold_type", adaptive_threshold_type_);
    this->get_parameter("function_parameters.adaptive_threshold_block_size", adaptive_threshold_block_size_);
    this->get_parameter("function_parameters.adaptive_threshold_constant_offset_from_mean", adaptive_threshold_constant_offset_from_mean_);

    this->get_parameter("tf_parameters.use_static_tf_broadcaster", use_static_tf_broadcaster_);
    this->get_parameter("tf_parameters.tf_broadcaster_republish_rate", tf_broadcaster_republish_rate_);
    this->get_parameter("tf_parameters.sensor_frame_override", sensor_frame_override_);
    this->get_parameter("tf_parameters.charuco_tf_frame", charuco_tf_frame_);

    this->get_parameter("subscribers.image_topic.topic", image_topic_);
    this->get_parameter("subscribers.camera_info.topic", camera_info_topic_);
    
    image_results_publish_topic_ = image_topic_ + "_charuco_detection";
    charuco_pose_publish_topic_ = image_topic_ + "_charuco_pose";

    if (dictionary_id_ > 0) {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id_));
    } else {
        dictionary_ = cv::aruco::generateCustomDictionary(number_of_markers_, number_of_bits_for_markers_sides_);
    }
    
    board_ = cv::aruco::CharucoBoard::create(
        number_of_squares_in_x_, 
        number_of_squares_in_y_,
        static_cast<float>(squares_sides_size_m_), 
        static_cast<float>(markers_sides_size_m_), 
        dictionary_
    );

    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
}

void ChArUcoOnlineDetector::startDetection() {
    RCLCPP_INFO(this->get_logger(), "Starting ChArUco online detection...");
    
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10, 
        std::bind(&ChArUcoOnlineDetector::imageCallback, this, std::placeholders::_1)
    );
    
    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, 10,
        std::bind(&ChArUcoOnlineDetector::cameraInfoCallback, this, std::placeholders::_1)
    );
    
    image_transport_publisher_ = image_transport_->advertise(image_results_publish_topic_, 1);
    
    charuco_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        charuco_pose_publish_topic_, 10
    );
    
    RCLCPP_INFO(this->get_logger(), "Online detector ready. Subscribed to: %s", image_topic_.c_str());
}

void ChArUcoOnlineDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    bool valid_camera_info = false;
    for (size_t i = 0; i < msg->k.size(); ++i) {
        if (msg->k[i] != 0.0) {
            valid_camera_info = true;
            break;
        }
    }
    
    if (valid_camera_info && !camera_info_received_) {
        camera_intrinsics_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
        camera_distortion_coefficients_matrix_ = cv::Mat::zeros(1, 5, CV_64F);
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camera_intrinsics_matrix_.at<double>(i, j) = msg->k[i * 3 + j];
            }
        }
        
        for (size_t i = 0; i < msg->d.size() && i < 5; ++i) {
            camera_distortion_coefficients_matrix_.at<double>(0, i) = msg->d[i];
        }
        
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded successfully");
    }
}

void ChArUcoOnlineDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Waiting for camera info...");
        return;
    }
    
    if (msg->data.empty() || msg->step == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty image");
        return;
    }
    
    cv::Mat image_grayscale;
    bool dynamic_range_applied = false;
    
    if (msg->encoding == sensor_msgs::image_encodings::MONO16 || use_dynamic_range_) {
        try {
            image_grayscale = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
            if (use_median_blur_) applyMedianBlur(image_grayscale);
            applyDynamicRange(image_grayscale);
            dynamic_range_applied = true;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error converting MONO16: %s", e.what());
        }
    }
    
    if (!dynamic_range_applied) {
        try {
            image_grayscale = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        } catch (const cv_bridge::Exception& e) {
            try {
                auto image = cv_bridge::toCvShare(msg);
                cv_bridge::CvtColorForDisplayOptions options;
                options.do_dynamic_scaling = true;
                options.colormap = -1;
                image_grayscale = cv_bridge::cvtColorForDisplay(image, sensor_msgs::image_encodings::MONO8, options)->image;
            } catch (const cv_bridge::Exception& e2) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e2.what());
                return;
            }
        }
    }
    
    if (!dynamic_range_applied && use_median_blur_) {
        applyMedianBlur(image_grayscale);
    }
    
    if (use_bilateral_filter_) {
        applyBilateralFilter(image_grayscale);
    }
    
    if (use_clahe_) {
        applyCLAHE(image_grayscale);
    }
    
    if (use_adaptive_threshold_) {
        applyAdaptiveThreshold(image_grayscale);
    }
    
    cv::Vec3d camera_rotation, camera_translation;
    cv::Mat image_results;
    
    if (detectChArUcoBoard(image_grayscale, 
                           camera_intrinsics_matrix_, 
                           camera_distortion_coefficients_matrix_,
                           camera_rotation, 
                           camera_translation, 
                           image_results, 
                           true)) {
        
        geometry_msgs::msg::PoseStamped charuco_pose;
        charuco_pose.header = msg->header;
        fillPose(camera_rotation, camera_translation, charuco_pose);
        charuco_pose_publisher_->publish(charuco_pose);
        
        transform_stamped_.header = msg->header;
        if (!sensor_frame_override_.empty()) {
            transform_stamped_.header.frame_id = sensor_frame_override_;
        }
        transform_stamped_.child_frame_id = charuco_tf_frame_;
        transform_stamped_.transform.translation.x = charuco_pose.pose.position.x;
        transform_stamped_.transform.translation.y = charuco_pose.pose.position.y;
        transform_stamped_.transform.translation.z = charuco_pose.pose.position.z;
        transform_stamped_.transform.rotation = charuco_pose.pose.orientation;
        transform_stamped_valid_ = true;
        
        if (use_static_tf_broadcaster_) {
            static_tf_broadcaster_->sendTransform(transform_stamped_);
        } else {
            tf_broadcaster_->sendTransform(transform_stamped_);
        }
        
        auto image_results_msg = cv_bridge::CvImage(msg->header, "bgr8", image_results).toImageMsg();
        image_transport_publisher_.publish(image_results_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "ChArUco board detected successfully");
    } else {
        auto image_filtered_msg = cv_bridge::CvImage(msg->header, "mono8", image_grayscale).toImageMsg();
        image_transport_publisher_.publish(image_filtered_msg);
    }
}

void ChArUcoOnlineDetector::applyMedianBlur(cv::Mat &image_in_out) {
    cv::Mat temp;
    cv::medianBlur(image_in_out, temp, median_blur_k_size_);
    image_in_out = temp;
}

void ChArUcoOnlineDetector::applyDynamicRange(cv::Mat& image_in_out) {
    double min_val = 0, max_val = 65535;
    cv::minMaxLoc(image_in_out, &min_val, &max_val);
    
    if (max_val > min_val) {
        cv::Mat temp = image_in_out - min_val;
        temp.convertTo(image_in_out, CV_8UC1, 255.0 / (max_val - min_val));
    }
}

void ChArUcoOnlineDetector::applyBilateralFilter(cv::Mat &image_in_out) {
    cv::Mat temp;
    cv::bilateralFilter(image_in_out, temp, 
                        bilateral_filter_pixel_neighborhood_, 
                        bilateral_filter_sigma_color_, 
                        bilateral_filter_sigma_space_, 
                        bilateral_filter_border_type_);
    image_in_out = temp;
}

void ChArUcoOnlineDetector::applyCLAHE(cv::Mat &image_in_out) {
    auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_size_x_, clahe_size_y_));
    cv::Mat temp;
    clahe->apply(image_in_out, temp);
    image_in_out = temp;
}

void ChArUcoOnlineDetector::applyAdaptiveThreshold(cv::Mat &image_in_out) {
    cv::Mat temp;
    cv::adaptiveThreshold(image_in_out, temp, 
                          adaptive_threshold_max_value_, 
                          adaptive_threshold_method_, 
                          adaptive_threshold_type_, 
                          adaptive_threshold_block_size_, 
                          adaptive_threshold_constant_offset_from_mean_);
    image_in_out = temp;
}

bool ChArUcoOnlineDetector::detectChArUcoBoard(const cv::Mat &image_grayscale, 
                                               const cv::Mat &camera_intrinsics, 
                                               const cv::Mat &camera_distortion_coefficients,
                                               cv::Vec3d &camera_rotation_out, 
                                               cv::Vec3d &camera_translation_out,
                                               cv::InputOutputArray image_with_detection_results, 
                                               bool show_rejected_markers) {
    
    std::vector<int> marker_ids, charuco_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_markers;
    std::vector<cv::Point2f> charuco_corners;
    
    cv::aruco::detectMarkers(image_grayscale, dictionary_, 
                             marker_corners, marker_ids, 
                             detector_parameters_, rejected_markers);
    
    if (!marker_ids.empty()) {
        cv::aruco::refineDetectedMarkers(image_grayscale, board_, 
                                         marker_corners, marker_ids, 
                                         rejected_markers, camera_intrinsics, 
                                         camera_distortion_coefficients);
    }
    
    int interpolated_corners = 0;
    if (!marker_ids.empty()) {
        interpolated_corners = cv::aruco::interpolateCornersCharuco(
            marker_corners, marker_ids, image_grayscale, board_, 
            charuco_corners, charuco_ids, camera_intrinsics, 
            camera_distortion_coefficients);
    }
    
    bool valid_pose = false;
    if (interpolated_corners > 0 && !camera_intrinsics.empty()) {
        valid_pose = cv::aruco::estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board_, camera_intrinsics, 
            camera_distortion_coefficients, camera_rotation_out, 
            camera_translation_out);
    }
    
    if (image_with_detection_results.needed()) {
        cv::cvtColor(image_grayscale, image_with_detection_results, cv::COLOR_GRAY2BGR);
        
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(image_with_detection_results, marker_corners);
        }
        
        if (show_rejected_markers && !rejected_markers.empty()) {
            cv::aruco::drawDetectedMarkers(image_with_detection_results, rejected_markers, 
                                           cv::noArray(), cv::Scalar(100, 0, 255));
        }
        
        if (interpolated_corners > 0) {
            cv::aruco::drawDetectedCornersCharuco(image_with_detection_results, 
                                                  charuco_corners, charuco_ids, 
                                                  cv::Scalar(255, 0, 0));
        }
        
        if (valid_pose) {
            float axis_length = 0.5f * static_cast<float>(
                std::min(number_of_squares_in_x_, number_of_squares_in_y_) * squares_sides_size_m_);
            cv::aruco::drawAxis(image_with_detection_results, camera_intrinsics, 
                                camera_distortion_coefficients, camera_rotation_out, 
                                camera_translation_out, axis_length);
        }
    }
    
    return valid_pose;
}

void ChArUcoOnlineDetector::fillPose(const cv::Vec3d &camera_rotation, 
                                     const cv::Vec3d &camera_translation, 
                                     geometry_msgs::msg::PoseStamped &pose_in_out) {
    
    cv::Mat rotation_matrix;
    cv::Rodrigues(camera_rotation, rotation_matrix);
    
    Eigen::Matrix3d eigen_rotation_matrix;
    cv::cv2eigen(rotation_matrix, eigen_rotation_matrix);
    
    Eigen::Quaterniond q(eigen_rotation_matrix);
    
    pose_in_out.pose.position.x = camera_translation(0);
    pose_in_out.pose.position.y = camera_translation(1);
    pose_in_out.pose.position.z = camera_translation(2);
    pose_in_out.pose.orientation.x = q.x();
    pose_in_out.pose.orientation.y = q.y();
    pose_in_out.pose.orientation.z = q.z();
    pose_in_out.pose.orientation.w = q.w();
}
