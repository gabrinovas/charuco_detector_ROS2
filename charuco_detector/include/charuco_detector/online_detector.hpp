#ifndef CHARUCO_DETECTOR_HPP_
#define CHARUCO_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <charuco_detector_interfaces/srv/eye2base.hpp>

class ChArUcoDetector : public rclcpp::Node 
{ 
public:
  ChArUcoDetector();
  ~ChArUcoDetector();

  void setupConfigurationFromParameterServer();
  void startDetection();

private:
  // Callbacks
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // Image processing
  void applyMedianBlur(cv::Mat &image_in_out);
  void applyDynamicRange(cv::Mat& image_in_out);
  void applyBilateralFilter(cv::Mat &image_in_out);
  void applyCLAHE(cv::Mat &image_in_out);
  void applyAdaptiveThreshold(cv::Mat &image_in_out);
  
  // ChArUco detection
  bool detectChArUcoBoard(const cv::Mat &image_grayscale, 
                          const cv::Mat &camera_intrinsics, 
                          const cv::Mat &camera_distortion_coefficients,
                          cv::Vec3d &camera_rotation_out, 
                          cv::Vec3d &camera_translation_out,
                          cv::InputOutputArray image_with_detection_results, 
                          bool show_rejected_markers);
  
  void fillPose(const cv::Vec3d &camera_rotation, 
                const cv::Vec3d &camera_translation, 
                geometry_msgs::msg::PoseStamped &pose_in_out);

  // Service
  void eyeToBaseCallback(const std::shared_ptr<charuco_detector_interfaces::srv::Eye2base::Request> request,
                         std::shared_ptr<charuco_detector_interfaces::srv::Eye2base::Response> response);

  // Parameters
  // ChArUco board parameters
  double squares_sides_size_m_;
  double markers_sides_size_m_;
  int number_of_bits_for_markers_sides_;
  int number_of_markers_;
  int number_of_squares_in_x_;
  int number_of_squares_in_y_;
  int dictionary_id_;
  
  // Image processing parameters
  bool use_median_blur_;
  int median_blur_k_size_;
  bool use_dynamic_range_;
  bool use_bilateral_filter_;
  int bilateral_filter_pixel_neighborhood_;
  double bilateral_filter_sigma_color_;
  double bilateral_filter_sigma_space_;
  int bilateral_filter_border_type_;
  bool use_clahe_;
  double clahe_clip_limit_;
  int clahe_size_x_;
  int clahe_size_y_;
  bool use_adaptive_threshold_;
  double adaptive_threshold_max_value_;
  int adaptive_threshold_method_;
  int adaptive_threshold_type_;
  int adaptive_threshold_block_size_;
  double adaptive_threshold_constant_offset_from_mean_;
  
  // TF parameters
  bool use_static_tf_broadcaster_;
  double tf_broadcaster_republish_rate_;
  std::string sensor_frame_override_;
  std::string charuco_tf_frame_;
  
  // Topic names
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string image_results_publish_topic_;
  std::string charuco_pose_publish_topic_;
  
  // OpenCV objects
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Mat camera_intrinsics_matrix_;
  cv::Mat camera_distortion_coefficients_matrix_;
  
  // ROS2 objects
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr charuco_pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_results_publisher_;
  rclcpp::Service<charuco_detector_interfaces::srv::Eye2base>::SharedPtr eye_to_base_service_;
  
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher image_transport_publisher_;
  
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
  bool transform_stamped_valid_;
  
  // Camera info
  bool camera_info_received_;
};

#endif //CHARUCO_DETECTOR_HPP_
