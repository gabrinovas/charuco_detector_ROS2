cmake_minimum_required(VERSION 3.8)
project(charuco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(charuco_detector_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(executable_name charuco_detector)
set(library_name ${executable_name}_core)

# Include directories
include_directories(
  include
)

# Library
add_library(${library_name} SHARED
  src/charuco_detector.cpp
)

target_compile_features(${library_name} PUBLIC cxx_std_17)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  OpenCV
  charuco_detector_interfaces
)

# Executable
add_executable(charuco_detector_node 
  src/charuco_detector_node.cpp
)

target_link_libraries(charuco_detector_node
  ${library_name}
)

ament_target_dependencies(charuco_detector_node
  rclcpp
  charuco_detector_interfaces
)

# Install
install(TARGETS
  ${library_name}
  charuco_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  yaml
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/hand_eye_trans.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(rclcpp Eigen3 OpenCV cv_bridge)

ament_package()
