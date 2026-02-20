cmake_minimum_required(VERSION 3.8)
project(charuco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ===== ONLINE DETECTOR =====
add_executable(online_detector_node
  src/online_detector_node.cpp
  src/online_detector.cpp
)

target_include_directories(online_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(online_detector_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  charuco_detector_interfaces
)

target_link_libraries(online_detector_node
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
)

# ===== OFFLINE DETECTOR =====
add_executable(offline_detector_node
  src/offline_detector_node.cpp
  src/offline_detector.cpp
)

target_include_directories(offline_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(offline_detector_node
  rclcpp
  OpenCV
)

target_link_libraries(offline_detector_node
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
)

# ===== MAIN INTELIGENTE =====
add_executable(charuco_detector
  src/charuco_detector_main.cpp
  src/online_detector.cpp
  src/offline_detector.cpp
)

target_include_directories(charuco_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(charuco_detector
  rclcpp
  OpenCV
  charuco_detector_interfaces
)

target_link_libraries(charuco_detector
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

# Instalar todos los ejecutables
install(TARGETS
  online_detector_node
  offline_detector_node
  charuco_detector
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
