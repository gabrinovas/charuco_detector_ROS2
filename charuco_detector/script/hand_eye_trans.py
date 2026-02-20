#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from charuco_detector_interfaces.srv import Eye2base
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class HandEyeTrans(Node):
    def __init__(self):
        super().__init__('hand_eye_trans')
        
        # Declare parameters
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool0')
        self.declare_parameter('eye_in_hand_mode', True)
        self.declare_parameter('customize', False)
        self.declare_parameter('filename', '')
        
        # Get parameters
        self.base_link = self.get_parameter('base_link').value
        self.tip_link = self.get_parameter('tip_link').value
        self.eye_in_hand_mode = self.get_parameter('eye_in_hand_mode').value
        self.customize = self.get_parameter('customize').value
        self.filename = self.get_parameter('filename').value
        
        # Paths
        pkg_share = get_package_share_directory('charuco_detector')
        self.camera_cali_dir = os.path.join(pkg_share, 'config', 'camera_calibration')
        self.hand_eye_cali_dir = os.path.join(pkg_share, 'config', 'hand_eye_calibration')
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Load calibrations
        self.camera_matrix = None
        self.hand_eye_transform = None
        self.load_calibrations()
        
        # Service
        self.srv = self.create_service(Eye2base, 'eye2base', self.eye2base_callback)
        self.get_logger().info('HandEyeTrans node initialized')
        
    def load_calibrations(self):
        """Load camera calibration and hand-eye calibration"""
        try:
            # Load camera calibration
            camera_file = os.path.join(self.camera_cali_dir, 'camera_calibration.ini')
            if os.path.exists(camera_file):
                self.camera_matrix = self.load_camera_calibration(camera_file)
                self.get_logger().info('Camera calibration loaded')
            else:
                self.get_logger().warn(f'Camera calibration not found: {camera_file}')
            
            # Load hand-eye calibration
            if self.customize and self.filename:
                calib_file = os.path.join(self.hand_eye_cali_dir, self.filename)
            else:
                if self.eye_in_hand_mode:
                    calib_file = os.path.join(self.hand_eye_cali_dir, 'eye_in_hand_calibration.ini')
                else:
                    calib_file = os.path.join(self.hand_eye_cali_dir, 'eye_to_hand_calibration.ini')
            
            if os.path.exists(calib_file):
                self.hand_eye_transform = self.load_hand_eye_calibration(calib_file)
                self.get_logger().info(f'Hand-eye calibration loaded from {calib_file}')
            else:
                self.get_logger().warn(f'Hand-eye calibration not found: {calib_file}')
                
        except Exception as e:
            self.get_logger().error(f'Error loading calibrations: {str(e)}')
    
    def load_camera_calibration(self, filename):
        """Load camera calibration from INI file"""
        camera_matrix = np.eye(3)
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
                
            in_intrinsic = False
            for line in lines:
                line = line.strip()
                if line.startswith('[Intrinsic]'):
                    in_intrinsic = True
                    continue
                elif line.startswith('['):
                    in_intrinsic = False
                    continue
                    
                if in_intrinsic and '=' in line:
                    key, value = line.split('=')
                    key = key.strip()
                    value = float(value.strip())
                    
                    if key == '0_0':
                        camera_matrix[0, 0] = value
                    elif key == '0_1':
                        camera_matrix[0, 1] = value
                    elif key == '0_2':
                        camera_matrix[0, 2] = value
                    elif key == '1_0':
                        camera_matrix[1, 0] = value
                    elif key == '1_1':
                        camera_matrix[1, 1] = value
                    elif key == '1_2':
                        camera_matrix[1, 2] = value
                    elif key == '2_0':
                        camera_matrix[2, 0] = value
                    elif key == '2_1':
                        camera_matrix[2, 1] = value
                    elif key == '2_2':
                        camera_matrix[2, 2] = value
        except Exception as e:
            self.get_logger().error(f'Error loading camera calibration: {str(e)}')
        
        return camera_matrix
    
    def load_hand_eye_calibration(self, filename):
        """Load hand-eye calibration from INI file"""
        transform = np.eye(4)
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
                
            in_calibration = False
            x = y = z = qx = qy = qz = qw = 0.0
            
            for line in lines:
                line = line.strip()
                if line.startswith('[hand_eye_calibration]'):
                    in_calibration = True
                    continue
                elif line.startswith('['):
                    in_calibration = False
                    continue
                    
                if in_calibration and '=' in line:
                    key, value = line.split('=')
                    key = key.strip()
                    value = float(value.strip())
                    
                    if key == 'x':
                        x = value
                    elif key == 'y':
                        y = value
                    elif key == 'z':
                        z = value
                    elif key == 'qx':
                        qx = value
                    elif key == 'qy':
                        qy = value
                    elif key == 'qz':
                        qz = value
                    elif key == 'qw':
                        qw = value
            
            # Convert quaternion to rotation matrix
            rot = self.quaternion_to_matrix([qx, qy, qz, qw])
            transform[:3, :3] = rot
            transform[:3, 3] = [x, y, z]
            
        except Exception as e:
            self.get_logger().error(f'Error loading hand-eye calibration: {str(e)}')
        
        return transform
    
    def quaternion_to_matrix(self, q):
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
    
    def matrix_to_quaternion(self, m):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
        trace = np.trace(m)
        if trace > 0:
            s = 2.0 * np.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                w = (m[2, 1] - m[1, 2]) / s
                x = 0.25 * s
                y = (m[0, 1] + m[1, 0]) / s
                z = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                w = (m[0, 2] - m[2, 0]) / s
                x = (m[0, 1] + m[1, 0]) / s
                y = 0.25 * s
                z = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                w = (m[1, 0] - m[0, 1]) / s
                x = (m[0, 2] + m[2, 0]) / s
                y = (m[1, 2] + m[2, 1]) / s
                z = 0.25 * s
        return np.array([x, y, z, w])
    
    def get_robot_transform(self):
        """Get current robot transform from TF"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_link,
                self.tip_link,
                rclpy.time.Time()
            )
            return trans
        except Exception as e:
            self.get_logger().warn(f'Failed to get robot transform: {str(e)}')
            return None
    
    def eye2base_callback(self, request, response):
        """Service callback for eye2base transformation"""
        self.get_logger().debug('Eye2base service called')
        
        try:
            # Get robot transform
            robot_tf = self.get_robot_transform()
            if robot_tf is None:
                raise Exception('Could not get robot transform')
            
            # Convert to numpy
            robot_mat = np.eye(4)
            robot_mat[:3, :3] = self.quaternion_to_matrix([
                robot_tf.transform.rotation.x,
                robot_tf.transform.rotation.y,
                robot_tf.transform.rotation.z,
                robot_tf.transform.rotation.w
            ])
            robot_mat[:3, 3] = [
                robot_tf.transform.translation.x,
                robot_tf.transform.translation.y,
                robot_tf.transform.translation.z
            ]
            
            # Process input pose
            if len(request.ini_pose) == 3:  # 3D point
                point = np.array(request.ini_pose + [1.0]).reshape(4, 1)
                
                if self.eye_in_hand_mode:
                    result = robot_mat @ np.linalg.inv(self.hand_eye_transform) @ point
                else:
                    result = robot_mat @ point
                
                response.pos = result[:3].flatten().tolist()
                response.quat = [0.0, 0.0, 0.0, 1.0]  # No rotation for points
                
            elif len(request.ini_pose) == 16:  # 4x4 transform matrix
                mat = np.array(request.ini_pose).reshape(4, 4)
                
                if self.eye_in_hand_mode:
                    result = robot_mat @ np.linalg.inv(self.hand_eye_transform) @ mat
                else:
                    result = robot_mat @ mat
                
                response.trans = result.flatten().tolist()
                response.pos = result[:3, 3].flatten().tolist()
                response.quat = self.matrix_to_quaternion(result[:3, :3]).tolist()
            
            self.get_logger().debug('Eye2base service completed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error in eye2base service: {str(e)}')
            response.trans = []
            response.pos = []
            response.quat = []
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeTrans()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
