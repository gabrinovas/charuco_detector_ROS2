#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import cv2.aruco as aruco
import os
import glob
import yaml
import tf_transformations
from ament_index_python.packages import get_package_share_directory

class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('hand_eye_calibrator')
        
        # Parámetros
        self.declare_parameter('images_folder', '')
        self.declare_parameter('robot_poses_file', 'robot_poses.txt')
        self.declare_parameter('camera_intrinsics_file', 'calibration.yaml')
        self.declare_parameter('config_file', 'charuco_params.yaml')
        
        # Obtener parámetros
        self.images_folder = self.get_parameter('images_folder').value
        self.robot_poses_file = self.get_parameter('robot_poses_file').value
        self.camera_intrinsics_file = self.get_parameter('camera_intrinsics_file').value
        config_file = self.get_parameter('config_file').value
        
        # Cargar configuración desde YAML
        self.load_config(config_file)
        
        # Cargar intrínsecos de cámara
        self.load_camera_intrinsics()
        
        # Configurar Charuco
        self.setup_charuco_board()
        
        self.get_logger().info(f"🤖 Iniciando calibración ojo-mano con tablero {self.cols}x{self.rows}")
        self.calibrate_hand_eye()

    def load_config(self, config_file):
        """Carga la configuración desde archivo YAML"""
        possible_paths = [
            config_file,
            os.path.join(get_package_share_directory('charuco_calibrator'), 'config', config_file),
            os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'charuco_calibrator', 'config', config_file)
        ]
        
        config_path = None
        for path in possible_paths:
            if os.path.exists(path):
                config_path = path
                break
        
        if config_path is None:
            self.get_logger().error(f"❌ No se encontró archivo de configuración: {config_file}")
            raise FileNotFoundError(f"Config file not found: {config_file}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        calib_params = config.get('charuco_calibrator', {}).get('ros__parameters', {})
        
        self.rows = calib_params.get('charuco_rows', 14)
        self.cols = calib_params.get('charuco_cols', 10)
        self.square_length = calib_params.get('square_length', 0.020)
        self.marker_length = calib_params.get('marker_length', 0.015)
        self.dictionary_name = calib_params.get('dictionary', 'DICT_4X4_1000')
        self.output_file = calib_params.get('hand_eye_output', 'hand_eye_calibration.yaml')
        
        self.get_logger().info(f"📋 Configuración cargada de: {config_path}")

    def setup_charuco_board(self):
        """Configura el tablero Charuco"""
        dictionary_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_5X5_1000': aruco.DICT_5X5_1000,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_6X6_1000': aruco.DICT_6X6_1000,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_7X7_250': aruco.DICT_7X7_250,
            'DICT_7X7_1000': aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL
        }
        
        dict_id = dictionary_map.get(self.dictionary_name, aruco.DICT_4X4_1000)
        self.aruco_dict = aruco.getPredefinedDictionary(dict_id)
        
        self.board = aruco.CharucoBoard(
            (self.cols, self.rows),
            self.square_length,
            self.marker_length,
            self.aruco_dict
        )

    def load_camera_intrinsics(self):
        """Carga la matriz de cámara y coeficientes de distorsión"""
        intrinsics_path = os.path.join(self.images_folder, self.camera_intrinsics_file)
        if not os.path.exists(intrinsics_path):
            self.get_logger().error(f"❌ No se encuentra archivo de intrínsecos: {intrinsics_path}")
            raise FileNotFoundError(f"Intrinsics file not found: {intrinsics_path}")
        
        with open(intrinsics_path, 'r') as f:
            data = yaml.safe_load(f)
        
        self.camera_matrix = np.array(data['camera_matrix'])
        self.dist_coeffs = np.array(data['distortion_coefficients'])
        self.get_logger().info("✅ Intrínsecos de cámara cargados correctamente")

    def load_robot_poses(self):
        """Carga las poses del robot desde archivo"""
        poses_path = os.path.join(self.images_folder, self.robot_poses_file)
        robot_poses = []
        
        if not os.path.exists(poses_path):
            self.get_logger().error(f"❌ No se encuentra el archivo de poses: {poses_path}")
            return robot_poses
        
        with open(poses_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                values = list(map(float, line.strip().split()))
                if len(values) == 7:
                    pos = np.array(values[0:3])
                    quat = np.array(values[3:7])
                    robot_poses.append((pos, quat))
        
        self.get_logger().info(f"✅ Cargadas {len(robot_poses)} poses del robot")
        return robot_poses

    def calibrate_hand_eye(self):
        """Realiza calibración ojo-mano"""
        
        image_paths = sorted(glob.glob(os.path.join(self.images_folder, '*.jpg')) + 
                            glob.glob(os.path.join(self.images_folder, '*.png')))
        robot_poses = self.load_robot_poses()
        
        if len(image_paths) != len(robot_poses):
            self.get_logger().error(f"❌ Número de imágenes ({len(image_paths)}) no coincide con poses ({len(robot_poses)})")
            return
        
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        valid_pairs = 0
        
        for i, (image_path, (robot_pos, robot_quat)) in enumerate(zip(image_paths, robot_poses)):
            self.get_logger().info(f"📸 Procesando pose {i+1}/{len(image_paths)}")
            
            img = cv2.imread(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
            
            if ids is not None and len(ids) > 3:
                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )
                
                if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 10:
                    ret, rvec, tvec = aruco.estimatePoseCharucoBoard(
                        charuco_corners, charuco_ids, self.board,
                        self.camera_matrix, self.dist_coeffs, None, None
                    )
                    
                    if ret:
                        R_ct, _ = cv2.Rodrigues(rvec)
                        
                        R_bg = tf_transformations.quaternion_matrix(robot_quat)[:3, :3]
                        t_bg = robot_pos.reshape(3, 1)
                        
                        R_gripper2base.append(R_bg)
                        t_gripper2base.append(t_bg)
                        R_target2cam.append(R_ct)
                        t_target2cam.append(tvec.reshape(3, 1))
                        valid_pairs += 1
                        self.get_logger().info(f"   ✅ Pose válida #{valid_pairs}")
        
        if len(R_gripper2base) < 3:
            self.get_logger().error(f"❌ No hay suficientes detecciones válidas ({len(R_gripper2base)}/3)")
            return
        
        self.get_logger().info(f"📊 Calibrando con {len(R_gripper2base)} pares válidos...")
        
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("✅ CALIBRACIÓN OJO-MANO COMPLETADA")
        self.get_logger().info("="*50)
        self.get_logger().info(f"Rotación:\n{R_cam2gripper}")
        self.get_logger().info(f"Traslación:\n{t_cam2gripper.flatten()}")
        
        self.save_hand_eye_calibration(R_cam2gripper, t_cam2gripper)

    def save_hand_eye_calibration(self, R, t):
        """Guarda la calibración ojo-mano"""
        
        T = np.vstack([np.hstack([R, t]), [0, 0, 0, 1]])
        quat = tf_transformations.quaternion_from_matrix(T)
        
        calibration_data = {
            'transform_matrix': T.tolist(),
            'rotation': R.tolist(),
            'translation': t.flatten().tolist(),
            'quaternion': quat.tolist(),
            'calibration_date': self.get_clock().now().to_msg().sec,
            'charuco_config': {
                'rows': self.rows,
                'cols': self.cols,
                'square_length': self.square_length,
                'marker_length': self.marker_length,
                'dictionary': self.dictionary_name
            }
        }
        
        output_path = os.path.join(self.images_folder, self.output_file)
        with open(output_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)
        
        self.get_logger().info(f"💾 Calibración ojo-mano guardada en: {output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
