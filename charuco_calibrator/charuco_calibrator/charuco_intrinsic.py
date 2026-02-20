#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import cv2.aruco as aruco
import os
import glob
from cv_bridge import CvBridge
import yaml
from ament_index_python.packages import get_package_share_directory

class CharucoIntrinsicCalibrator(Node):
    def __init__(self):
        super().__init__('charuco_intrinsic_calibrator')
        
        # Parámetros que se pueden pasar por línea de comandos
        self.declare_parameter('images_folder', '')
        self.declare_parameter('config_file', 'charuco_params.yaml')
        
        # Obtener parámetros
        self.images_folder = self.get_parameter('images_folder').value
        config_file = self.get_parameter('config_file').value
        
        # Cargar configuración desde YAML
        self.load_config(config_file)
        
        # Crear diccionario Charuco
        self.setup_charuco_board()
        
        self.bridge = CvBridge()
        self.calibrate_from_folder()

    def load_config(self, config_file):
        """Carga la configuración desde archivo YAML"""
        # Buscar el archivo de configuración
        possible_paths = [
            config_file,
            os.path.join(get_package_share_directory('charuco_calibrator'), 'config', config_file),
            os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'charuco_calibrator', 'config', config_file),
            os.path.join(os.path.dirname(__file__), '..', 'config', config_file)
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
        
        # Extraer parámetros de charuco_calibrator
        calib_params = config.get('charuco_calibrator', {}).get('ros__parameters', {})
        
        self.rows = calib_params.get('charuco_rows', 14)
        self.cols = calib_params.get('charuco_cols', 10)
        self.square_length = calib_params.get('square_length', 0.020)
        self.marker_length = calib_params.get('marker_length', 0.015)
        self.dictionary_name = calib_params.get('dictionary', 'DICT_4X4_1000')
        self.output_file = calib_params.get('output_file', 'calibration.yaml')
        
        self.get_logger().info(f"📋 Configuración cargada de: {config_path}")
        self.get_logger().info(f"   Tablero: {self.cols}x{self.rows}")
        self.get_logger().info(f"   Square: {self.square_length}m, Marker: {self.marker_length}m")
        self.get_logger().info(f"   Diccionario: {self.dictionary_name}")

    def setup_charuco_board(self):
        """Configura el tablero Charuco según los parámetros"""
        # Mapeo de nombres de diccionario a objetos de OpenCV
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
        
        try:
            dict_id = dictionary_map.get(self.dictionary_name, aruco.DICT_4X4_1000)
            self.aruco_dict = aruco.getPredefinedDictionary(dict_id)
        except:
            self.get_logger().warn(f"⚠️ No se pudo cargar {self.dictionary_name}, usando DICT_4X4_1000")
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        
        # Crear el board
        self.board = aruco.CharucoBoard(
            (self.cols, self.rows),
            self.square_length,
            self.marker_length,
            self.aruco_dict
        )

    def calibrate_from_folder(self):
        """Calibra la cámara usando imágenes de una carpeta"""
        
        if not self.images_folder:
            self.get_logger().error("❌ No se especificó images_folder")
            return
        
        if not os.path.exists(self.images_folder):
            self.get_logger().error(f"❌ La carpeta no existe: {self.images_folder}")
            return
        
        # Buscar imágenes
        image_paths = glob.glob(os.path.join(self.images_folder, '*.jpg')) + \
                      glob.glob(os.path.join(self.images_folder, '*.png'))
        
        if len(image_paths) == 0:
            self.get_logger().error(f"No se encontraron imágenes en {self.images_folder}")
            return
        
        self.get_logger().info(f"Encontradas {len(image_paths)} imágenes")
        
        all_corners = []
        all_ids = []
        image_size = None
        valid_images = 0
        
        for image_path in image_paths:
            self.get_logger().info(f"Procesando: {os.path.basename(image_path)}")
            img = cv2.imread(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            if image_size is None:
                image_size = gray.shape[::-1]
            
            # Detectar marcadores ArUco
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
            
            if ids is not None and len(ids) > 3:
                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )
                
                if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 10:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    valid_images += 1
                    self.get_logger().info(f"   ✅ Detectadas {len(charuco_corners)} esquinas")
                else:
                    self.get_logger().warn(f"   ⚠️ Pocas esquinas detectadas")
            else:
                self.get_logger().warn(f"   ⚠️ Pocos marcadores detectados")
        
        if len(all_corners) >= 5:
            self.get_logger().info(f"📊 Calibrando con {len(all_corners)} imágenes válidas...")
            
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=all_corners,
                charucoIds=all_ids,
                board=self.board,
                imageSize=image_size,
                cameraMatrix=None,
                distCoeffs=None
            )
            
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info("✅ CALIBRACIÓN EXITOSA")
            self.get_logger().info(f"{'='*50}")
            self.get_logger().info(f"📏 Error de reproyección: {ret:.6f}")
            self.get_logger().info(f"\n📷 Matriz de cámara:\n{camera_matrix}")
            self.get_logger().info(f"\n📐 Coeficientes de distorsión:\n{dist_coeffs.reshape(-1)}")
            
            # Guardar calibración
            self.save_calibration(camera_matrix, dist_coeffs, image_size, ret)
        else:
            self.get_logger().error(f"❌ No se detectaron suficientes patrones. Válidas: {len(all_corners)}/5")

    def save_calibration(self, camera_matrix, dist_coeffs, image_size, reprojection_error):
        """Guarda los parámetros de calibración en archivo YAML"""
        
        calibration_data = {
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.reshape(-1).tolist(),
            'image_width': image_size[0],
            'image_height': image_size[1],
            'reprojection_error': float(reprojection_error),
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
        
        self.get_logger().info(f"💾 Calibración guardada en: {output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = CharucoIntrinsicCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
