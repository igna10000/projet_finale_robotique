#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import sys
from pathlib import Path
import torch
import numpy as np
from ament_index_python.packages import get_package_share_directory
import cv2

# Configuración de rutas YOLOv5
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / 'yolov5'  # Ruta al directorio yolov5
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

try:
    from models.common import DetectMultiBackend
    from utils.general import check_img_size, non_max_suppression, scale_boxes
    from utils.torch_utils import select_device
except ImportError as e:
    print(f"Error importing YOLOv5 modules: {e}")
    sys.exit(1)

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('weights', 'yolov5s_best.pt'),
                ('img_size', 640),
                ('conf_thres', 0.15),
                ('iou_thres', 0.45),
                ('device', 'cpu')
            ])

        self.init_yolov5()
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(
            Detection2DArray, '/fire_detections', 10)

        self.get_logger().info(f"FireDetector inicializado con modelo: {self.weights}")

    def init_yolov5(self):
        """Inicializa el modelo YOLOv5 con ruta portable"""
        package_share = get_package_share_directory('fire_detection')
        weights_param = self.get_parameter('weights').value
        self.weights = Path(package_share) / 'yolov5' / weights_param
        self.device = select_device(self.get_parameter('device').value)
        self.model = DetectMultiBackend(str(self.weights), device=self.device)
        self.stride = self.model.stride
        self.img_size = check_img_size([self.get_parameter('img_size').value]*2, s=self.stride)
        self.names = self.model.names
        self.model.warmup(imgsz=(1, 3, *self.img_size))

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img = self.preprocess(cv_img)
            detections = self.detect_fire(img, cv_img.shape)
            self.pub.publish(detections)
        except Exception as e:
            self.get_logger().error(f"Error en procesamiento: {str(e)}", throttle_duration_sec=5)

    def preprocess(self, img):
        img = img[:, :, ::-1]  # BGR -> RGB
        img = cv2.resize(img, (self.img_size[0], self.img_size[1]))
        img = torch.from_numpy(img.transpose(2, 0, 1)).to(self.device)
        img = img.half() if self.model.fp16 else img.float()
        img = img / 255.0
        return img.unsqueeze(0)

    def detect_fire(self, img, orig_shape):
        detections = Detection2DArray()
        pred = self.model(img, augment=False, visualize=False)
        
        # Versión compatible de non_max_suppression
        try:
            # Primero intenta con la firma más reciente
            pred = non_max_suppression(
                pred,
                conf_thres=self.get_parameter('conf_thres').value,
                iou_thres=self.get_parameter('iou_thres').value,
                max_det=1000)
        except TypeError:
            # Fallback para versiones antiguas
            pred = non_max_suppression(
                pred,
                self.get_parameter('conf_thres').value,
                self.get_parameter('iou_thres').value)
        
        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], orig_shape).round()
                for *xyxy, conf, cls in reversed(det):
                    if cls == 0:  # Solo fuego (clase 0)
                        detection = Detection2D()
                        detection.bbox.center.position.x = float((xyxy[0] + xyxy[2]) / 2)
                        detection.bbox.center.position.y = float((xyxy[1] + xyxy[3]) / 2)
                        detection.bbox.size_x = float(xyxy[2] - xyxy[0])
                        detection.bbox.size_y = float(xyxy[3] - xyxy[1])
                        detections.detections.append(detection)

        return detections

def main(args=None):
    rclpy.init(args=args)
    node = FireDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
