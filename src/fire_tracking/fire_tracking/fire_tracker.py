#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from time import sleep

# ros2 run fire_detection fire_detector --ros-args -p conf_thres:=0.19
# ros2 run fire_tracking fire_tracker --ros-args -p image_width:=600 -p kp:=0.0007 -p kd:=0.0006


class FireTracker(Node):
    def __init__(self):
        super().__init__('fire_tracker')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('kd', 0.0005)
        self.declare_parameter('max_angular_speed', 0.5)

        self.image_width = self.get_parameter('image_width').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular_speed').value

        self.last_error = 0.0

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/fire_detections',
            self.detection_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bomba_pub = self.create_publisher(Bool, '/bomba', 10)


        self.get_logger().info("FireTracker listo para seguir fuego.")

    def detection_callback(self, msg):
        
        if not msg.detections:
            self.get_logger().info('No detecciones.')
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            
            bomba_msg = Bool()
            bomba_msg.data = False
            self.bomba_pub.publish(bomba_msg)
            return
        

        # Tomar el centro de la primera detección
        bomba_msg = Bool()
        
        
        
        detection = msg.detections[0]
        x_center = detection.bbox.center.position.x

        size_x = detection.bbox.size_x
        size_y = detection.bbox.size_y
        
        
        if (size_x >550 or size_y>550):
            self.get_logger().info('Cerca.')
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            bomba_msg.data = True
            self.bomba_pub.publish(bomba_msg)
            sleep(5)
            return

        bomba_msg.data = False
        self.bomba_pub.publish(bomba_msg)

        # Calcular error (centro imagen - centro fuego)
        error = (self.image_width / 2) - x_center
        derivative = error - self.last_error
        angular_z = self.kp * error + self.kd * derivative
        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)
        self.last_error = error
        
        self.get_logger().info(f"x_center: {x_center}")

        # Publicar comando
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = 0.35  # opcional: avanza lentamente hacia el fuego ros2 run fire_tracking fire_tracker --ros-args -p image_width:=600 -p kp:=0.0015 -p kd:=0.001
        # ros2 run fire_tracking fire_tracker --ros-args -p image_width:=600 -p kp:=0.0007 -p kd:=0.0006
        # ros2 run fire_detection fire_detector --ros-args -p conf_thres:=0.25
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FireTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
