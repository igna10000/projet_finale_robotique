import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
import time


class ArucoCmdVel(Node):

    def __init__(self):
        super().__init__('aruco_cmd_vel_node')

        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_pose = None
        self.aruco_detected = False

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # PID variables
        self.kp = 0.6
        self.ki = 0.02
        self.kd = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = self.get_clock().now()

    def aruco_callback(self, msg):
        if len(msg.poses) > 0:
            self.aruco_detected = True
            self.last_pose = msg.poses[0]
            x = self.last_pose.position.x
            z = self.last_pose.position.z
            self.get_logger().info(f'Detección ArUco: x = {x:.3f}, z = {z:.3f}')
        else:
            self.aruco_detected = False
            self.last_pose = None
            self.get_logger().info('Sin detección ArUco')

    def control_loop(self):
        twist = Twist()

        if self.aruco_detected and self.last_pose is not None:
            error = self.last_pose.position.x  # Queremos que x = 0
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9  # seg
            self.last_time = current_time

            # PID components
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            self.prev_error = error

            # PID output for angular control
            angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
            angular_z = max(min(angular_z, 0.7), -0.7)  # limit max angular speed

            # Solo avanza si está lejos
            if self.last_pose.position.z >= 0.35:
                twist.linear.x = 0.3
            else:
                twist.linear.x = 0.0
                self.get_logger().info('Z menor a 0.35, deteniendo avance')

            twist.angular.z = -angular_z  # signo depende del eje del sistema de coordenadas
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('No se detectó ArUco, deteniendo')

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

