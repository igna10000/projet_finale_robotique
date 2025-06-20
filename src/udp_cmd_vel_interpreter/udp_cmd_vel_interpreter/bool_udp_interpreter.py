import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socket
import json

class BombaUDPInterpreter(Node):
    def __init__(self):
        super().__init__('bomba_udp_interpreter')
        self.subscription = self.create_subscription(Bool, 'bomba', self.listener_callback, 10)

        # Configuraciones UDP
        self.ESP32_IP = "192.168.10.14"  # Cambia a la IP de tu ESP32
        self.ESP32_PORT_SEND = 2023      # Puerto donde la ESP32 recibe el Bool
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info("Suscrito al tópico bomba y listo para enviar datos a la ESP32.")

    def listener_callback(self, msg):
        # Crear mensaje JSON a enviar por UDP
        bomba_msg = {
            "bomba": {
                "data": msg.data
            }
        }
        # Enviar mensaje a ESP32
        self.sock_send.sendto(json.dumps(bomba_msg).encode(), (self.ESP32_IP, self.ESP32_PORT_SEND))
        self.get_logger().info(f'Enviado a ESP32: {bomba_msg}')

def main(args=None):
    rclpy.init(args=args)
    bomba_udp_interpreter = BombaUDPInterpreter()
    rclpy.spin(bomba_udp_interpreter)

    # Destruir el nodo
    bomba_udp_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

