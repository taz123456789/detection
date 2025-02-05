import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        self.serial_port = serial.Serial('COM8', 115200, timeout=1)
        
        self.subscription = self.create_subscription(
            Float32,
            'target_angle',
            self.serial_callback,
            10
        )

    def serial_callback(self, msg):
        angle = msg.data
        try:
            self.serial_port.write(f"{angle}\n".encode('utf-8'))
            self.serial_port.flush()
            read = self.serial_port.readline()
            self.get_logger().info(f"Serial read: {read}")
            self.get_logger().info(f"Sent angle: {angle}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {str(e)}")

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()