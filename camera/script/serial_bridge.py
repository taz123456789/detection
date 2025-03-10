import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ESP32AngleControl(Node):
    def __init__(self):
        super().__init__('esp32_angle_control')
        self.subscription = self.create_subscription(
            Float32,
            'target_angle', 
            self.listener_callback,
            10)
        
        self.serial_port = serial.Serial('COM11', 115200, timeout=1)

    def listener_callback(self, msg):
        angle = msg.data  # Target angle 
        self.serial_port.write((str(angle) + "\n").encode())  # Send to ESP32
        self.get_logger().info(f"Sent target angle: {angle}")

def main(args=None):
    rclpy.init(args=args)
    esp32_angle_control = ESP32AngleControl()
    rclpy.spin(esp32_angle_control)
    esp32_angle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
