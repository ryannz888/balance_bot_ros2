import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
from geometry_msgs.msg import Twist

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.enc_pub = self.create_publisher(String, '/wheel/encoders', 10)
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1.0)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('serial_bridge started')
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()

    def _cb(self, msg: Twist):
    	linear  = msg.linear.x
    	angular = msg.angular.z
    	left_pwm  = int((linear - angular) * 100)
    	right_pwm = int((linear + angular) * 100)
    	cmd = f'M,{left_pwm},{right_pwm}\n'
    	self.ser.write(cmd.encode())

    def _read_serial(self):
        while True:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('E,'):
                continue
            parts = line.split(',')
            if len(parts) != 3:
                continue
            msg = String()
            msg.data = f'{parts[1]},{parts[2]}'
            self.enc_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(SerialBridge())
    rclpy.shutdown()

