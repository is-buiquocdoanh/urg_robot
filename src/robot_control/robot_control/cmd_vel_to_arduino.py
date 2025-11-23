import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

K_PULSE2V = 0.00013   # phải giống Arduino
K_PULSE2W = 0.0035

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)
        time.sleep(2)

        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 10
        )

        self.get_logger().info("Arduino bridge ready.")

    def cmd_callback(self, msg: Twist):
        V = msg.linear.x       # m/s
        W = msg.angular.z      # rad/s
        
        cmd = "x"
        pulse = 0

        # ƯU TIÊN XOAY NẾU W != 0
        if abs(W) > 0.01:
            pulse = abs(int(W / K_PULSE2W))
            pulse = min(pulse, 9999)  # bảo vệ

            if W > 0:
                cmd = f"a{pulse}"   # quay trái
            else:
                cmd = f"d{pulse}"   # quay phải

        # Nếu không xoay → xử lý tiến/lùi
        elif abs(V) > 0.01:
            pulse = abs(int(V / K_PULSE2V))
            pulse = min(pulse, 9999)

            if V > 0:
                cmd = f"w{pulse}"   # tiến
            else:
                cmd = f"s{pulse}"   # lùi

        else:
            cmd = "x"               # dừng

        # Gửi kèm newline (rất quan trọng)
        self.ser.write((cmd + "\n").encode())

        self.get_logger().info(f"[TX] {cmd}  (V={V:.3f}, W={W:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
