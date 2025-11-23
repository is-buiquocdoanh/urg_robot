import rclpy
from rclpy.node import Node
import serial
import threading

from std_msgs.msg import String

class UARTBridge(Node):
    def __init__(self):
        super().__init__("uart_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f"UART Bridge connected: {port} @ {baud}")

        self.pub_rx = self.create_publisher(String, "/uart_rx", 20)
        self.sub_tx = self.create_subscription(String, "/uart_tx", self.tx_callback, 20)

        # Thread đọc liên tục
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()

    def tx_callback(self, msg):
        """Nhận dữ liệu từ ROS, gửi xuống Arduino"""
        try:
            self.ser.write(msg.data.encode())
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def reader(self):
        """Đọc dữ liệu từ Arduino và publish lên ROS"""
        while True:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.pub_rx.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
