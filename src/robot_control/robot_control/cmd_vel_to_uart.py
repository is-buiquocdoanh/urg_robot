import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelToUART(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_uart")

        self.pub = self.create_publisher(String, "/uart_tx", 20)
        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 20
        )

    def send(self, ch):
        msg = String()
        msg.data = ch
        self.pub.publish(msg)
        self.get_logger().info(f"TX: {ch}")

    def cmd_callback(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z

        # STOP
        if lin == 0 and ang == 0:
            self.send("x")
            return

        # Đi thẳng / lùi
        if lin > 0:
            self.send("w")
        elif lin < 0:
            self.send("s")

        # Rẽ trái / phải
        if ang > 0:
            self.send("a")
        elif ang < 0:
            self.send("d")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUART()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
