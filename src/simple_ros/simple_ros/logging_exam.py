import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity

class Logging_exam(Node):
    def __init__(self):
        super().__init__("logging_exam")
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_timer(1, self.log_exam)
        self.pub = self.create_publisher(String, "send", self.qos_profile)
        self.count = 0

    def log_exam(self):
        self.get_logger().log("print once example....", LoggingSeverity.WARN, once=True)

        msg = String()
        msg.data = f"hello, ros2! nice to meet you! + {self.count}"
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1

        # logger function filter 
        if self.debug_function_to_evaluate():
            self.get_logger().debug(msg.data)

    def debug_function_to_evaluate(self):
        if self.count % 5 == 0 : 
            return True
        return False

def main():
    rclpy.init()
    node = Logging_exam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
