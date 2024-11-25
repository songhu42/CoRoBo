import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
# from std_msgs.msg import String
from user_interface.msg import UserInts


class User_inter_pub(Node):
    def __init__(self):
        
        super().__init__("user_inter_pub")
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_timer(1, self.print_hello)
        self.pub = self.create_publisher(UserInts, "user_inter_pub", self.qos_profile)
        self.number = 0
        self.msg = UserInts()
        self.msg.user_int1 = 1
        self.msg.user_int2 = 2
        self.msg.user_int3 = 3
        self.msg.stamp = self.get_clock().now().to_msg()

        self.get_logger().info("finish creator")
        
    def print_hello(self):
        self.get_logger().info("print_hello creator")
        self.msg.stamp = self.get_clock().now().to_msg()
        self.msg.user_int1 += 1
        self.msg.user_int2 += 1
        self.msg.user_int3 += 1
        
        self.pub.publish(self.msg)
        self.get_logger().info(f"stamp : {self.msg.stamp} user_int1 : {self.msg.user_int1}")
        self.number += 1

def main():
    rclpy.init()
    node = User_inter_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
