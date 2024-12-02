import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool 
import time

# CoRoBo Service Centor .. 
# moveto, move, armto, greep ... 

class CrbmCenter(Node):
    def __init__(self):
        super().__init__("crebm_center")

        # 나의 현재 정보 publishing 
        self.create_timer(1/10, self.update_me)
        self.pub = self.create_publisher(Twist, "cmd_vel", self.qos_profile)

        self.create_service(SetBool, "setBoolService", self.setBool_callback) 
        self.bool = bool()
        self.count = 0

    def setBool_callback(self, request:SetBool.Request, response:SetBool.Response):
        self.get_logger().info(f"request data : {request.data}")
        self.bool = request.data
        self.count += 1

        if request.data:
            response.success = True
            response.message = "True response"
        else :
            response.success = False
            response.message = "False response"

        self.get_logger().info(f"response {self.count} => {response.message}")

        return response

    def update_me(self):
        #self.get_logger().info(f"current bool : {self.bool}")
        pass

def main():
    rclpy.init()
    node = CrbmCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupted")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
