import rclpy
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

class Service_server(Node):
    def __init__(self):
        super().__init__("crebs_center")
        self.create_timer(1, self.print_hello)
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

    def print_hello(self):
        #self.get_logger().info(f"current bool : {self.bool}")
        pass

def main():
    rclpy.init()
    node = Service_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupted")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
