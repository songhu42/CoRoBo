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

class Service_client(Node):
    def __init__(self):
        super().__init__("Service_client")
        self.create_timer(0.5, self.print_status)
        self.client = self.create_client(SetBool, "setBoolService") 
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service is not available!!")

        self.request = SetBool.Request()

        self.create_timer(2.0, self.send_request)
        # self.send_request()   
        self.bool = bool()
    
    def send_request(self):
        self.bool = not self.bool 
        self.request.data = self.bool
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response : SetBool.Response = future.result()
        self.get_logger().info(f"result : {response.success}")
        self.get_logger().info(f"message : {response.message}")

    def setBool_callback(self, request:SetBool.Request, response:SetBool.Response):
        self.get_logger().info(f"request data : {request.data}")
        self.bool = request.data

        if request.data:
            response.success = True
            response.message = "True response"
        else :
            response.success = False
            response.message = "False response"

        self.get_logger().info(response.message)
        return response

    def print_status(self):
        self.get_logger().info(f"current bool : {self.bool}")

def main():
    rclpy.init()
    node = Service_client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupted")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
