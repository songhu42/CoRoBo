import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter 
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
# 모든 노드에서 기본으로 제공하는 서비스 임 
from rcl_interfaces.srv import SetParameters

class Simple_param2(Node):
    def __init__(self):
        super().__init__("simple_param2")
        self.client = self.create_client(SetParameters, "simple_param/set_parameters")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service server is not available..")
        self.count = 0
        self.declare_parameter("num1", 0)
        self.create_timer(1, self.print_param)

    def print_param(self):
        self.count += 1
        param = Parameter()
        param.name = "num1"
        param.value = ParameterValue() 
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = self.count

        req = SetParameters.Request()
        req.parameters = [param]
        future = self.client.call_async(req)
        future.add_done_callback(self.done_callback)


    def done_callback(self, future):
        response = future.result()
        self.get_logger().info(f'done_callback response : {response.results}')


def main():
    rclpy.init()
    node = Simple_param2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
