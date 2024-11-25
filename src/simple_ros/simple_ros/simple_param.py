import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter 
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class Simple_param(Node):
    def __init__(self):
        super().__init__("simple_param")
        self.declare_parameter('my_param', '내가만든파라미터')
        self.declare_parameter('node_name', 'simple_param')
        self.declare_parameter('num1', 12)
        self.declare_parameter('num2', 3.1234)

        self.add_on_set_parameters_callback(self.set_parameters_callback)
        self.create_timer(1, self.print_param)
        self.my_param = self.get_parameter('my_param').get_parameter_value().string_value
        self.node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self.num1 = self.get_parameter('num1').get_parameter_value().integer_value
        self.num2 = self.get_parameter('num2').get_parameter_value().double_value
        self.number = 0

    def set_parameters_callback(self, params:list[Parameter]) :
        for param in params:
            if param.name == "my_param":
                self.my_param = param.get_parameter_value().string_value
            elif param.name == "node_name":
                # self.my_param2 = param.get_parameter_value().string_value
                self.node_name = param.value
            elif param.name == "num1":
                self.num1 = param.value
            elif param.name == "num2":
                self.num2 = param.value
        return SetParametersResult(successful=True)
    
    def print_param(self):
        self.get_logger().info(f'parameter my_param : {self.my_param}')
        self.get_logger().info(f'parameter node_name : {self.node_name}')
        sum = self.num1 + self.num2 
        self.get_logger().info(f'parameter num1({self.num1}) + num2({self.num2}) = {sum}')
        self.number += 1

def main():
    rclpy.init()
    node = Simple_param()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
