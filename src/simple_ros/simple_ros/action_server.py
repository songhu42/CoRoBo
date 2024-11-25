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
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from user_interface.action import Fibonacci

class Action_server(Node):
    def __init__(self):
        super().__init__("action_server")
        self.create_timer(1, self.print_hello)
        self.action_server = ActionServer(self, Fibonacci, "fibonacci", self.action_callback)
        self.bool = bool()
        self.count = 0

    # 첫 서비스 실행 콜백 .. 
    def action_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info(f"request data : {goal_handle.request.req_step}")
        self.count += 1
        goal = Fibonacci.Goal()
        result = Fibonacci.Result()
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.feedback_seq = [0, 1]

        req_step = goal_handle.request.req_step
        if req_step <= 0:
            self.get_logger().info(f"req_step is less than 1 : {req_step}")
            # when server error will be return STATUS_ABORTED
            goal_handle.abort()
            result.result_seq = feedback_msg.feedback_seq
            return result
        else:
            for i in range(1, req_step+1):
                sum = feedback_msg.feedback_seq[i] + feedback_msg.feedback_seq[i-1]
                feedback_msg.feedback_seq.append(sum)
                # send feedback msg 
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

            goal_handle.succeed()
            result.result_seq = feedback_msg.feedback_seq
            return result

    def print_hello(self):
        #self.get_logger().info(f"current bool : {self.bool}")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Action_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
