from asyncio import Future
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
import sys 
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus
from user_interface.action import Fibonacci

class Action_client(Node):
    def __init__(self):
        super().__init__("action_client")
        self.create_timer(1, self.print_hello)
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.bool = bool()
        self.count = 0

    # 첫 서비스 실행 콜백 .. 
    def send_goal(self, step):
        self.get_logger().info(f"request goal : {step}")

        while not self.action_client.wait_for_server():
            self.count += 1
            self.get_logger().info(f"server not availible : {self.count}")
        goal = Fibonacci.Goal()
        goal.req_step = int(step)
        self.future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback )  
        self.future.add_done_callback(self.result_callback)
        
    
    def feedback_callback(self, msg):
        seq = msg.feedback.feedback_seq
        self.get_logger().info(f"feedback_seq : {seq}")

    def result_callback(self, future:Future):
        goal_handle : ClientGoalHandle= future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"goal is not accepted")
            return; 

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future:Future):
        goal_handle = future.result()
        status = goal_handle.status
        self.get_logger().info(f"get_result_callback status : {status}")
        result = goal_handle.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"get_result_callback result seq : {result.result_seq}")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"get_result_callback result canceled : {result.result_seq}")
        elif status == GoalStatus.STATUS_CANCELING:
            self.get_logger().info(f"get_result_callback result canceling : {result.result_seq}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f"get_result_callback result aborted : {result.result_seq}")
        elif status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info(f"get_result_callback result accepted .. ")
        elif status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().info(f"get_result_callback result executing ....")


    def print_hello(self):
        #self.get_logger().info(f"current bool : {self.bool}")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Action_client()
    try:
        node.send_goal(sys.argv[1])
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
