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
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action._follow_waypoints import FollowWaypoints_GetResult_Response

class Action_client(Node):
    def __init__(self):
        super().__init__("waypoints_client")
        self.create_timer(1, self.print_hello)
        self.action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.bool = bool()
        self.count = 0
        self.patrol_points = [[4.0, 0.0], [2.0, 1.0], [4.0, 3.0]]
        self.patrol_index = 0

        self.go_next()

    def go_next(self) : 
        self.send_goal(self.patrol_points[self.patrol_index][0], self.patrol_points[self.patrol_index][1])
        self.patrol_index += 1

    # 첫 서비스 실행 콜백 .. 
    def send_goal(self, x, y):
        self.get_logger().info(f"request goal : x:{x} y:{y}")

        while not self.action_client.wait_for_server():
            self.count += 1
            self.get_logger().info(f"server not availible : {self.count}")

        pose : PoseStamped = PoseStamped()    
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        goal = FollowWaypoints.Goal()
        goal.poses.append(pose)

        self.future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback )  
        self.future.add_done_callback(self.result_callback)
        
    
    def feedback_callback(self, msg):
        feedback : FollowWaypoints.Feedback = msg.feedback
        self.get_logger().info( f"Received feedback : {feedback.current_waypoint}")

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
            self.get_logger().info(f"get_result_callback result missed_waypoints : {result.result.missed_waypoints}")
            if self.patrol_index == self.patrol_points.count:
                self.patrol_index = 0
            else:
                self.go_next()
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"get_result_callback result canceled.. ")
            rclpy.shutdown()
        elif status == GoalStatus.STATUS_CANCELING:
            self.get_logger().info(f"get_result_callback result canceling.. ")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f"get_result_callback result aborted.. ")
            rclpy.shutdown()
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
        node.go_next()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
