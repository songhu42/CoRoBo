import rclpy
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.clock import Duration
from rclpy.node import Node
from rclpy.task import Future


class CrbsMani(Node):
    def __init__(self):
        super().__init__("crbs_mani")
        
        self.joint_client = self.create_client(SetJointPosition, "goal_joint_space_path")
        self.task_client = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.posi_req = SetJointPosition.Request()
        self.task_req = SetKinematicsPose.Request()

        self.create_timer(1/20, self.update)
        self.joint_angles = self.posi_req.joint_position.position 
        self.goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.prev_time = self.get_clock().now()
        self.stage = 0

    def send_joint_request(self, path_time=0.5):
        self.posi_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.posi_req.joint_position.position = self.joint_angles
        self.posi_req.path_time = path_time

        self.joint_future = self.joint_client.call_async(self.posi_req)
        self.joint_future.add_done_callback(self.done_joint_callback)
        self.get_logger().info(f"send_joint_request : {self.posi_req.joint_position.position}")


    def send_task_request(self, path_time=0.5):
        self.task_req.end_effector_name = 'gripper'
        self.task_req.kinematics_pose.pose.position.x = self.goal_kinematics_pose[0]
        self.task_req.kinematics_pose.pose.position.y = self.goal_kinematics_pose[1]
        self.task_req.kinematics_pose.pose.position.z = self.goal_kinematics_pose[2]
        self.task_req.kinematics_pose.pose.orientation.w = self.goal_kinematics_pose[3]
        self.task_req.kinematics_pose.pose.orientation.x = self.goal_kinematics_pose[4]
        self.task_req.kinematics_pose.pose.orientation.y = self.goal_kinematics_pose[5]
        self.task_req.kinematics_pose.pose.orientation.z = self.goal_kinematics_pose[6]
        self.task_req.path_time = path_time

        self.task_future = self.joint_client.call_async(self.posi_req)
        self.task_future.add_done_callback(self.done_task_callback)
        self.get_logger().info(f"send_task_request : {self.task_req.kinematics_pose.pose.position}")


        try:
            send_goal_task = self.task_client.call_async(self.task_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))


    def done_joint_callback(self, future : Future):
        response : SetJointPosition.Response = future.result()
        self.get_logger().info(f"done_joint_callback : {response.is_planned}")

    def done_task_callback(self, future : Future):
        response : SetKinematicsPose.Response = future.result()
        self.get_logger().info(f"done_task_callback : {response.is_planned}")

    def update(self):
        if self.prev_time + Duration(seconds=3) < self.get_clock().now():
            # first move
            if self.stage == 0:
                self.joint_angles = [0.5, 0.0, 0.3, 0.0, 1.0]
                self.get_logger().info(f"stage 0 called")
                self.send_joint_request(2.0)
                self.goal_kinematics_pose = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                #self.send_task_request(0.3)
                self.stage = 1
                self.prev_time = self.get_clock().now()
            elif self.stage == 1:
                self.joint_angles = [0.0, 0.5, 0.1, 0.5, 1.0]
                self.get_logger().info(f"stage 1 called")
                self.send_joint_request(2.0)
                self.goal_kinematics_pose = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                #self.send_task_request(0.3)
                self.stage = 0
                self.prev_time = self.get_clock().now()


def main():
    rclpy.init()
    node = CrbsMani()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupted")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
