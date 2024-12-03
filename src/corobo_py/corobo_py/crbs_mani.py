import rclpy
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.clock import Duration
from rclpy.node import Node
from rclpy.task import Future
from crb_interface.srv import CrbsArmSrv


class CrbsMani(Node):
    def __init__(self):
        super().__init__("crbs_mani")
        
        self.create_service(CrbsArmSrv, "crbs_arm_service", self.crbs_mani_callback) 

        self.joint_client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.tool_client = self.create_client(SetJointPosition, 'goal_tool_control')

        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("joint_client service not available")
        while not self.tool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("tool_client service not available")

        self.joint_req = SetJointPosition.Request()
        self.tool_req = SetJointPosition.Request() 

        # self.create_timer(1/20, self.update)
        self.joint_angles = [0.0, 0.0, -1.5, 0.0, 0.0] 

        self.prev_time = self.get_clock().now()
        self.stage = 0

    def crbs_mani_callback(self, request:CrbsArmSrv.Request, response:CrbsArmSrv.Response):
        self.get_logger().info(f"crbs_mani_callback gripper : {request.is_gripper}")
        self.get_logger().info(f"crbs_mani_callback : {request}")
        self.joint_angles[0] = request.joint1
        self.joint_angles[1] = request.joint2
        self.joint_angles[2] = request.joint3
        self.joint_angles[3] = request.joint4
        self.joint_angles[4] = request.gripper

        if request.is_gripper:
            response.message = "gripper moving service completed"
            self.send_tool_request(request.path_time)
        else:
            response.message = "joint moving service completed"
            self.send_joint_request(request.path_time)

        response.success = True

        return response
        

    def send_joint_request(self, path_time=0.5):
        self.joint_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.joint_req.joint_position.position = [self.joint_angles[0], self.joint_angles[1], self.joint_angles[2], self.joint_angles[3]]
        #self.joint_req.joint_position.position = self.joint_angles
        self.joint_req.path_time = path_time

        try:
            send_goal_joint = self.joint_client.call_async(self.joint_req)
        except Exception as e:
            self.get_logger().info('send_joint_request failed %r' % (e,))

        self.joint_future = self.joint_client.call_async(self.joint_req)
        self.joint_future.add_done_callback(self.done_joint_callback)
        

    def send_tool_request(self, path_time=0.5):
        self.tool_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_req.joint_position.position = [self.joint_angles[0], self.joint_angles[1], self.joint_angles[2], self.joint_angles[3], self.joint_angles[4]]
        self.tool_req.path_time = path_time

        try:
            self.tool_future = self.tool_client.call_async(self.tool_req)
            self.tool_future.add_done_callback(self.done_tool_callback)
            self.get_logger().info(f"send_tool_request : {self.joint_angles}")
        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))


    def done_joint_callback(self, future : Future):
        response : SetJointPosition.Response = future.result()
        self.get_logger().info(f"done_joint_callback : {response.is_planned}")

    def done_tool_callback(self, future : Future):
        response : SetJointPosition.Response = future.result()
        self.get_logger().info(f"done_tool_callback : {response.is_planned}")

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
