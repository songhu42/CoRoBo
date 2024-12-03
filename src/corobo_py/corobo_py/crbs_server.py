import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from turtlesim.msg import Color, Pose
import corobo_py.crb_db as crb_db 
import corobo_py.crbs_mani as crbs_mani 
from crb_interface.srv import CrbsCmdSrv
from crb_interface.srv import CrbsArmSrv
from crb_interface.msg import CrbsCmdMsg
from geometry_msgs.msg import Vector3
# callback_group : 같은 함수를 여러번 호출하는 경우에만 적용됨..
from rclpy.callback_groups import ReentrantCallbackGroup
# 순차적으로 실행해야 할때.. 
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import asyncio
import time 

class CrbsServer(Node):
    def __init__(self, node1=None):
        self.mani_node = node1
        self.cmd = None
        self.req = CrbsCmdMsg()
        self.target_pos = Vector3() 
        self.callback_group = ReentrantCallbackGroup()
        
        # TODO : read from DB for base position 선자세 각도 기준 
        self.base_angles = [0.0, 0.0, -1.5, 0.0, 0.0]
        self.joint_angles = [0.0, 0.0, -1.5, 0.0, 0.0]
        self.prev_angles = [0.0, 0.0, -1.5, 0.0, 0.0]

        super().__init__("crbs_server")
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth=10)
        # set slow for test .. 0.1 => 1.0, 1/20 => 0.1 
        self.create_timer(1.0, self.twist_pub)
        self.create_timer(0.1, self.update)
        
        self.create_service(CrbsCmdSrv, "crbs_m_server", self.crbs_cmd_callback, callback_group=self.callback_group) 

        # create crbs_mani by called arm 
        self.arm_client = self.create_client(CrbsArmSrv, "crbs_arm_service") 
        while not self.arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service crbs_arm_service is not available!!")

        self.arm_req = CrbsArmSrv.Request()

        self.pub = self.create_publisher(Twist, "cmd_vel", self.qos_profile)
        self.create_subscription(Pose, "pose", self.pose_callback, self.qos_profile)

        self.twist = Twist()
        self.pose = Pose()

        self.count = 0
        self.phase = 0
        self.prev_time = self.get_clock().now()

        self.act_dur = 0.5
        self.act_step = 8
        self.act_delay_factor=0.9 

        # db parameters loading ...  
        self.get_logger().info(f"test_db start in node..33.")
        crb_db.test_db(self.get_logger())
        self.get_logger().info(f"test_db end in node...")


    def crbs_cmd_callback(self, request:CrbsCmdSrv.Request, response:CrbsCmdSrv.Response):
        self.get_logger().info(f"crbs_cmd_callback ")
        self.get_logger().info(f"crbs_cmd_callback data : {request.cmd}")
        self.req = request
        self.cmd = self.req.cmd

        self.count += 1
        try:
            # 요청 데이터 가공이 필요한 경우 수행한다. 
            # TODO:정해진 목표점으로 이동한다. 목표점에 tf 발행 후 tf를 향한 이동 구현 
            if self.cmd == "moveto":
                self.target_pos.x = self.req.x
                self.target_pos.y = self.req.y
                
                response.result = "Success"
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
            # 단순히 지정된 방향/속도로 이동한다. 
            elif self.cmd == "move":
                self.target_pos.x = self.req.x
                self.target_pos.y = self.req.y
                
                response.result = "Success"
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
            # TODO:정해진 목표점으로 이동한다. 목표점에 tf 발행 후 tf를 향한 이동 구현 
            elif self.cmd == "armto":
                self.target_pos.x = self.req.x
                self.target_pos.y = self.req.y
                self.target_pos.z = self.req.z
                
                response.result = "Success"
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
            # 단순히 지정된 각도로 로봇팔을 움직인다. 
            elif self.cmd == "arm":
                #[0, 0, -1.5, 0]
                # joint1 + : left, joint2,3,4 + : down 
                # test position1 [0.1, 0.6, 0.9, -1.27]
                # => 0.095, 0.69, 0.65, -1.25 
                # => 0.095, 0.5, 0.63, -1.25 
                # => 0.095, 0.11, 0.47, -0.68 
                # => 0.095, 0.01, 0.06, -0.04 

                # TODO: read parameter from db 
                self.act_dur = 0.5
                self.act_step = 8
                self.act_delay_factor = 0.8 

                # TODO : remove for test 
                self.prev_angle = [0.092, 0.0153, -0.0598, -0.0414, -0.005] # origin
                #self.prev_angle = [0.0, 0.0, -1.5, 0.0, 0.002] # 위쪽 
                #self.prev_angle = [1.0, 1.0, -1.5, -0.5, 0.002] # 비틀기 

                #req_joint_pos = [0.092, 0.0153, -0.0598, -0.0414, 0.0] # origin
                #self.prev_angle = [0.0, 0.0, -1.5, 0.0, 0.002] # 위쪽 
                req_joint_pos = [1.0, 1.0, -1.5, -0.5, self.req.x] # 비틀기 

                # self.req_move_joint(req_joint_pos)
                self.req_move_gripper(req_joint_pos)
            
                # self.mani_node.joint_angles = [self.req.x, self.req.y, self.req.z, 0.0, 1.0]
                response.result = "Success"
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
            else :
                response.result = "Wrong Cmd"
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0

        except Exception as e:
            self.get_logger().info(f"CrbsCmd Exception !! => {e.__doc__}")
            self.get_logger().info(e.__doc__)
            self.get_logger().info(e.__traceback__)
            # self.get_logger().info('Tool control failed %r' % (e,))
            response.result = "Exception occured!!!!"
        finally:
            return response

    
    def req_move_joint(self, req_joint_pos):
        # rate = self.create_rate(1.0/act_dur)
        self.get_logger().info(f"req_move_joint called {req_joint_pos} step : {self.act_dur}")
        for step in range(0, self.act_step): 
            # calculate self.joint_anges step by step .... 
            self.cal_setp_joint_angle(req_joint_pos, self.act_step, step)
            self.get_logger().info(f"joint request called {self.joint_angles} step : {step}")
            
            self.arm_req.is_gripper = False
            self.arm_req.joint1 = self.joint_angles[0]
            self.arm_req.joint2 = self.joint_angles[1]
            self.arm_req.joint3 = self.joint_angles[2]
            self.arm_req.joint4 = self.joint_angles[3]
            self.arm_req.gripper = self.joint_angles[4]
            self.arm_req.path_time = self.act_dur

            self.arm_req_future = self.arm_client.call_async(self.arm_req)
            self.arm_req_future.add_done_callback(self.response_arm_callback)

            self.prev_angle = self.joint_angles
            # 전체 이동시간이 끝나기 전에 다음 이동이 있는 경우 호출되도록 90% delay 적용.. 
            time.sleep(self.act_dur*self.act_delay_factor)

            #await asyncio.sleep(2)  


    def req_move_gripper(self, req_joint_pos):
        # rate = self.create_rate(1.0/act_dur)
        self.get_logger().info(f"req_move_joint called {req_joint_pos} step : {self.act_dur}")
        self.joint_angles[4] = req_joint_pos[4]
        self.arm_req.is_gripper = True
        self.arm_req.joint1 = 0.0
        self.arm_req.joint2 = 0.0
        self.arm_req.joint3 = 0.0
        self.arm_req.joint4 = 0.0
        # range -0.009(grip) ~ 0.01  
        self.arm_req.gripper = self.joint_angles[4]
        self.arm_req.path_time = self.act_dur

        self.arm_req_future = self.arm_client.call_async(self.arm_req)
        self.arm_req_future.add_done_callback(self.response_arm_callback)

        self.prev_angle[4] = self.joint_angles[4]


    # after arm service callback ... 
    def response_arm_callback(self, future):
        response : CrbsArmSrv.Response = future.result()
        self.get_logger().info(f"response_arm_callback success : {response.success}")
    
    # 이동할 position과 이전 positon으로 step별 위치를 계산한다. 
    def cal_setp_joint_angle(self, req_joint_pos, act_step, step):
        div_step = act_step - step
        for i in range(0, 5):
            self.joint_angles[i] = self.prev_angle[i] + (req_joint_pos[i]-self.prev_angle[i])/div_step

    # self.twist update 후 cmd_vel 발행 
    def twist_pub(self):
        self.pub.publish(self.twist)
        # self.get_logger().info(f"twist_pub published...")

    def pose_callback(self, msg: Pose):
        self.pose = msg
        self.get_logger().info(f"twpose_callback called...")

    def update(self):
        """ self.twist, self.pose, self.color 을 이용한 알고리즘"""
        if self.cmd == "moveto":
            if self.phase == 0:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 2.0
                if (self.get_clock().now() - self.prev_time) > Duration(seconds=1, nanoseconds=250_000_000):
                    self.prev_time = self.get_clock().now()
                    self.phase = 1
            elif self.phase == 1:
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0
                if (self.get_clock().now() - self.prev_time) > Duration(seconds=2):
                    self.prev_time = self.get_clock().now()
                    self.phase = 0
        #elif self.cmd == "arm":
        #    

def main():
    rclpy.init()
    #node2 = crbs_mani.CrbsMani()
    #node = CrbsServer(node2)
    node = CrbsServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
