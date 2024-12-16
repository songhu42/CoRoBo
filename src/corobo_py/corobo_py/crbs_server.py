import sys
import rclpy
import math 
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import tf2_ros
import tf_transformations
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry

from turtlesim.msg import Color, Pose
import corobo_py.crb_db as crb_db 
from crb_interface.srv import CrbsCmdSrv
from crb_interface.srv import CrbsArmSrv
from crb_interface.msg import CrbsCmdMsg
from geometry_msgs.msg import Vector3
# callback_group : 같은 함수를 여러번 호출하는 경우에만 적용됨..
from rclpy.callback_groups import ReentrantCallbackGroup
# 순차적으로 실행해야 할때.. 
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time 
import traceback
from rclpy.qos import qos_profile_sensor_data
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, Time

MAX_VEL = 0.21
MAX_ANGLE = 2.8 # radian/sec

class CrbsServer(Node):
    def __init__(self, server_type): 
        self.cmd = None
        self.req = CrbsCmdMsg()
        self.target_pos = Vector3() 
        self.callback_group = ReentrantCallbackGroup()
        super().__init__("crbs_server") 

        self.qos_profile = qos_profile_sensor_data
        self.server_type = server_type
        self.get_logger().info(f"CrbsServer server_type : {self.server_type}  ")

        # TODO : read from DB for base position 선자세 각도 기준 
        self.base_angles = [0.0, 0.0, -1.5, 0.0, 0.0]
        self.joint_angles = [0.0, 0.0, -1.5, 0.0, 0.0]
        self.prev_angles = [0.0, 0.0, 0.0, 0.0, 0.0]

        # set slow for test .. 0.1 => 1.0, 1/20 => 0.1 
        self.create_timer(0.2, self.twist_pub)
        self.create_timer(0.1, self.update)
        #self.create_timer(0.1, self.twist_pub)
        #self.create_timer(1/60, self.update)

        self.create_service(CrbsCmdSrv, "crbs_m_server", self.crbs_cmd_callback, callback_group=self.callback_group) 

        # create crbs_mani by called arm 
        self.arm_client = self.create_client(CrbsArmSrv, "crbs_arm_service") 
        while not self.arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service crbs_arm_service is not available!!")

        self.arm_req = CrbsArmSrv.Request()

        self.pub = self.create_publisher(Twist, "cmd_vel", self.qos_profile)
        self.create_subscription(Pose, "pose", self.pose_callback, self.qos_profile)
        self.create_subscription(ArucoMarkers, "/aruco_markers", self.aruco_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10) 
        
        self.aruco_markers = ArucoMarkers()
        self.twist = Twist()
        self.pose = Pose()
        self.follow_tf = Pose()
        self.follow_tf2 = TransformStamped()
        self.theta = 0.0 # raian
        self.phase = 0
        self.laserscan_degree = [3.5 for i in range(360)]
        self.find_wall = False
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_time = self.get_clock().now()

        # reading from request .. 
        self.act_dur = 0.5
        self.act_step = 8
        self.act_delay_factor=0.9 

        # db parameters loading ...  
        self.conn = None
        self.cursor = None 

    def crbs_cmd_callback(self, request:CrbsCmdSrv.Request, response:CrbsCmdSrv.Response):
        self.get_logger().info(f"crbs_cmd_callback ")
        self.get_logger().info(f"crbs_cmd_callback cmd : {request.cmd}")
        self.req = request
        self.cmd = self.req.cmd

        try:                
            # 요청 데이터 가공이 필요한 경우 수행한다. 
            # 단순히 지정된 방향/속도로 이동한다. 
            if self.cmd == "move":
                self.target_pos.x = self.req.x
                self.target_pos.y = self.req.y
                self.target_pos.z = self.req.z # 목적지 theta 
                
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
            elif self.cmd == "arm_joint":
                #[0, 0, -1.5, 0]
                # joint1 + : left, joint2,3,4 + : down 
                # test position1 [0.1, 0.6, 0.9, -1.27]
                # => 0.095, 0.69, 0.65, -1.25 
                # => 0.095, 0.5, 0.63, -1.25 
                # => 0.095, 0.11, 0.47, -0.68 
                # => 0.095, 0.01, 0.06, -0.04 
                #self.act_dur = 0.5
                #self.act_step = 8
                #self.act_delay_factor = 0.8 

                # set parameter from req  
                self.act_dur = self.req.act_dur 
                self.act_step = self.req.act_step 
                self.act_delay_factor = self.req.act_delay_factor  

                # TODO : remove for test 
                #self.prev_angles = [0.092, 0.0153, -0.0598, -0.0414, -0.005] # origin
                #self.prev_angles = [0.0, 0.0, -1.5, 0.0, 0.002] # 위쪽 
                #self.prev_angles = [1.0, 1.0, -1.5, -0.5, 0.002] # 비틀기 

                #req_joint_pos = [0.092, 0.0153, -0.0598, -0.0414, 0.0] # origin
                #self.prev_angles = [0.0, 0.0, -1.5, 0.0, 0.002] # 위쪽 
                #req_joint_pos = [1.0, 1.0, -1.5, -0.5, self.req.x] # 비틀기 

                req_joint_pos = [self.req.x, self.req.y, self.req.z, self.req.w, self.prev_angles[4]] # 비틀기 

                # prev values init setting ... 
                self.init_prev_angles(req_joint_pos)
                self.req_move_joint(req_joint_pos)

                response.success = True 
                response.result = "Success req_move_joint"
                response.x = self.prev_angles[0]
                response.y = self.prev_angles[1]
                response.z = self.prev_angles[2]
                response.w = self.prev_angles[3]

            elif self.cmd == "arm_gripper":
                # set parameter from req  
                self.act_dur = self.req.act_dur 
                self.act_step = self.req.act_step 
                self.act_delay_factor = self.req.act_delay_factor  

                req_joint_pos = [self.prev_angles[0], self.prev_angles[1], self.prev_angles[2], self.prev_angles[3], self.req.x] # 비틀기 
                self.req_move_gripper(req_joint_pos)

                response.success = True 
                response.result = "Success req_move_joint"
                response.x = self.prev_angles[4]
                response.y = 0.0
                response.z = 0.0
                response.w = 0.0

            # TODO:정해진 목표점으로 이동한다. 목표점에 tf 발행 후 tf를 향한 이동 구현 
            elif self.cmd == "robo_moveto":
                # set parameter from req  
                self.act_dur = self.req.act_dur 
                self.act_step = self.req.act_step 
                self.act_delay_factor = self.req.act_delay_factor  

                self.target_pos.x = self.req.x
                self.target_pos.y = self.req.y
                self.target_pos.z = self.req.z

                self.follow_tf.position.x = self.req.x 
                self.follow_tf.position.y = self.req.y 
                self.follow_tf.position.z = self.twist.linear.z 
                self.follow_tf.orientation.x = self.twist.angular.x 
                self.follow_tf.orientation.z = self.twist.angular.y 
                self.follow_tf.orientation.z = self.req.z
                self.follow_tf.orientation.w = self.twist.angular.w 
                
                # 목적지 tf 발행.. 
                self.aruco_tf_publish_function()

                response.success = True 
                response.result = "Success robo_moveto"
                response.x = self.prev_angles[4]
                response.y = 0.0
                response.z = 0.0
                response.w = 0.0
            else :
                response.result = "Wrong Cmd"
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
                response.w = 0.0

            response.success = True 

        except Exception as e:
            self.get_logger().error(f"CrbsCmd Exception !! => {e.__doc__}")
            self.get_logger().error(traceback.format_exc())
            response.success = False 
            response.result = f"Exception occured!!!! => {e.__doc__}"
        finally:
            return response

    def init_prev_angles(self, angles):
        if self.prev_angles[0] == 0.0 and self.prev_angles[1] == 0.0 and self.prev_angles[2] == 0.0 and self.prev_angles[3] == 0.0 :
            for i in range(5):
                self.prev_angles[i] = angles[i]

    def set_prev_angles(self, angles):
        for i in range(5):
            self.prev_angles[i] = angles[i]


    def req_move_joint(self, req_joint_pos):
        # rate = self.create_rate(1.0/act_dur)
        self.get_logger().info(f"req_move_joint called {req_joint_pos} step : {self.act_step} dur : {self.act_dur}")
        
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

            self.set_prev_angles(self.joint_angles)
            # 전체 이동시간이 끝나기 전에 다음 이동이 있는 경우 호출되도록 90% delay 적용.. 
            time.sleep(self.act_dur*self.act_delay_factor)

            #await asyncio.sleep(2)  



    def req_move_gripper(self, req_joint_pos):
        # rate = self.create_rate(1.0/act_dur)
        self.get_logger().info(f"req_move_gripper called : {req_joint_pos} step : {self.act_step} dur : {self.act_dur}")
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

        self.prev_angles[4] = self.joint_angles[4]


    # after arm service callback ... 
    def response_arm_callback(self, future):
        response : CrbsArmSrv.Response = future.result()
        self.get_logger().info(f"response_arm_callback success : {response.success}")
    
    # 이동할 position과 이전 positon으로 step별 위치를 계산한다. 
    def cal_setp_joint_angle(self, req_joint_pos, act_step, step):
        div_step = act_step - step
        for i in range(0, 5):
            self.joint_angles[i] = self.prev_angles[i] + (req_joint_pos[i]-self.prev_angles[i])/div_step

    def odom_callback(self, msg: Odometry):
        self.odom = msg
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.theta = tf_transformations.euler_from_quaternion((x, y, z, w))

    

    # self.twist update 후 cmd_vel 발행 
    def twist_pub(self):
        self.restrain()
        self.pub.publish(self.twist)

    def aruco_callback(self, msg: ArucoMarkers):
        self.aruco_markers = msg
        for marker_id_ele in msg.marker_ids:
            if marker_id_ele == 1:
                self.follow_tf = msg.poses[msg.marker_ids.index(marker_id_ele)]
        self.aruco_tf_publish_function()

    def aruco_tf_publish_function(self):
        # TODO : follow_tf 획득 여부 체크 필요.. 
        # tf2로 구현
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link_optical"
        t.child_frame_id = "follow_point"
        t.transform.translation.x = self.follow_tf.position.x
        t.transform.translation.y = self.follow_tf.position.y
        t.transform.translation.z = self.follow_tf.position.z
        t.transform.rotation.x = self.follow_tf.orientation.x
        t.transform.rotation.y = self.follow_tf.orientation.y
        t.transform.rotation.z = self.follow_tf.orientation.z
        t.transform.rotation.w = self.follow_tf.orientation.w
        self.tf_broadcaster.sendTransform(t)

    def pose_callback(self, msg: Pose):
        self.pose = msg
        self.get_logger().info(f"twpose_callback called...")

    def update(self):
        """ self.twist, self.pose, self.color 을 이용한 알고리즘"""
        if self.cmd == "robo_moveto":
            buffer = Buffer()
            self.tf_listener = TransformListener(buffer, self)
            try:
                self.follow_tf2 = buffer.lookup_transform("camera_link", "follow_point", Time())
                self.get_logger().info(f"follow_tf : {self.follow_tf2}")
                self.twist.angular.z = math.atan2(
                    self.follow_tf2.transform.translation.y,
                    self.follow_tf2.transform.translation.x)
                self.twist.linear.x = math.sqrt(
                    self.follow_tf2.transform.translation.x**2 +
                    self.follow_tf2.transform.translation.y**2)
            except Exception as e:
                self.get_logger().info(f"Exception : {e}")
                self.twist.angular.z = math.atan2(
                    self.follow_tf2.transform.translation.y,
                    self.follow_tf2.transform.translation.x)
                self.twist.linear.x = math.sqrt(
                    self.follow_tf2.transform.translation.x**2 +
                    self.follow_tf2.transform.translation.y**2)
                
        elif self.cmd == "move":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0
            if (self.get_clock().now() - self.prev_time) > Duration(seconds=1, nanoseconds=250_000_000):
                self.prev_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)

    server_type="main"

    if len(sys.argv) > 2:
        server_type=sys.argv[2]

    node = CrbsServer(server_type)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
