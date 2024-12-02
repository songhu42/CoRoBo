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
from crb_interface.msg import CrbsCmdMsg
from geometry_msgs import Vector3

class CrbsServer(Node):
    def __init__(self, node1):
        self.mani_node = node1
        self.cmd = None
        self.req = CrbsCmdMsg()
        self.target_pos : Vector3 = Vector3() 

        super().__init__("crbs_server")
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth=10)
        # set slow for test .. 0.1 => 1.0, 1/20 => 0.1 
        self.create_timer(1.0, self.twist_pub)
        self.create_timer(0.1, self.update)
        
        self.create_service(CrbsCmdSrv, "crbs_m_server", self.crbs_cmd_callback) 

        self.pub = self.create_publisher(Twist, "cmd_vel", self.qos_profile)
        self.create_subscription(Pose, "pose", self.pose_callback, self.qos_profile)

        self.twist = Twist()
        self.pose = Pose()

        self.phase = 0
        self.prev_time = self.get_clock().now()

        # db test 
        self.get_logger().info(f"test_db start in node...")
        crb_db.test_db(self.get_logger())


    def crbs_cmd_callback(self, request:CrbsCmdSrv.Request, response:CrbsCmdSrv.Response):
        self.get_logger().info(f"crbs_cmd_callback data : {request.data}")
        self.req = request.data
        self.cmd = self.req.cmd

        self.count += 1

        # 요청 데이터 가공이 필요한 경우 수행한다. 
        # TODO:정해진 목표점으로 이동한다. 목표점에 tf 발행 후 tf를 향한 이동 구현 
        if self.cmd == "moveto":
            self.target_pos.x = self.req.x
            self.target_pos.y = self.req.y
            
            response.success = True
            response.message = "True response"
        # 단순히 지정된 방향/속도로 이동한다. 
        elif self.cmd == "move":
            self.target_pos.x = self.req.x
            self.target_pos.y = self.req.y
            
            response.success = True
            response.message = "True response"
        # TODO:정해진 목표점으로 이동한다. 목표점에 tf 발행 후 tf를 향한 이동 구현 
        elif self.cmd == "armto":
            self.target_pos.x = self.req.x
            self.target_pos.y = self.req.y
            self.target_pos.z = self.req.z
            
            response.success = True
            response.message = "True response"
        # 단순히 지정된 각도로 로봇팔을 움직인다. 
        elif self.cmd == "arm":
            
            self.mani_node.joint_angles = [self.req.x, self.req.y, self.req.z, 0.0, 1.0]
            response.success = True
            response.message = "True response"
        else :
            response.success = False
            response.message = "False response"

        self.get_logger().info(f"response {self.count} => {response.message}")

        return response
    
    # self.twist update 후 cmd_vel 발행 
    def twist_pub(self):
        self.pub.publish(self.twist)
        self.get_logger().info(f"twist_pub published...")

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
        elif self.cmd == "arm":
            # crb_mani.update_mani()

def main():
    rclpy.init()
    node2 = crbs_mani.CrbMani()
    node = CrbsServer(node2)
    
    try:
        while True:
            rclpy.spin_once(node2)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
