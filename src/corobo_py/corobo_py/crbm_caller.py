import rclpy
from rclpy.node import Node
from rclpy.node import Node
from crb_interface.srv import CrbsCmdSrv
from crb_interface.srv import CrbmCenterSrv
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import time
import sys 
import traceback

# CoRoBo Mission Centor .. 
# MSN_ID : 1, 2 : arm_lift, arm_release... 

class CrbmCaller(Node):
    def __init__(self, server_type):
        super().__init__("crbm_caller")
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.cur_msn_id = 0

        self.server_type = server_type 

        self.get_logger().info(f"CrbmCaller server_type : {self.server_type}  ")

        # 현재 미션 정보 publishing 
        #self.create_timer(3, self.update_me)

        self.create_service(CrbmCenterSrv, "crbm_caller", self.crbm_caller_callback, callback_group=self.callback_group) 
        
        # create crbs_mani by called arm 
        self.cmd_client = self.create_client(CrbmCenterSrv, "crbm_center") 

        while not self.cmd_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service crbm_center is not available!!")


    # 최초 미션 서비스 실행 로직 .. 
    def crbm_caller_callback(self, request:CrbmCenterSrv.Request, response:CrbmCenterSrv.Response):
        self.get_logger().info(f"crbm_caller_callback request msn_id : {request.msn_id}")
        
        # service call ... 
        self.cmd_client.call_async(request) 

        response.success = True
        response.message = "Mission Succeeded Response"

        self.get_logger().info(f"response => {response.message}")

        return response
    
def main(args=None):
    rclpy.init(args=args)
    server_type="main"

    if len(sys.argv) > 2:
        server_type=sys.argv[2]
    node = CrbmCaller(server_type)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupted")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
