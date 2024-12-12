import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool 
import corobo_py.crb_db as crb_db 
from corobo_py.service_dao import ServiceDao 
from corobo_py.parm_dao import ParmDao 
from crb_interface.srv import CrbsCmdSrv
from crb_interface.srv import CrbmCenterSrv
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import time
import sys 
import traceback

# CoRoBo Mission Centor .. 
# MSN_ID : 1, 2 : arm_lift, arm_release... 

class CrbmCenter(Node):
    def __init__(self):
        super().__init__("crbm_center")
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.cur_msn_id = 0
        self.cur_msn_nm = ""
        self.cur_msn_desc = ""

        self.declare_parameter('server_type', 'main')
        self.server_type = self.get_parameter('server_type').get_parameter_value().string_value

        self.get_logger().info(f"CrbmCenter server_type : {self.server_type}  ")

        # 현재 미션 정보 publishing 
        self.create_timer(3, self.update_me)

        self.create_service(CrbmCenterSrv, "crbm_center", self.crbm_callback, callback_group=self.callback_group) 
        
        # create crbs_mani by called arm 
        self.cmd_client = self.create_client(CrbsCmdSrv, "crbs_m_server") 
        while not self.cmd_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service crbs_m_server is not available!!")

        self.cmd_req = CrbsCmdSrv.Request()

        self.conn = None 
        self.service_list = [] 

    # 최초 미션 서비스 실행 로직 .. 
    def crbm_callback(self, request:CrbmCenterSrv.Request, response:CrbmCenterSrv.Response):
        self.get_logger().info(f"crbm_callback request msn_id : {request.msn_id}")
        
        try:
            # DB 미션/서비스/파라메터 데이터 로딩 ... 
            self.init_mission(request.msn_id)

            for service_info in self.service_list:
                #self.cmd_req.cmd = "arm_joint"  
                self.get_logger().info(f"service_info request to crbs_server : {service_info.srv_call}")

                # 실행 시간에 따른 act_dur, act_step 세팅 .. 
                self.cmd_req.act_dur = 0.5 
                self.cmd_req.act_step = int(1 + 2*(service_info.srv_dur/1000))
                self.cmd_req.act_delay_factor = 0.6

                self.cmd_req.cmd = service_info.srv_call 

                # add service cmds here ... 
                if self.cmd_req.cmd == "arm_joint":
                    self.cmd_req.x = float(service_info.get_parm_val("joint1"))
                    self.cmd_req.y = float(service_info.get_parm_val("joint2"))
                    self.cmd_req.z = float(service_info.get_parm_val("joint3"))
                    self.cmd_req.w = float(service_info.get_parm_val("joint4"))
                elif self.cmd_req.cmd == "arm_gripper":
                    self.cmd_req.x = float(service_info.get_parm_val("gripper"))

                # pre delay duration.. 
                if service_info.pre_dur > 0 :
                    time.sleep(service_info.pre_dur/1000.0)

                # service call ... 
                self.cmd_req_future = self.cmd_client.call_async(self.cmd_req)
                self.cmd_req_future.add_done_callback(self.response_cmd_callback)

                # TODO : Check delay is necessary... 
                #time.sleep(service_info.srv_dur/1000.0)

                if service_info.aft_dur > 0 :
                    time.sleep(service_info.aft_dur/1000.0)

        except Exception as e:
            self.get_logger().error(f"Center Exception !! => {e.__doc__}")
            self.get_logger().error(traceback.format_exc())

            response.success = False
            response.message = f"Mission Failed Response {e.__doc__}"
            return response
        
        response.success = True
        response.message = "Mission Succeeded Response"

        self.get_logger().info(f"response => {response.message}")

        return response
    
    def response_cmd_callback(self, future):
        response : CrbsCmdSrv.Response = future.result()
        self.get_logger().info(f"response_cmd_callback success : {response.result}") 

    def init_mission(self, msn_id):
        self.cur_msn_id = msn_id 
        cursor = None 
        
        try:
            self.conn = crb_db.connect_db(self.get_logger()) 

            # 미션정보 가져오기
            sql = f"SELECT A.MSN_NM, A.MSN_DESC, A.MSN_TP, A.ROBO_CNT, A.ROBO_MAIN, A.ROBO_SUB1 FROM MSN_MST A WHERE A.MSN_ID = {msn_id}"
            cursor = crb_db.select_sql(self.conn, sql)  

            result = cursor.fetchall()
            for row in result:
                self.get_logger().info(f"MSN_NM : {row[0]} MSN_DESC : {row[1]} ")
                self.cur_msn_nm = row[0]
                self.cur_msn_desc = row[1]
                
            cursor.close()

            # 서비스정보 가져오기
            sql = ("SELECT A.SRV_SEQ, A.SRV_ID, B.SRV_NM, C.SRV_DESC, C.SRV_CALL, C.SRV_DUR, C.PRE_DUR, C.AFT_DUR, "
                "C.MAP_TP  FROM MSN_SRV_INFO A "
                "LEFT OUTER JOIN SRV_MST B ON (A.SRV_ID = B.SRV_ID) "
                "LEFT OUTER JOIN SRV_INFO C ON (A.SRV_SEQ = C.SRV_SEQ) "
                f"WHERE A.MSN_ID = {msn_id} ORDER BY A.SORTS ") 
            self.get_logger().info(f"SRV_SEQ : {row[0]} MSN_DESC : {row[1]} ")
            cursor = crb_db.select_sql(self.conn, sql)  

            self.service_list = [] 
            result = cursor.fetchall()
            for row in result:
                self.get_logger().info(f"SRV_SEQ : {row[0]} SRV_NM : {row[2]} ")
                svr_info = ServiceDao()
                svr_info.srv_seq = row[0]
                svr_info.srv_id = row[1]
                svr_info.srv_nm = row[2]
                svr_info.srv_desc = row[3]
                svr_info.srv_call = row[4]
                svr_info.srv_dur = row[5]
                svr_info.pre_dur = row[6]
                svr_info.aft_dur = row[7]
                svr_info.map_tp = row[8]

                self.service_list.append(svr_info)

            cursor.close()

            # 파라메터 정보 가져오기
            for svr in self.service_list:
                    
                sql = ("SELECT A.PARM_ID, A.PARM_DESC, A.PARM_VAL FROM PARM_INFO A "
                    f"WHERE A.SRV_SEQ = {svr.srv_seq} ORDER BY A.SORTS ") 
                cursor = crb_db.select_sql(self.conn, sql)  

                svr.parm = [] 
                result = cursor.fetchall()
                for row in result:
                    self.get_logger().info(f"parm_id : {row[0]} parm_val : {row[2]} ")
                    parm_info = ParmDao()
                    parm_info.srv_seq = svr.srv_seq
                    parm_info.parm_id = row[0]
                    parm_info.parm_desc = row[1]
                    parm_info.parm_val = row[2] 

                    svr.parm.append(parm_info)

                cursor.close()

        except Exception as e:
            crb_db.log(f"DB Exception !! => {e.__doc__}")
            crb_db.log(e.__doc__)
            crb_db.log(e.__traceback__)

            raise e

        finally:
            # 연결과 커서 닫기
            cursor.close()
            self.conn.close()


    def update_me(self):
        self.get_logger().info(f"current mission : {self.cur_msn_id} : {self.cur_msn_desc}")
        
def main(args=None):
    rclpy.init(args=args)
    # print(f"args {sys.argv[1]}")
    node = CrbmCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupted")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
