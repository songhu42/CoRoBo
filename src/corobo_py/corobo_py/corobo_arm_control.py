import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # 액션 클라이언트 초기화
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # 팔 초기 위치 (1단계 포지션)
        self.arm_initial_positions = [0.0, -1.5785, 1.2241, 0.2654]

    def send_gripper_command(self, position, max_effort=50.0):
        self.get_logger().info(f"그립퍼 명령 전송: position={position}, max_effort={max_effort}")
        
        # 그립퍼 명령 생성
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        # 목표 전송
        self.gripper_client.wait_for_server()
        self.get_logger().info("그립퍼 명령이 성공적으로 전송되었습니다.")
        self.gripper_client.send_goal_async(goal_msg)

    def send_arm_command(self, joint_positions, time_from_start_sec=1):
        self.get_logger().info(f"팔 명령 전송: joint_positions={joint_positions}, time_from_start_sec={time_from_start_sec}")
        
        # 팔 명령 생성
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = time_from_start_sec
        trajectory_msg.points = [point]

        # FollowJointTrajectory.Goal 메시지로 감싸기
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        # 목표 전송
        self.arm_client.wait_for_server()
        self.get_logger().info("팔 명령이 성공적으로 전송되었습니다.")
        self.arm_client.send_goal_async(goal_msg)

    def execute_steps(self):
        # 1단계
        self.get_logger().info("\n==================== 1단계 ====================")
        self.get_logger().info("실행 중: 그립퍼 열기 및 팔 초기 위치 설정...")
        self.send_gripper_command(0.0)  # 그립퍼 열기
        time.sleep(1)  # 1초 대기
        self.send_arm_command(self.arm_initial_positions)  # 팔 초기 위치로 설정
        time.sleep(1)  # 1초 대기
        self.get_logger().info("1단계 완료: 그립퍼 열기 및 팔 초기 위치 설정.")
        
        # 2단계
        self.get_logger().info("\n==================== 2단계 ====================")
        self.get_logger().info("실행 중: 그립퍼 완전히 열기 및 팔 목표 위치로 이동...")
        self.send_gripper_command(0.023)  # 그립퍼 열기
        time.sleep(1.5)  # 1.5초 대기
        self.send_arm_command([1.5, 0.0, 0.0, 0.0])  # 팔 위치 설정
        time.sleep(1.5)  # 1.5초 대기
        self.get_logger().info("2단계 완료: 그립퍼 완전히 열기 및 팔 목표 위치로 이동.")
        
        # 3단계
        self.get_logger().info("\n==================== 3단계 ====================")
        self.get_logger().info("실행 중: 팔 목표 위치 2로 이동...")
        self.send_arm_command([1.5, 1.4, 0.1, -1.5])  # 팔 위치 설정
        time.sleep(1.5)  # 1.5초 대기
        self.get_logger().info("3단계 완료: 팔 목표 위치 2로 이동.")
        
        # 4단계
        self.get_logger().info("\n==================== 4단계 ====================")
        self.get_logger().info("실행 중: 팔 목표 위치 3으로 이동...")
        self.send_arm_command([1.5, 1.6, -0.8, -0.9])  # 팔 위치 설정
        time.sleep(1.5)  # 1.5초 대기
        self.get_logger().info("4단계 완료: 팔 목표 위치 3으로 이동.")
        
        # 5단계
        self.get_logger().info("\n==================== 5단계 ====================")
        self.get_logger().info("실행 중: 그립퍼 닫기...")
        self.send_gripper_command(-0.014)  # 그립퍼 닫기
        time.sleep(1.5)  # 1.5초 대기
        self.get_logger().info("5단계 완료: 그립퍼 닫기.")
        
        # 6단계
        self.get_logger().info("\n==================== 6단계 ====================")
        self.get_logger().info("실행 중: 팔 픽업 위치로 이동...")
        self.send_arm_command([1.5, 0.5, -0.6, 0.0])  # 팔 위치 설정
        time.sleep(1.5)  # 1.5초 대기
        self.get_logger().info("6단계 완료: 팔 픽업 위치로 이동.")

        # 모든 단계가 완료되면 노드를 종료하지 않고 계속 spin을 호출
        self.get_logger().info("\n==================== 모든 단계 완료 ====================")
        self.get_logger().info("모든 단계가 성공적으로 완료되었습니다.")

def main(args=None):
    rclpy.init(args=args)

    robot_control_node = RobotControlNode()

    # Step 실행
    robot_control_node.execute_steps()
    
    # 노드 종료
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
