import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        # 이미지 구독 (카메라에서 발행되는 이미지 토픽을 구독)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 카메라에서 발행되는 이미지 토픽
            self.image_callback,
            1
        )

        # CvBridge 초기화 (ROS 메시지 -> OpenCV 이미지 변환)
        self.bridge = CvBridge()

        # MediaPipe 포즈 추정 모듈 초기화
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5, model_complexity=2)
        self.mp_drawing = mp.solutions.drawing_utils

        # OpenCV 사람 검출을 위한 HOG descriptor
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # 타이머 설정 (1초에 한 번씩 루프 돌기)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 초기 시간 설정
        self.start_time = time.time()

    def image_callback(self, msg):
        """카메라에서 발행된 이미지를 처리하는 콜백 함수"""
        self.get_logger().info('Received image message')

        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # HOG 사람 검출기 사용
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

        for (x, y, w, h) in boxes:
            # 사람 부분만 잘라내기
            person_frame = frame[y:y+h, x:x+w]

            # 포즈 추정
            rgb_person_frame = cv2.cvtColor(person_frame, cv2.COLOR_BGR2RGB)
            result_pose = self.pose.process(rgb_person_frame)

            if result_pose.pose_landmarks:
                self.mp_drawing.draw_landmarks(person_frame, result_pose.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # 사람에 대한 바운딩 박스 그리기
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, 'Person', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # FPS 계산
        fps = 1.0 / (time.time() - self.start_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 결과를 화면에 출력
        cv2.imshow('MJPEG Image', frame)
        cv2.waitKey(1)

    def timer_callback(self):
        """타이머 콜백 함수 (1초마다 실행)"""
        self.get_logger().info('Timer triggered!')

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node interrupted by user")
    finally:
        node.destroy_node()
        # rclpy.shutdown() is automatically handled by rclpy.spin()
        # No need to call rclpy.shutdown() here.

if __name__ == '__main__':
    main()
