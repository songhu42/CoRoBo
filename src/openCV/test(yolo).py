import cv2
import mediapipe as mp
import numpy as np
from ultralytics import YOLO
import time

# MediaPipe 포즈 추정 모듈 초기화 (MultiPose 모델)
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5, model_complexity=2)
mp_drawing = mp.solutions.drawing_utils

# YOLOv8 모델 로드
yolo_model = YOLO("yolov8n.pt")  # Use the appropriate YOLOv8 model

# 비디오 캡처 (웹캠 또는 비디오 파일)
capture = cv2.VideoCapture("/home/test/Desktop/CoRoBo/src/imgs/nn.mp4")

# 화면 크기 비율 및 인식률
s_factor = 0.5
frameCount = 0
start_time = time.time()

while True:
    ret, frame = capture.read()
    if not ret:
        break

    frameCount += 1

    if frameCount % 5 == 0:  # 5프레임마다 1회 처리
        frame_resized = cv2.resize(frame, None, fx=s_factor, fy=s_factor, interpolation=cv2.INTER_AREA)

        # YOLOv8 객체 탐지 (사람만)
        results = yolo_model(frame_resized)

        # Extract detections from the results object
        detections = results[0].boxes

        for detection in detections:
            xmin, ymin, xmax, ymax = detection.xyxy[0]
            conf = detection.conf[0]
            class_id = int(detection.cls[0])

            # "person"만 인식, 신뢰도가 0.5 이상인 경우만 처리
            if class_id == 0 and conf > 0.5:  # Person class and confidence threshold
                xmin = int(xmin * (1 / s_factor))
                ymin = int(ymin * (1 / s_factor))
                xmax = int(xmax * (1 / s_factor))
                ymax = int(ymax * (1 / s_factor))

                # Crop the person from the frame
                person_frame = frame[ymin:ymax, xmin:xmax]

                # Apply pose estimation
                rgb_person_frame = cv2.cvtColor(person_frame, cv2.COLOR_BGR2RGB)
                result_pose = pose.process(rgb_person_frame)

                if result_pose.pose_landmarks:
                    mp_drawing.draw_landmarks(person_frame, result_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                # Draw bounding box around person
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, 'Person', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Calculate FPS
        end_time = time.time()
        fps = frameCount / (end_time - start_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the output
        cv2.imshow('Pose Estimation with YOLOv8', frame)

    key = cv2.waitKey(20)
    if key == 27:  # ESC key to exit
        break

capture.release()
cv2.destroyAllWindows()
