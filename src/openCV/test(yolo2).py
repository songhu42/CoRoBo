import cv2
import numpy as np
from ultralytics import YOLO

# 동영상 파일 경로
video_path = "/home/test/Desktop/CoRoBo/src/imgs/ss.mp4"  # 여기에 동영상 파일 경로를 입력하세요

# 동영상 신호 받기
VideoSignal = cv2.VideoCapture(video_path)

# YOLOv8 모델 로드 (모델 파일 경로)
model = YOLO("yolov8n.pt")  # yolov8n.pt는 yolov8의 경량 모델 예시입니다.

# 기준 객체 크기 (cm)
# 예: 사람이 1.7m(170cm)라고 가정
reference_object_height_cm = 170  # 실제 크기 (cm)

# 카메라의 focal length를 알아야 하지만, 여기서는 임의로 추정합니다.
# 초점 거리(focal length) 계산이 필요할 수 있음
focal_length_px = 800  # 초점 거리(픽셀) 예시 값

# 기준 객체의 픽셀 크기 (동영상에서 사람의 높이를 추정하여 지정)
reference_object_height_px = 250  # 예시로 픽셀 단위 크기 지정

while True:
    # 동영상 프레임 읽기
    ret, frame = VideoSignal.read()

    # 동영상이 끝났으면 종료
    if not ret:
        print("동영상 재생이 끝났습니다.")
        break

    # YOLOv8 모델을 사용한 객체 탐지
    results = model(frame)

    # results[0]은 YOLO 객체로, 여기에서 탐지된 정보와 클래스 이름을 추출
    detections = results[0].boxes

    # 탐지된 객체의 정보 가져오기
    boxes = detections.xyxy.cpu().numpy()  # [x1, y1, x2, y2] 형식으로 좌표
    confidences = detections.conf.cpu().numpy()  # 신뢰도
    class_ids_detected = detections.cls.cpu().numpy().astype(int)  # 감지된 클래스의 ID들

    # 클래스 이름 가져오기 (results[0].names는 모델 클래스 이름 목록)
    class_names = results[0].names  # 모델의 클래스 이름 목록

    # 객체가 감지되었을 때 경계상자 그리기
    for i, box in enumerate(boxes):
        if confidences[i] > 0.5:  # 신뢰도가 0.5 이상인 객체만 표시
            x1, y1, x2, y2 = map(int, box[:4])
            label = class_names[class_ids_detected[i]]  # 클래스 이름
            confidence = confidences[i]

            # 객체의 높이 (픽셀)
            object_height_px = y2 - y1

            # 기준 객체의 크기 비율을 이용하여 cm로 변환
            # reference_object_height_px: 기준 객체의 픽셀 높이
            # reference_object_height_cm: 기준 객체의 실제 cm 높이
            object_height_cm = (object_height_px * reference_object_height_cm) / reference_object_height_px

            # 경계상자와 클래스 정보 이미지에 입력
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 5)
            cv2.putText(frame, f"{label} {confidence:.2f} ({object_height_cm:.2f} cm)", 
                        (x1, y1 - 10), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)

    # 결과 이미지 출력
    cv2.imshow("YOLOv8 - cm", frame)

    # 'Esc' 키를 눌러 종료
    if cv2.waitKey(1) & 0xFF == 27:  # 27은 Esc 키 코드
        break

# 종료 시 동영상 닫기
VideoSignal.release()
cv2.destroyAllWindows()
