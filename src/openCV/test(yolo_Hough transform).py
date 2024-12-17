import cv2
import numpy as np
import logging
from ultralytics import YOLO  # YOLOv8 라이브러리 불러오기

def detect_edges(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([
        [(0, height), (width, height), (int(width / 2), int(height / 2))]  # 삼각형 영역
    ], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1
    angle = np.pi / 180
    min_threshold = 20
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                    np.array([]), minLineLength=30, maxLineGap=50)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        logging.info('라인 세그먼트가 감지되지 않았습니다.')
        return lane_lines
    
    left_fit = []
    right_fit = []

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]

            if slope < -0.5:
                left_fit.append((slope, intercept))
            elif slope > 0.5:
                right_fit.append((slope, intercept))

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_points(frame, left_fit_average))

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line

    y1 = height
    y2 = int(y1 * 0.6)

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def calculate_center_line(frame, lane_lines):
    if len(lane_lines) == 2:
        left_line = lane_lines[0][0]
        right_line = lane_lines[1][0]

        center_line = [
            int((left_line[0] + right_line[0]) / 2),
            int((left_line[1] + right_line[1]) / 2),
            int((left_line[2] + right_line[2]) / 2),
            int((left_line[3] + right_line[3]) / 2),
        ]
        return [center_line]
    return []

def display_center_line(frame, center_line):
    for line in center_line:
        x1, y1, x2, y2 = line
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)  # 파란색 선

        # 거리 표시 (예제: 픽셀 -> 미터 변환 비율 적용)
        distance_cm = abs(x2 - x1) * 0.05  # 픽셀당 0.05m 비율 가정
        label = f'{distance_cm:.2f} cm'
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame

def car_detect(frame, model, confThreshold=0.5):
    results = model(frame)
    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = int(box.cls[0].item())
            confidence = box.conf[0].item()
            if confidence > confThreshold:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()

                label = f'{model.names[class_id]} {confidence:.2f}'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame

capture = cv2.VideoCapture("/home/test/Desktop/CoRoBo/src/imgs/rrr.mp4")

if not capture.isOpened():
    print("Error: Cannot open video.")
    exit()

model = YOLO('yolov8n.pt')

while capture.isOpened():
    ret, frame = capture.read()
    if not ret:
        break

    edges = detect_edges(frame)
    cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(frame, line_segments)

    center_line = calculate_center_line(frame, lane_lines)
    frame_with_center = display_center_line(frame, center_line)

    final_frame = car_detect(frame_with_center, model)

    cv2.imshow("Lane and Car Detection", final_frame)

    if cv2.waitKey(1) == 27:
        break

capture.release()
cv2.destroyAllWindows()
