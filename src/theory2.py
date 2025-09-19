import cv2
import numpy as np
import os
import time

# ----- Simulated Motor Control -----
def move_forward():
    print("[Motor] Moving Forward")

def stop():
    print("[Motor] Stopping")

def turn_right():
    print("[Motor] Turning Right")
    time.sleep(1)
    stop()

def turn_left():
    print("[Motor] Turning Left")
    time.sleep(1)
    stop()

# ----- Input Source -----
input_source = 0  # Change to image path or 0 for webcam

# If the video file does not exist, create a test video
if isinstance(input_source, str) and not os.path.exists(input_source):
    print("Test video not found. Generating test video...")
    width, height = 640, 480
    fps = 30
    duration = 5  # seconds
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(input_source, fourcc, fps, (width, height))
    for frame_idx in range(fps * duration):
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        # Red block moves left to right
        red_x = int((frame_idx / (fps * duration)) * (width - 100))
        cv2.rectangle(frame, (red_x, 100), (red_x + 100, 200), (0, 0, 255), -1)
        # Green block moves right to left
        green_x = int(width - 100 - (frame_idx / (fps * duration)) * (width - 100))
        cv2.rectangle(frame, (green_x, 300), (green_x + 100, 400), (0, 255, 0), -1)
        video.write(frame)
    video.release()
    print(f"Test video created: {input_source}")

# ----- Load Video or Image -----
is_image = False
frame = None
cap = None

if isinstance(input_source, int) or input_source.isdigit() or input_source == 0:
    cap = cv2.VideoCapture(int(input_source))
elif os.path.exists(input_source):
    cap = cv2.VideoCapture(input_source)
else:
    frame = cv2.imread(input_source)
    if frame is not None:
        is_image = True
    else:
        raise FileNotFoundError(f"Cannot open {input_source}")

# ----- Color Detection -----
lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red mask
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Green mask
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    red_detected = False
    green_detected = False

    # Draw red detections
    for c in contours_red:
        if cv2.contourArea(c) > 500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame, "RED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            red_detected = True

    # Draw green detections
    for c in contours_green:
        if cv2.contourArea(c) > 500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            green_detected = True

    # Action logic
    if red_detected:
        print("Red Block Detected! Turning Right.")
        stop()
        time.sleep(1)
        turn_right()
    elif green_detected:
        print("Green Block Detected! Turning Left.")
        stop()
        time.sleep(1)
        turn_left()
    else:
        move_forward()

    cv2.imshow("Detection Simulation", frame)

# ----- Main Loop -----
if is_image:
    process_frame(frame)
    cv2.waitKey(0)
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        process_frame(frame)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

if cap:
    cap.release()
cv2.destroyAllWindows()
print("Simulation Ended.")
