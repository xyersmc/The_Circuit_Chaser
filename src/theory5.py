import cv2
import numpy as np
import RPi.GPIO as GPIO
import time, json, math
from gpiozero import Servo, DistanceSensor
from time import sleep

# ----------------------------
# SERVO (steering) + ESC (throttle)
# ----------------------------
steering = Servo(18, min_pulse_width=500/1_000_000,
                 max_pulse_width=2500/1_000_000)

throttle = Servo(12, min_pulse_width=1000/1_000_000,   # ESC pin
                 max_pulse_width=2000/1_000_000)

def move_forward():
    steering.value = 0       # straight
    throttle.value = 0.5     # forward speed (adjust 0.2–1.0)
    print("🚗 Moving forward")
    sleep(0.2)

def stop():
    throttle.value = 0.0     # neutral
    print("⛔ Stop")

def turn_right():
    steering.value = -1      # right
    throttle.value = 0.4     # keep some forward motion
    print("↪ Turning right")
    sleep(0.7)

def turn_left():
    steering.value = 1       # left
    throttle.value = 0.4
    print("↩ Turning left")
    sleep(0.7)

# ----------------------------
# CAMERA (OpenCV pillar detection)
# ----------------------------
cap = cv2.VideoCapture(0)
lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])
lower_green, upper_green = np.array([40, 50, 50]), np.array([80, 255, 255])

def detect_objects():
    ret, frame = cap.read()
    if not ret:
        return False, False, frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    red_detected = cv2.countNonZero(mask_red) > 500
    green_detected = cv2.countNonZero(mask_green) > 500
    return red_detected, green_detected, frame

def release_camera():
    cap.release()
    cv2.destroyAllWindows()

# ----------------------------
# TCS3200 (ground color sensor)
# ----------------------------
S0, S1, S2, S3, OUT = 5, 6, 13, 19, 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(S0, GPIO.OUT); GPIO.setup(S1, GPIO.OUT)
GPIO.setup(S2, GPIO.OUT); GPIO.setup(S3, GPIO.OUT)
GPIO.setup(OUT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(S0, True); GPIO.output(S1, False)

def read_color(s2_state, s3_state):
    GPIO.output(S2, s2_state); GPIO.output(S3, s3_state)
    time.sleep(0.05)
    start = time.time()
    for _ in range(10):
        GPIO.wait_for_edge(OUT, GPIO.FALLING)
    duration = time.time() - start
    return 10 / duration

def get_rgb():
    r = read_color(False, False)
    b = read_color(False, True)
    g = read_color(True, True)
    return (r, g, b)

def normalize_rgb(rgb):
    total = sum(rgb)
    return tuple(val/total for val in rgb) if total else (0, 0, 0)

def rgb_to_cmyk(rgb):
    r, g, b = normalize_rgb(rgb)
    c, m, y = 1-r, 1-g, 1-b
    k = min(c, m, y)
    if k >= 1: return (0,0,0,1)
    return ((c-k)/(1-k), (m-k)/(1-k), (y-k)/(1-k), k)

def euclidean_distance(c1, c2):
    return math.sqrt(sum((a-b)**2 for a,b in zip(c1,c2)))

def get_color():
    with open("cmyk_calibration.json","r") as f:
        ref = json.load(f)
    cmyk = rgb_to_cmyk(get_rgb())
    best_match, best_dist = None, float("inf")
    for name, ref_cmyk in ref.items():
        dist = euclidean_distance(cmyk, ref_cmyk)
        if dist < best_dist:
            best_match, best_dist = name, dist
    return best_match if best_dist < 0.35 else "unknown"

# ----------------------------
# ULTRASONIC sensors
# ----------------------------
ultrasonic_sensor1 = DistanceSensor(trigger=27, echo=17)
ultrasonic_sensor2 = DistanceSensor(trigger=22, echo=23)
ultrasonic_sensor3 = DistanceSensor(trigger=9,  echo=10)
ultrasonic_sensor4 = DistanceSensor(trigger=24, echo=25)

# ----------------------------
# MAIN LOOP with Lap Counter
# ----------------------------
last_action_time = 0
action_delay = 1.0
last_ground_color = None
STOP_DELAY = 1.0

lap_count = 0
on_marker = False
START_MARKER = "orange"
TOTAL_LAPS = 3

try:
    while lap_count < TOTAL_LAPS:
        red_detected, green_detected, frame = detect_objects()
        obstacle_front = ultrasonic_sensor1.distance < 0.2 or ultrasonic_sensor2.distance < 0.2
        obstacle_side  = ultrasonic_sensor3.distance < 0.2 or ultrasonic_sensor4.distance < 0.2
        now = time.time()

        # --- Ground lap counter
        ground_color = get_color()
        if ground_color == START_MARKER and not on_marker:
            lap_count += 1
            print(f"✅ Lap {lap_count} completed!")
            on_marker = True
            time.sleep(1.0)
        elif ground_color != START_MARKER:
            on_marker = False

        if lap_count >= TOTAL_LAPS:
            stop()
            print("🏁 Race finished: 3 laps complete")
            break

        # --- Obstacle handling
        if red_detected or obstacle_front:
            if now - last_action_time > action_delay:
                stop(); turn_right()
                last_action_time = now
        elif green_detected or obstacle_side:
            if now - last_action_time > action_delay:
                stop(); turn_left()
                last_action_time = now
        else:
            move_forward()

        # Show camera feed
        cv2.imshow("Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    stop()
    release_camera()
    GPIO.cleanup()
