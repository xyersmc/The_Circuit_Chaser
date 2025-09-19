Python 3.13.6 (tags/v3.13.6:4e66535, Aug  6 2025, 14:36:00) [MSC v.1944 64 bit (AMD64)] on win32
Enter "help" below or click "Help" above for more information.
>>> from gpiozero import Servo, DistanceSensor, PWMOutputDevice, OutputDevice
... from time import sleep, monotonic
... import RPi.GPIO as GPIO
... import time, json, math, sys, signal
... 
... # ========= USER CONFIG =========
... # Servo (steering)
... SERVO_PIN = 18
... 
... # Ultrasonic sensors
... ULTRA1 = {'trigger': 27, 'echo': 17}  # front-left
... ULTRA2 = {'trigger': 22, 'echo': 23}  # front-right
... ULTRA3 = {'trigger': 9,  'echo': 10}  # side/rear-left
... ULTRA4 = {'trigger': 24, 'echo': 25}  # side/rear-right
... 
... # Motor driver pins (single DC motor)
... IN1_PIN = 20   # direction A
... IN2_PIN = 21   # direction B
... ENA_PIN = 12   # PWM speed control
... 
... # TCS3200 pins
... S0, S1, S2, S3, OUT = 5, 6, 13, 19, 26
... 
... # Behavior thresholds
... DIST_THRESH_M = 0.20        # 20 cm obstacle threshold
... ACTION_DELAY_S = 1.0
... TURN_TIME_S = 1.0
... BASE_SPEED = 0.7            # PWM duty (0.0 - 1.0)
... TCS_SAMPLES = 10
... TOLERANCE = 0.35
... CAL_FILE = "cmyk_calibration.json"
... # ===============================
... 
... # ========= Hardware Setup =========
... servo = Servo(SERVO_PIN, min_pulse_width=500/1_000_000, max_pulse_width=2500/1_000_000)
... 
ultrasonic_sensor1 = DistanceSensor(trigger=ULTRA1['trigger'], echo=ULTRA1['echo'])
ultrasonic_sensor2 = DistanceSensor(trigger=ULTRA2['trigger'], echo=ULTRA2['echo'])
ultrasonic_sensor3 = DistanceSensor(trigger=ULTRA3['trigger'], echo=ULTRA3['echo'])
ultrasonic_sensor4 = DistanceSensor(trigger=ULTRA4['trigger'], echo=ULTRA4['echo'])

# Motor driver (1 channel + PWM)
IN1 = OutputDevice(IN1_PIN)
IN2 = OutputDevice(IN2_PIN)
ENA = PWMOutputDevice(ENA_PIN)

# TCS3200 setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(S0, GPIO.OUT)
GPIO.setup(S1, GPIO.OUT)
GPIO.setup(S2, GPIO.OUT)
GPIO.setup(S3, GPIO.OUT)
GPIO.setup(OUT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(S0, True)   # 20% frequency scaling
GPIO.output(S1, False)

# ========= Motor & Motion =========
def motor_forward(speed=BASE_SPEED):
    IN1.on(); IN2.off()
    ENA.value = speed

def motor_backward(speed=BASE_SPEED):
    IN1.off(); IN2.on()
    ENA.value = speed

def motor_stop():
    ENA.value = 0
    IN1.off(); IN2.off()

def move_forward():
    print("→ Forward")
    servo.value = 0.0
    motor_forward()

def stop():
    print("■ Stop")
    motor_stop()

def turn_right():
    print("↷ Right turn")
    servo.value = -1.0
    motor_forward()
    sleep(TURN_TIME_S)
    motor_stop()
    servo.value = 0.0

def turn_left():
    print("↶ Left turn")
    servo.value = 1.0
    motor_forward()
    sleep(TURN_TIME_S)
    motor_stop()
    servo.value = 0.0

# ========= TCS3200 Helpers =========
def _read_channel(s2_state, s3_state, samples=TCS_SAMPLES):
    GPIO.output(S2, s2_state)
    GPIO.output(S3, s3_state)
    time.sleep(0.001)
    start = time.time()
    for _ in range(samples):
        GPIO.wait_for_edge(OUT, GPIO.FALLING)
    duration = time.time() - start
    return samples / duration if duration > 0 else 0

def get_rgb():
    r = _read_channel(False, False)
    b = _read_channel(False, True)
    g = _read_channel(True,  True)
    return (r, g, b)

def normalize_rgb(rgb):
    total = sum(rgb)
    return tuple(val / total for val in rgb) if total else (0,0,0)

def rgb_to_cmyk(rgb):
    r, g, b = normalize_rgb(rgb)
    c = 1 - r; m = 1 - g; y = 1 - b
    k = min(c, m, y)
    if k >= 1.0:
        return (0,0,0,1)
    c = (c-k)/(1-k); m=(m-k)/(1-k); y=(y-k)/(1-k)
    return (c, m, y, k)

def euclidean_distance(c1, c2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2)))

def load_references():
    try:
        with open(CAL_FILE, "r") as f:
            return json.load(f)
    except:
        return {
            "orange": [0.0, 0.4, 0.9, 0.0],
            "blue":   [1.0, 1.0, 0.0, 0.2]
        }

reference_colors = load_references()

# ========= Main Loop =========
last_action_time = monotonic()

def too_close_front():
    return ultrasonic_sensor1.distance < DIST_THRESH_M or ultrasonic_sensor2.distance < DIST_THRESH_M

def too_close_side():
    return ultrasonic_sensor3.distance < DIST_THRESH_M or ultrasonic_sensor4.distance < DIST_THRESH_M

def graceful_exit(*args):
    stop()
    GPIO.cleanup()
    sys.exit(0)

import signal
signal.signal(signal.SIGINT, graceful_exit)
signal.signal(signal.SIGTERM, graceful_exit)

print("Robot started. Press Ctrl+C to stop.")

while True:
    now = monotonic()

    # Obstacle check
    if too_close_front():
        stop()
        if now - last_action_time > ACTION_DELAY_S:
            turn_right()
            last_action_time = now
        continue
    elif too_close_side():
        stop()
        if now - last_action_time > ACTION_DELAY_S:
            turn_left()
            last_action_time = now
        continue

    # Color detection
    rgb = get_rgb()
    cmyk = rgb_to_cmyk(rgb)
    best, best_dist = None, float("inf")
    for name, ref in reference_colors.items():
        dist = euclidean_distance(cmyk, ref)
        if dist < best_dist:
            best, best_dist = name, dist

    print(f"TCS3200 → {best} (dist={best_dist:.3f})")

    if best and best_dist <= TOLERANCE and now - last_action_time > ACTION_DELAY_S:
        if best.lower() == "orange":
            stop(); turn_right()
            last_action_time = now
        elif best.lower() == "blue":
            stop(); turn_left()
            last_action_time = now
    else:
        move_forward()
