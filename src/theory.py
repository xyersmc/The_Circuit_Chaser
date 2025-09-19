import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Motor control pins
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Function to move forward
def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# Function to stop the car
def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Function to turn the car right
def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    time.sleep(1)
    stop()

# Function to turn the car left
def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(1)
    stop()

# Initialize the camera
cap = cv2.VideoCapture(0)

# Define color ranges for detection
lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])

lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

while True:
    ret, frame = cap.read()
    
    # Convert frame to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Detect red blocks
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Detect green blocks
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Check if any red block is detected
    red_detected = False
    for contour in contours_red:
        if cv2.contourArea(contour) > 500:  # Ignore small contours
            red_detected = True
    
    # Check if any green block is detected
    green_detected = False
    for contour in contours_green:
        if cv2.contourArea(contour) > 500:  # Ignore small contours
            green_detected = True

    # If a red block is detected, turn right
    if red_detected:
        print("Red Block Detected! Turning Right.")
        stop()
        time.sleep(1)  # Pause for a moment before turning
        turn_right()
    
    # If a green block is detected, turn left
    elif green_detected:
        print("Green Block Detected! Turning Left.")
        stop()
        time.sleep(1)  # Pause for a moment before turning
        turn_left()
    
    # If no color block is detected, keep moving forward
    else:
        move_forward()

    # Show the frame with the detected color blocks
    cv2.imshow("Frame", frame)
    
    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and clean up GPIO
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
