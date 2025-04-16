import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
from gpiozero import Servo
from time import sleep

# Servo GPIO pins (change if needed)
SERVO_PIN_X = 17  # GPIO 17 (Pin 11 on Pi)
SERVO_PIN_Y = 27  # GPIO 27 (Pin 13 on Pi)

# Initialize servos
servoX = Servo(SERVO_PIN_X, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servoY = Servo(SERVO_PIN_Y, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

# Map 0-180 to -1 to 1 for gpiozero Servo
def degree_to_gpio(val):
    return np.interp(val, [0, 180], [-1, 1])

# Initialize
servoPos = [90, 90]
servoX.value = degree_to_gpio(servoPos[0])
servoY.value = degree_to_gpio(servoPos[1])

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

detector = FaceDetector()

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        fx, fy = bboxs[0]["center"]
        pos = [fx, fy]

        servoX_val = np.interp(fx, [0, ws], [180, 0])
        servoY_val = np.interp(fy, [0, hs], [180, 0])

        # Clamp between 0 and 180
        servoX_val = max(0, min(180, servoX_val))
        servoY_val = max(0, min(180, servoY_val))

        servoPos = [servoX_val, servoY_val]

        # Update servos
        servoX.value = degree_to_gpio(servoX_val)
        servoY.value = degree_to_gpio(servoY_val)

        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)

        # Reset to center
        servoX.value = degree_to_gpio(90)
        servoY.value = degree_to_gpio(90)

    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
