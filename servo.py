import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define pins
servo_pinX = 17  # GPIO17 (physical pin 11)
servo_pinY = 18  # GPIO18 (physical pin 12)

# Laser pin (optional)
#laser_pin = 22   # GPIO22

# Setup pins
GPIO.setup(servo_pinX, GPIO.OUT)
GPIO.setup(servo_pinY, GPIO.OUT)
GPIO.setup(laser_pin, GPIO.OUT)

# Initialize PWM on 50Hz
servoX = GPIO.PWM(servo_pinX, 50)
servoY = GPIO.PWM(servo_pinY, 50)
servoX.start(7.5)  # Mid position
servoY.start(7.5)

#GPIO.output(laser_pin, GPIO.LOW)

# Face Detection Setup
cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

detector = FaceDetector()
servoPos = [90, 90]

def angle_to_duty(angle):
    return 2.5 + (angle / 180.0) * 10.0  # Convert angle to duty cycle

try:
    while True:
        success, img = cap.read()
        img, bboxs = detector.findFaces(img, draw=False)

        if bboxs:
            fx, fy = bboxs[0]["center"]
            pos = [fx, fy]

            servoX_deg = np.interp(fx, [0, ws], [180, 0])
            servoY_deg = np.interp(fy, [0, hs], [0, 180])

            servoX_deg = max(0, min(180, servoX_deg))
            servoY_deg = max(0, min(180, servoY_deg))

            servoPos[0] = servoX_deg
            servoPos[1] = servoY_deg

            dutyX = angle_to_duty(servoX_deg)
            dutyY = angle_to_duty(servoY_deg)
            servoX.ChangeDutyCycle(dutyX)
            servoY.ChangeDutyCycle(dutyY)

            GPIO.output(laser_pin, GPIO.HIGH)

            # Draw target
            cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
            cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
            cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
            cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)
            cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
            cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        else:
            GPIO.output(laser_pin, GPIO.LOW)
            cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
            cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
            cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
            cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)
            cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)

        cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

        cv2.imshow("Image", img)
        cv2.waitKey(1)

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    servoX.ChangeDutyCycle(7.5)
    servoY.ChangeDutyCycle(7.5)
    time.sleep(0.5)
    servoX.stop()
    servoY.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
