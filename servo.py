import cv2
import numpy as np
import pygame
import time
import random
from cvzone.FaceDetectionModule import FaceDetector
from gpiozero import Servo

# ========== Raspberry Pi Servo Setup ==========
# Servo range from -1 to 1 in gpiozero
servo_left = Servo(17)   # GPIO pin 17
servo_right = Servo(18)  # GPIO pin 18

# ========== Pygame Setup ==========
pygame.init()
ws, hs = 1280, 720
screen = pygame.display.set_mode((ws, hs))
pygame.display.set_caption("Face-Eye & Servo Tracker")

# Colors
BLACK = (0, 0, 0)
BLUE = (0, 170, 255)
DARK_BLUE = (0, 120, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)

# Eye positions
eye_left_start = [280, 300]
eye_right_start = [520, 300]
eye_size = 100
movement_range = 60
speed = 0.5

# Emotion & blink logic
last_blink = time.time()
blink_duration = 0.1
blink_interval = random.randint(3, 6)
is_blinking = False
emotions = ["neutral", "happy", "sad", "angry", "surprised"]
current_emotion = "neutral"
last_emotion_change = time.time()
emotion_change_interval = 5

# ========== Camera & Face Detection ==========
cap = cv2.VideoCapture(0)
cap.set(3, ws)
cap.set(4, hs)
face_detector = FaceDetector()

# Map function for servo
def map_range(value, leftMin, leftMax, rightMin, rightMax):
    value = max(leftMin, min(value, leftMax))  # Clamp
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

# ========== Main Loop ==========
running = True
while running:
    screen.fill(BLACK)
    eye_left = eye_left_start.copy()
    eye_right = eye_right_start.copy()

    success, img = cap.read()
    img, bboxs = face_detector.findFaces(img, draw=False)

    if bboxs:
        fx, fy = bboxs[0]["center"]

        # Eye movement range for display
        target_eyeX = np.interp(fx, [100, 540], [movement_range, -movement_range])
        target_eyeY = np.interp(fy, [50, 430], [movement_range, -movement_range])

        # Smooth Pygame eye motion
        eye_left[0] += (eye_left_start[0] + target_eyeX - eye_left[0]) * speed
        eye_left[1] += (eye_left_start[1] + target_eyeY - eye_left[1]) * speed
        eye_right[0] += (eye_right_start[0] + target_eyeX - eye_right[0]) * speed
        eye_right[1] += (eye_right_start[1] + target_eyeY - eye_right[1]) * speed

        # Servo control — FIXED direction!
        servo_angle = map_range(fx, 100, 540, -0.7, 0.7)
        servo_left.value = servo_angle
        servo_right.value = servo_angle

    # Blinking logic
    current_time = time.time()
    if current_time - last_blink >= blink_interval:
        is_blinking = True
        last_blink = current_time
        blink_interval = random.randint(3, 6)
    if is_blinking and current_time - last_blink >= blink_duration:
        is_blinking = False

    # Change emotion
    if current_time - last_emotion_change >= emotion_change_interval:
        current_emotion = random.choice(emotions)
        last_emotion_change = current_time

    # Emotion visuals
    if current_emotion == "happy":
        eye_color = GREEN
        border_radius = 30
    elif current_emotion == "sad":
        eye_color = BLUE
        border_radius = 10
    elif current_emotion == "angry":
        eye_color = RED
        border_radius = 5
    elif current_emotion == "surprised":
        eye_color = YELLOW
        border_radius = 40
    else:
        eye_color = DARK_BLUE
        border_radius = 20

    # Draw eyes
    if not is_blinking:
        pygame.draw.rect(screen, eye_color, (eye_left[0] - eye_size // 2, eye_left[1] - eye_size // 2, eye_size, eye_size), border_radius=border_radius)
        pygame.draw.rect(screen, eye_color, (eye_right[0] - eye_size // 2, eye_right[1] - eye_size // 2, eye_size, eye_size), border_radius=border_radius)
        pygame.draw.rect(screen, BLUE, (eye_left[0] - eye_size // 2 + 10, eye_left[1] - eye_size // 2 + 10, eye_size - 20, eye_size - 20), border_radius=border_radius)
        pygame.draw.rect(screen, BLUE, (eye_right[0] - eye_size // 2 + 10, eye_right[1] - eye_size // 2 + 10, eye_size - 20, eye_size - 20), border_radius=border_radius)
    else:
        pygame.draw.rect(screen, DARK_BLUE, (eye_left[0] - eye_size // 2, eye_left[1] - 5, eye_size, 10), border_radius=5)
        pygame.draw.rect(screen, DARK_BLUE, (eye_right[0] - eye_size // 2, eye_right[1] - 5, eye_size, 10), border_radius=5)

    # Display OpenCV window
    cv2.imshow("Face Tracker", img)
    pygame.display.flip()

    # Exit
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

# Cleanup
cap.release()
cv2.destroyAllWindows()
pygame.quit()
servo_left.value = 0
servo_right.value = 0
