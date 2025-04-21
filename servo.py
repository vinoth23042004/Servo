import RPi.GPIO as GPIO
import time
import cv2
import pygame
import numpy as np

# Setup GPIO pins for servos and IR sensor
PAN_PIN = 12  # Servo for pan
TILT_PIN = 13  # Servo for tilt
IR_SENSOR_PIN = 6  # IR sensor input pin

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PAN_PIN, GPIO.OUT)
GPIO.setup(TILT_PIN, GPIO.OUT)
GPIO.setup(IR_SENSOR_PIN, GPIO.IN)

# Initialize PWM for servos
pan_pwm = GPIO.PWM(PAN_PIN, 50)  # 50Hz for servos
tilt_pwm = GPIO.PWM(TILT_PIN, 50)
pan_pwm.start(7.5)  # Neutral position for pan
tilt_pwm.start(7.5)  # Neutral position for tilt

# Initialize the camera
camera = cv2.VideoCapture(0)

# Load the pre-trained Haar Cascade Classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initialize Pygame for display
pygame.init()
WIDTH, HEIGHT = 500, 300
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cosmo-style Eyes with Changing Shape")

# Colors for eyes
BG_COLOR = (10, 10, 25)
EYE_COLOR = (120, 255, 255)
LID_COLOR = BG_COLOR
OUTLINE_COLOR = (70, 220, 220)

# Eye shape and position
eye_width = 70
eye_height = 100
eye_radius = 35
eye_spacing = 120
eye_y = HEIGHT // 2 - eye_height // 2
eye_x1 = WIDTH // 2 - eye_spacing // 2 - eye_width // 2
eye_x2 = WIDTH // 2 + eye_spacing // 2 - eye_width // 2

# Happy eyes flag and time control
is_happy = False
last_change_time = time.time()
change_interval = 3  # Change shape every 3 seconds

# Setup Clock for Pygame
clock = pygame.time.Clock()

# Servo Movement Functions
def move_pan(angle):
    """Moves the pan servo to a specific angle."""
    duty_cycle = angle / 18 + 2  # Convert angle to PWM duty cycle
    pan_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)

def move_tilt(angle):
    """Moves the tilt servo to a specific angle."""
    duty_cycle = angle / 18 + 2  # Convert angle to PWM duty cycle
    tilt_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)

def detect_faces():
    """Detect faces in the camera frame."""
    ret, frame = camera.read()
    if not ret:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
    return faces, frame

def track_face(faces, frame):
    """Track the first face and adjust the pan and tilt."""
    if len(faces) == 0:
        return None
    
    # Use the first detected face
    (x, y, w, h) = faces[0]
    
    # Get the center of the face
    face_center_x = x + w // 2
    face_center_y = y + h // 2
    
    # Get the center of the frame
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2
    
    # Adjust pan and tilt based on face position
    if face_center_x < frame_center_x - 50:
        move_pan(5)  # Move left
    elif face_center_x > frame_center_x + 50:
        move_pan(10)  # Move right
    
    if face_center_y < frame_center_y - 50:
        move_tilt(10)  # Move down
    elif face_center_y > frame_center_y + 50:
        move_tilt(5)  # Move up

def detect_motion():
    """Detect motion using the IR sensor."""
    return GPIO.input(IR_SENSOR_PIN) == GPIO.HIGH  # Motion detected if HIGH signal

def happy_state():
    """Move to a happy state."""
    move_pan(90)  # Pan to the right (happy state)
    move_tilt(30)  # Tilt downwards (happy state)

def draw_eye(x, y, happy=False):
    """Draws the main eye with squircle style, changes shape if happy."""
    if happy:
        # Draw a very small central part of the eye for a "happy" expression
        pygame.draw.rect(screen, EYE_COLOR, (x, y + eye_height // 3, eye_width, eye_height // 6), border_radius=eye_radius)
        pygame.draw.rect(screen, OUTLINE_COLOR, (x, y + eye_height // 3, eye_width, eye_height // 6), 2, border_radius=eye_radius)
    else:
        # Regular eye (fully open)
        pygame.draw.rect(screen, EYE_COLOR, (x, y, eye_width, eye_height), border_radius=eye_radius)
        pygame.draw.rect(screen, OUTLINE_COLOR, (x, y, eye_width, eye_height), 2, border_radius=eye_radius)

def update_eye_shape():
    """Change the eye shape periodically."""
    global is_happy, last_change_time
    current_time = time.time()
    
    # If time exceeds the change interval, toggle the eye shape
    if current_time - last_change_time > change_interval:
        is_happy = not is_happy  # Toggle between happy and normal eyes
        last_change_time = current_time  # Reset the timer

# Main loop
try:
    while True:
        faces, frame = detect_faces()
        
        if faces is not None:
            # Track the first face
            track_face(faces, frame)
        
        # Check if IR sensor detects motion (hand near the sensor)
        if detect_motion():
            print("Happy state triggered!")
            happy_state()
        
        # Show the video feed on the Pi Display (use Pygame for eyes)
        screen.fill(BG_COLOR)
        
        update_eye_shape()  # Check if it's time to change the eye shape
        
        # Draw eyes with or without happy expression
        draw_eye(eye_x1, eye_y, happy=is_happy)
        draw_eye(eye_x2, eye_y, happy=is_happy)

        # Show the Pygame window
        pygame.display.flip()
        
        # Exit on pressing 'q'
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        time.sleep(0.1)  # To ensure the loop runs at a manageable speed

except KeyboardInterrupt:
    print("Program stopped.")
finally:
    # Cleanup
    pan_pwm.stop()
    tilt_pwm.stop()
    GPIO.cleanup()
    camera.release()
    pygame.quit()
    cv2.destroyAllWindows()
