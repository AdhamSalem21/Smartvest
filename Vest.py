import os
import time
import cv2
import torch
import serial
import threading
import RPi.GPIO as GPIO
from datetime import datetime

# GPIO Setup 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Buttons
BUTTON_MODE_1 = 5  # Object Detection
BUTTON_MODE_2 = 6  # SOS Mode
BUTTON_MODE_3 = 26  # Obstacle Detection

# Ultrasonic Sensor Pins
TRIG = 23
ECHO = 24

# Setup pins
GPIO.setup(BUTTON_MODE_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_MODE_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_MODE_3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Text-to-Speech
def speak(text):
    print(f"[TTS] {text}")
    os.system(f'espeak "{text}" 2>/dev/null')

# Serial Setup
print("Initializing serial connections...")
gps_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)        # TTL USB GPS
sim800_serial = serial.Serial('/dev/serial0', 9600, timeout=1)     # SIM800L
print("Serial ports initialized.")

# Load YOLOv5 
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', trust_repo=True)
print("YOLOv5 model loaded.")

# Helper Functions 
def get_gps_location():
    print("[GPS] Reading location...")
    while True:
        line = gps_serial.readline().decode('utf-8', errors='ignore')
        if "$GPRMC" in line or "$GPGGA" in line:
            parts = line.split(',')
            if len(parts) > 5 and parts[2] == 'A':  # Valid fix
                lat = parts[3]
                lon = parts[5]
                print(f"[GPS] Location: Lat={lat}, Lon={lon}")
                return f"Lat: {lat}, Lon: {lon}"
        time.sleep(0.5)

def send_sms(location):
    print(f"[SMS] Sending: {location}")
    sim800_serial.write(b'AT\r')
    time.sleep(1)
    sim800_serial.write(b'AT+CMGF=1\r')
    time.sleep(1)
    sim800_serial.write(b'AT+CMGS="+201553401717"\r')  
    time.sleep(1)
    sim800_serial.write(f"Emergency! Location: {location}".encode())
    sim800_serial.write(bytes([26]))  
    time.sleep(3)
    print("[SMS] Sent!")

# Mode Functions
def object_detection_mode(active_flag):
    speak("Object detection mode activated")
    cap = cv2.VideoCapture('/dev/video1')
    if not cap.isOpened():
        speak("Camera not accessible")
        return

    while active_flag['mode'] == 1:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)
        labels = results.names
        detected = set()

        for *box, conf, cls in results.xyxy[0]:
            label = labels[int(cls)]
            detected.add(label)

        for label in detected:
            speak(f"I see a {label}")

        cv2.imshow("YOLOv5 Detection", results.render()[0])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Check for mode switch (button press)
        if GPIO.input(BUTTON_MODE_1) == GPIO.LOW and active_flag['mode'] != 1:
            print("Switching mode to 1...")
            break
        if GPIO.input(BUTTON_MODE_2) == GPIO.LOW and active_flag['mode'] != 2:
            print("Switching mode to 2...")
            break
        if GPIO.input(BUTTON_MODE_3) == GPIO.LOW and active_flag['mode'] != 3:
            print("Switching mode to 3...")
            break

    cap.release()
    cv2.destroyAllWindows()
    speak("Exiting object detection mode")

def sos_mode(active_flag):
    speak("SOS mode activated")

    def periodic_sms():
        while active_flag['mode'] == 2:
            location = get_gps_location()
            send_sms(location)
            time.sleep(1800)

    threading.Thread(target=periodic_sms, daemon=True).start()

    while active_flag['mode'] == 2:
        if GPIO.input(BUTTON_MODE_1) == GPIO.LOW and active_flag['mode'] != 1:
            print("Switching mode to 1...")
            break
        if GPIO.input(BUTTON_MODE_2) == GPIO.LOW and active_flag['mode'] != 2:
            print("Switching mode to 2...")
            break
        if GPIO.input(BUTTON_MODE_3) == GPIO.LOW and active_flag['mode'] != 3:
            print("Switching mode to 3...")
            break
        if GPIO.input(BUTTON_MODE_2) == GPIO.LOW:
            location = get_gps_location()
            send_sms(location)
            speak("SOS message sent")
            time.sleep(1)

    speak("Exiting SOS mode")

def obstacle_detection_mode(active_flag):
    speak("Obstacle detection mode activated")

    while active_flag['mode'] == 3:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        duration = pulse_end - pulse_start
        distance = (duration * 34300) / 2

        print(f"[Ultrasonic] Distance: {distance:.2f} cm")
        if distance < 50:
            speak("Obstacle ahead. Please turn left.")
        time.sleep(1)

        # Check for mode switch (button press)
        if GPIO.input(BUTTON_MODE_1) == GPIO.LOW and active_flag['mode'] != 1:
            print("Switching mode to 1...")
            break
        if GPIO.input(BUTTON_MODE_2) == GPIO.LOW and active_flag['mode'] != 2:
            print("Switching mode to 2...")
            break
        if GPIO.input(BUTTON_MODE_3) == GPIO.LOW and active_flag['mode'] != 3:
            print("Switching mode to 3...")
            break

    speak("Exiting obstacle detection mode")

#Main Loop 
def main():
    speak("Device ready. Press a button to select mode.")
    print("System started. Awaiting button press...")

    active_flag = {'mode': 0}  # Shared flag to track current mode

    while True:
        if GPIO.input(BUTTON_MODE_1) == GPIO.LOW and active_flag['mode'] != 1:
            print("Mode 1 button pressed.")
            active_flag['mode'] = 1
            object_detection_mode(active_flag)

        elif GPIO.input(BUTTON_MODE_2) == GPIO.LOW and active_flag['mode'] != 2:
            print("Mode 2 button pressed.")
            active_flag['mode'] = 2
            sos_mode(active_flag)

        elif GPIO.input(BUTTON_MODE_3) == GPIO.LOW and active_flag['mode'] != 3:
            print("Mode 3 button pressed.")
            active_flag['mode'] = 3
            obstacle_detection_mode(active_flag)

        time.sleep(0.1)

# Entry
if _name_ == "_main_":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\n🛑 Program terminated.")
