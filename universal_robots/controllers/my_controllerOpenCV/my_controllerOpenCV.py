from controller import Robot, Camera
import cv2
import numpy as np

robot = Robot()
TIME_STEP = 32
MAX_STEPS = 100

# Create camera
camera = robot.getDevice('camera')
# Enable the camera and recognition
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)