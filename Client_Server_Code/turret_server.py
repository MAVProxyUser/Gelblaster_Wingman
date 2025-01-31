#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import paho.mqtt.client as mqtt
import json
import platform
import socket
import netifaces
import cv2
import numpy as np
import os
import subprocess
import logging
import depthai as dai

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Platform detection
current_platform = platform.system()
print(f"Running on {current_platform}", flush=True)

# Camera initialization settings for macOS
if platform.system() == 'Darwin':
    print("\nMacOS Camera Setup Instructions:", flush=True)
    print("1. Open System Preferences", flush=True)
    print("2. Go to Security & Privacy -> Privacy -> Camera", flush=True)
    print("3. Ensure Terminal has camera access", flush=True)
    print("4. If Terminal isn't listed, try running this command first:", flush=True)
    print("   tccutil reset Camera\n", flush=True)

    os.environ['OPENCV_AVFOUNDATION_SKIP_AUTH'] = '0'
    os.environ['OPENCV_VIDEOIO_DEBUG'] = '1'

# Common imports for all platforms
if current_platform == 'Linux':
    import Jetson.GPIO as GPIO
    import depthai as dai
    from dynamixel_sdk import *
    import serial.tools.list_ports

# Add cleanup at start to ensure GPIO is released
if current_platform == 'Linux':
    try:
        if GPIO.getmode() is not None:  # Only cleanup if GPIO is initialized
            GPIO.cleanup()
    except:
        pass
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # Initialize relay pin
    RELAY_PIN = 7  # GPIO9
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    GPIO.output(RELAY_PIN, GPIO.HIGH)  # Start with relay deactivated
    print(f"Initialized relay pin {RELAY_PIN} to HIGH (deactivated)", flush=True)

# MQTT Topics
MQTT_TOPIC_CONTROL = "dpad/commands"
MQTT_TOPIC_CAMERA = "camera/feed"
MQTT_TOPIC_BBOX = "camera/bbox"
MQTT_TOPIC_SERVO_STATUS = "servo/status"
MQTT_TOPIC_PLATFORM = "server/platform"
MQTT_TOPIC_PLATFORM_REQUEST = "server/platform_request"
MQTT_TOPIC_DEPTH = "camera/depth"  # Define the topic for depth data
MQTT_TOPIC_RELAY = "relay/command"  # Topic for relay control
RELAY_ACTIVATE = 'off'    # Relay is "normally closed" - sending 'off' activates it
RELAY_DEACTIVATE = 'on'   # Sending 'on' deactivates the normally closed relay

# Add with other MQTT topics at the top
MQTT_TOPIC_COMMAND = "dpad/commands"  # Topic for keyboard commands

# Servo Configuration
INITIAL_TILT_ANGLE = -240
TILT_ANGLE_MAX = -130
TILT_ANGLE_MIN = -340
PAN_ANGLE_MIN = -1200
PAN_ANGLE_MAX = 1200

# Dynamixel Configuration
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
LEN_GOAL_POSITION = 4
DXL1_ID = 1  # Pan servo ID
DXL2_ID = 2  # Tilt servo ID
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 40
ADDR_GOAL_TORQUE = 102
LEN_GOAL_TORQUE = 2
DXL_GOAL_TORQUE = 500

# Operating mode settings
OPERATING_MODE = 4         # Extended position control mode (multi-turn)
MAX_POSITION_VALUE = 4095  # Maximum position value
MIN_POSITION_VALUE = 0     # Minimum position value
POSITION_CENTER = 2048     # Center position

# Thread-safe variables
latest_command = {'pan_angle': 0.0, 'tilt_angle': 0.0}  # Default values
command_lock = threading.Lock()

# Global TOF variable
tof = 0.0  # Default TOF value set to 0

# Global variable for the latest frame
latest_frame = None
frame_lock = threading.Lock()

# Remove the fixed HOME position constants and make them global variables
HOME_PAN_ANGLE = None
HOME_TILT_ANGLE = None

# Initialize current state variables
current_pan_angle = HOME_PAN_ANGLE
current_tilt_angle = HOME_TILT_ANGLE
current_speed = 0  # or whatever the default speed should be
current_auto_mode = False  # or whatever the default auto mode should be

# Add a global variable to store sensitivity
sensitivity = 1.0

# Add these constants at the top of the file with other constants
# HSV color range for green detection
LOWER_GREEN = np.array([35, 50, 50])   # Lower bound of green in HSV
UPPER_GREEN = np.array([85, 255, 255]) # Upper bound of green in HSV
MIN_CONTOUR_AREA = 15  # Lower the minimum area threshold

# Add this with other global variables near the top of the file
running = True

# Near the top of the file with other constants
PAN_ANGLE_PER_PIXEL = 0.1  # Degrees per pixel for pan
TILT_ANGLE_PER_PIXEL = 0.1  # Degrees per pixel for tilt

# Near the top of the file with other constants
PAN_ANGLE_MIN = -90.0  # Minimum pan angle in degrees
PAN_ANGLE_MAX = 90.0   # Maximum pan angle in degrees
TILT_ANGLE_MIN = -270.0  # Minimum tilt angle in degrees
TILT_ANGLE_MAX = -130.0  # Maximum tilt angle in degrees

# Global servo objects
pan_servo = None
tilt_servo = None

# Servo Control Parameters
SMOOTHING_FACTOR = 0.3  # Smoothing factor (0.0-1.0)
MAX_SPEED_DEG_PER_SEC = 360.0  # Doubled again to 360 deg/sec
UPDATE_RATE_HZ = 50  # Control loop update rate
MAX_SPEED = MAX_SPEED_DEG_PER_SEC / UPDATE_RATE_HZ  # Max speed per update
MIN_CORRECTION = 0.1  # Minimum correction to apply (degrees)

# Add this before the main execution block
def initialize_camera():
    """Initialize the camera based on platform."""
    global cap, q_rgb, device  # Add device to global
    
    if current_platform == 'Darwin':
        cap = cv2.VideoCapture(0)
        return cap.isOpened()
    else:
        try:
            # Try OAK-D first
            try:
                # Create pipeline
                pipeline = dai.Pipeline()

                # Define sources and outputs
                camRgb = pipeline.createColorCamera()
                xoutRgb = pipeline.createXLinkOut()

                xoutRgb.setStreamName("rgb")

                # Properties
                camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Use CAM_A instead of RGB
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                camRgb.setPreviewSize(640, 480)
                camRgb.setInterleaved(False)
                camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                camRgb.setFps(30)  # Set FPS
                
                # Enable auto focus and exposure with correct enum values
                camRgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
                camRgb.initialControl.setAutoExposureEnable()
                # Use AUTO for white balance instead of CONTINUOUS
                camRgb.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)

                # Linking
                camRgb.preview.link(xoutRgb.input)

                # Connect to device and start pipeline
                try:
                    # First try to find an available device
                    found_devices = dai.Device.getAllAvailableDevices()
                    if len(found_devices) == 0:
                        print("No devices found!")
                        return False
                    
                    for device_info in found_devices:
                        print(f"Found device: {device_info.getMxId()} {device_info.state}")
                    
                    # Connect to first available device
                    device = dai.Device(pipeline)
                    print(f"Connected to {device.getMxId()}")
                    
                    # Get output queue with latest-only mode
                    q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
                    
                    print("Camera initialized successfully")
                    return True
                    
                except Exception as e:
                    print(f"Error connecting to device: {e}")
                    return False
                
            except Exception as e:
                logging.warning(f"Failed to initialize OAK-D camera: {e}")
                logging.info("Falling back to regular webcam")
                # Fallback to regular webcam
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    logging.error("Failed to open webcam")
                    return False
                return True
                
        except Exception as e:
            logging.error(f"Error initializing camera: {e}")
            return False

def get_local_ip():
    if current_platform == 'Darwin':
        return '10.42.0.1'
    
    try:
        # Rest of the function for Linux...
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            if interface.startswith('wlan') or interface.startswith('eth'):
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    ip = addrs[netifaces.AF_INET][0]['addr']
                    if not ip.startswith('127.'):
                        print(f"Using IP address: {ip}", flush=True)
                        return ip
        return '10.42.0.1'  # Fallback to default IP
    except Exception as e:
        print(f"Error getting IP: {e}", flush=True)
        return '10.42.0.1'  # Fallback to default IP

def on_sensitivity_update(client, userdata, message):
    global sensitivity
    try:
        payload = json.loads(message.payload)
        sensitivity = payload.get('sensitivity', 1.0)
        print(f"Updated sensitivity to: {sensitivity}", flush=True)
    except Exception as e:
        print(f"Error processing sensitivity update: {e}", flush=True)

def detect_green_objects(frame):
    """Detect green objects in the frame."""
    # Store original frame dimensions
    height, width = frame.shape[:2]
    frame_center_x = width / 2
    frame_center_y = height / 2
    
    # Convert to HSV and create mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    logging.debug("Converted frame to HSV")

    # Create mask for green color using the constants
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    logging.debug("Created mask for green color")

    # Apply morphological operations
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    logging.debug("Applied morphological operations")

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    logging.debug(f"Found {len(contours)} contours")

    # Filter contours by area
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        logging.debug(f"Contour area: {area}")
        if area >= MIN_CONTOUR_AREA:
            valid_contours.append(contour)

    logging.debug(f"Filtered to {len(valid_contours)} valid contours")

    # Only process the largest contour if we found one
    if len(valid_contours) > 0:
        largest_contour = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        logging.debug(f"Largest contour at x={x}, y={y}, width={w}, height={h}")
        
        # Calculate center of frame and target
        target_x = x + w/2
        target_y = y + h/2
        
        # Check if crosshair is inside bounding box
        crosshair_inside = (x <= frame_center_x <= x + w and y <= frame_center_y <= y + h)
        crosshair_color = (0, 0, 255) if crosshair_inside else (0, 255, 0)  # Red if inside, green if outside
        
        # Draw crosshair with appropriate color
        cv2.line(frame, (int(frame_center_x - 10), int(frame_center_y)), 
                (int(frame_center_x + 10), int(frame_center_y)), crosshair_color, 2)
        cv2.line(frame, (int(frame_center_x), int(frame_center_y - 10)), 
                (int(frame_center_x), int(frame_center_y + 10)), crosshair_color, 2)
        
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Send relay command directly if in auto mode
        with command_lock:
            if latest_command.get('auto_mode', False):
                # Send 'off' to activate relay when crosshair is inside box
                client.publish(MQTT_TOPIC_RELAY, RELAY_ACTIVATE if crosshair_inside else RELAY_DEACTIVATE)
        
        # Calculate angle corrections
        pan_correction = (target_x - frame_center_x) * PAN_ANGLE_PER_PIXEL
        tilt_correction = (target_y - frame_center_y) * TILT_ANGLE_PER_PIXEL
        
        logging.info(f"Pixel offsets - x: {target_x - frame_center_x}, y: {target_y - frame_center_y}")
        logging.info(f"Angle corrections - pan: {pan_correction:.2f}, tilt: {tilt_correction:.2f}")
        
        # Update command with corrections
        with command_lock:
            latest_command['pan_correction'] = pan_correction
            latest_command['tilt_correction'] = tilt_correction
        
        return frame, target_x, target_y
    else:
        logging.info("No valid target found in frame")
        # If no target found and in auto mode, deactivate relay
        with command_lock:
            if latest_command.get('auto_mode', False):
                client.publish(MQTT_TOPIC_RELAY, RELAY_DEACTIVATE)
        return frame, None, None

def find_dynamixel_port():
    """Automatically detect the Dynamixel U2D2 controller port."""
    try:
        # List all serial ports
        ports = list(serial.tools.list_ports.comports())
        print(f"Available ports: {[port.device for port in ports]}", flush=True)
        
        # First, try to find U2D2 by its typical USB ID
        for port in ports:
            if "FT232R" in port.description or "U2D2" in port.description:
                print(f"Found potential U2D2 device at {port.device}", flush=True)
                port_handler = PortHandler(port.device)
                if port_handler.openPort():
                    print(f"Opened port {port.device}", flush=True)
                    if port_handler.setBaudRate(BAUDRATE):
                        print(f"Set baud rate to {BAUDRATE}", flush=True)
                        packet_handler = PacketHandler(PROTOCOL_VERSION)
                        # Try to ping both servos
                        for dxl_id in [DXL1_ID, DXL2_ID]:
                            dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, dxl_id)
                            if dxl_comm_result == COMM_SUCCESS:
                                print(f"Successfully pinged Dynamixel ID {dxl_id} on port {port.device}", flush=True)
                                return port_handler, packet_handler
                    port_handler.closePort()

        # If U2D2 not found by description, try /dev/ttyUSB* devices
        for port in ports:
            if "ttyUSB" in port.device:
                print(f"Trying USB device at {port.device}", flush=True)
                port_handler = PortHandler(port.device)
                if port_handler.openPort():
                    print(f"Opened port {port.device}", flush=True)
                    if port_handler.setBaudRate(BAUDRATE):
                        print(f"Set baud rate to {BAUDRATE}", flush=True)
                        packet_handler = PacketHandler(PROTOCOL_VERSION)
                        # Try to ping both servos
                        for dxl_id in [DXL1_ID, DXL2_ID]:
                            dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, dxl_id)
                            if dxl_comm_result == COMM_SUCCESS:
                                print(f"Successfully pinged Dynamixel ID {dxl_id} on port {port.device}", flush=True)
                                return port_handler, packet_handler
                    port_handler.closePort()

        print("Could not find U2D2 device", flush=True)
        return None, None

    except Exception as e:
        print(f"Error finding Dynamixel port: {e}", flush=True)
        import traceback
        print(traceback.format_exc(), flush=True)
        return None, None

def initialize_servos(portHandler, packetHandler):
    """Initialize servo objects"""
    global pan_servo, tilt_servo
    
    # Store packet handler for servo control
    pan_servo = packetHandler
    tilt_servo = packetHandler
    
    # Enable torque
    pan_servo.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    tilt_servo.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    
    logging.info("Servo objects initialized")
    return True

def initialize_servo(dxl_id):
    """Initialize a servo with extended position control mode settings."""
    try:
        # Set operating mode to extended position control mode (multi-turn) for pan
        if dxl_id == DXL1_ID:
            mode = OPERATING_MODE  # Extended position mode for pan
        else:
            mode = 3  # Position control mode for tilt
        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_id, 11, mode)  # Address 11 is operating mode
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set operating mode for ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}", flush=True)
            return False

        # Set to maximum torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, dxl_id, 14, 1023)  # Address 14 is max torque
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set max torque for ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}", flush=True)
            return False

        print(f"Successfully initialized servo ID {dxl_id}", flush=True)
        return True
        
    except Exception as e:
        print(f"Error initializing servo ID {dxl_id}: {str(e)}", flush=True)
        return False

def angle_to_servo_position(servo_id, angle):
    """Convert angle in degrees to servo position value"""
    if servo_id == DXL1_ID:  # Pan servo
        # Convert angle to position (0-4095)
        position = int((angle + 180.0) * (4095.0 / 360.0))
    else:  # Tilt servo
        # Convert angle to position (0-4095) 
        position = int((angle + 180.0) * (4095.0 / 360.0))
    
    # Ensure position is within valid range
    position = max(0, min(4095, position))
    return position

def servo_position_to_angle(dxl_id, position):
    """Convert servo position to angle."""
    try:
        if dxl_id == DXL1_ID:  # Pan servo
            rotations = (position - POSITION_CENTER) / 4096.0
            return rotations * 360.0
        else:  # Tilt servo
            degrees_per_unit = 360.0 / 4096
            angle = (position - POSITION_CENTER) * degrees_per_unit
            return angle  # Remove the -140 offset
    except Exception as e:
        print(f"Error converting position to angle: {e}", flush=True)
        return 0.0

# Add at the top with other globals
last_relay_change = 0
RELAY_DEBOUNCE_TIME = 0.1  # 100ms debounce

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    """Callback when client connects to MQTT broker"""
    print(f"Connected with result code {rc}", flush=True)
    
    # Subscribe to all required topics
    client.subscribe([
        (MQTT_TOPIC_CONTROL, 0),
        (MQTT_TOPIC_RELAY, 0),
        (MQTT_TOPIC_COMMAND, 0),  # Add subscription to command topic
        ("server/sensitivity", 0)
    ])
    
    # Initialize with auto mode off
    global latest_command
    latest_command = {
        'auto_mode': False,
        'pan_angle': HOME_PAN_ANGLE,
        'tilt_angle': HOME_TILT_ANGLE
    }
    logging.info("Connected to MQTT broker and initialized command state")

def on_message(client, userdata, msg):
    """MQTT message callback"""
    global latest_command
    
    try:
        print(f"Server received message on topic: {msg.topic}")  # Debug print
        print(f"Payload: {msg.payload.decode()}")  # Debug print
        
        if msg.topic == MQTT_TOPIC_RELAY:
            if current_platform == 'Linux':
                relay_state = msg.payload.decode()
                print(f"Setting relay to {relay_state}", flush=True)
                if relay_state == RELAY_ACTIVATE:  # 'off' activates the relay
                    GPIO.output(RELAY_PIN, GPIO.LOW)
                    print("Relay ACTIVATED (LOW)", flush=True)
                else:  # 'on' deactivates the relay
                    GPIO.output(RELAY_PIN, GPIO.HIGH)
                    print("Relay DEACTIVATED (HIGH)", flush=True)
        elif msg.topic == MQTT_TOPIC_COMMAND:  # Handle keyboard commands
            command = json.loads(msg.payload.decode())
            print(f"Received command message: {command}")  # Debug print
            
            # Handle relative movement commands
            with command_lock:
                if 'pan_delta' in command:
                    latest_command['pan_angle'] = latest_command.get('pan_angle', 0) + command['pan_delta']
                    print(f"Updated pan angle to: {latest_command['pan_angle']}")  # Debug print
                elif 'tilt_delta' in command:
                    latest_command['tilt_angle'] = latest_command.get('tilt_angle', INITIAL_TILT_ANGLE) + command['tilt_delta']
                    print(f"Updated tilt angle to: {latest_command['tilt_angle']}")  # Debug print
        else:
            command = json.loads(msg.payload.decode())
            with command_lock:
                latest_command.update(command)
            
    except Exception as e:
        logging.error(f"Error processing command: {e}")
        import traceback
        traceback.print_exc()

def camera_feed_thread():
    global latest_frame, running
    
    while running:
        try:
            # Handle keyboard input for manual mode
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # If a key was pressed
                with command_lock:
                    if not latest_command.get('auto_mode', False):  # Only in manual mode
                        if key == ord(' '):  # Spacebar
                            client.publish(MQTT_TOPIC_RELAY, RELAY_ACTIVATE)
                            time.sleep(0.1)  # Brief delay
                            client.publish(MQTT_TOPIC_RELAY, RELAY_DEACTIVATE)
                        elif key == 81 or key == ord('a'):  # Left arrow or 'a'
                            latest_command['pan_angle'] = min(latest_command.get('pan_angle', 0) + 5, PAN_ANGLE_MAX)
                        elif key == 83 or key == ord('d'):  # Right arrow or 'd'
                            latest_command['pan_angle'] = max(latest_command.get('pan_angle', 0) - 5, PAN_ANGLE_MIN)
                        elif key == 82 or key == ord('w'):  # Up arrow or 'w'
                            latest_command['tilt_angle'] = min(latest_command.get('tilt_angle', TILT_ANGLE_MIN) + 5, TILT_ANGLE_MAX)
                        elif key == 84 or key == ord('s'):  # Down arrow or 's'
                            latest_command['tilt_angle'] = max(latest_command.get('tilt_angle', TILT_ANGLE_MAX) - 5, TILT_ANGLE_MIN)
            
            # Get new frame
            if current_platform == 'Darwin':
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read from camera")
                    time.sleep(1)
                    continue
            else:
                in_rgb = q_rgb.tryGet()
                if in_rgb is None:
                    time.sleep(0.001)
                    continue
                frame = in_rgb.getCvFrame()
            
            # Flip frame both horizontally and vertically
            frame = cv2.flip(frame, -1)
            
            # Store frame for other threads to use
            with frame_lock:
                latest_frame = frame.copy()
            
            # Process frame once for green objects and bounding boxes
            processed_frame, target_x, target_y = detect_green_objects(frame.copy())
            
            # Store detection results
            with detection_lock:
                latest_detection = {
                    'frame': processed_frame,
                    'target_x': target_x,
                    'target_y': target_y
                }
            
            # Encode and publish frame
            _, img_encoded = cv2.imencode('.jpg', processed_frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            client.publish(MQTT_TOPIC_CAMERA, img_encoded.tobytes(), qos=0)
            
            time.sleep(0.01)  # 100Hz max update rate
            
        except Exception as e:
            print(f"Error in camera thread: {e}")
            traceback.print_exc()
            time.sleep(1)

def calculate_lead(current_position, target_position, velocity, tof):
    """Calculate the lead based on current position, target position, velocity, and TOF."""
    # Calculate the lead distance
    lead_distance = velocity * tof
    lead_position = target_position + lead_distance
    return lead_position

def calculate_relative_position(current_position, target_position):
    """Calculate the relative position from the current position."""
    return target_position - current_position

def simulate_servo_movement():
    """Simulate servo movement on macOS."""
    global latest_frame, tof
    previous_bbox_center = None
    current_x, current_y = HOME_PAN_ANGLE, HOME_TILT_ANGLE

    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.05)
                continue
            frame = latest_frame.copy()

        bbox = detect_green_objects(frame)
        if bbox:
            bbox_center = (bbox['center_x'], bbox['center_y'])

            # Calculate velocity if previous center is available
            if previous_bbox_center:
                velocity_x = bbox_center[0] - previous_bbox_center[0]
                velocity_y = bbox_center[1] - previous_bbox_center[1]
            else:
                velocity_x, velocity_y = 0, 0

            # Calculate lead positions
            lead_x = calculate_lead(current_x, bbox_center[0], velocity_x, tof)
            lead_y = calculate_lead(current_y, bbox_center[1], velocity_y, tof)

            # Update previous center
            previous_bbox_center = bbox_center

            # Adjust the factor for speed and TOF influence
            current_x += (lead_x - current_x) * 0.1  # Adjust the factor for speed
            current_y += (lead_y - current_y) * 0.1

            # Log the detailed calculations
            logging.debug(f"TOF: {tof}, Pan Angle: {latest_command['pan_angle']}, Tilt Angle: {latest_command['tilt_angle']}")
            logging.debug(f"Velocity X: {velocity_x}, Velocity Y: {velocity_y}")
            logging.debug(f"Lead X: {lead_x}, Lead Y: {lead_y}")
            logging.debug(f"Simulated position: x={current_x}, y={current_y}")

        time.sleep(0.05)

def servo_update_thread():
    """Thread to continuously update servo positions"""
    global latest_command, HOME_PAN_ANGLE, HOME_TILT_ANGLE
    
    try:
        while running:
            with command_lock:
                command = latest_command.copy()
                
                # Get current state and corrections
                auto_mode = command.get('auto_mode', False)
                
                if auto_mode:
                    # Only process corrections in auto mode
                    pan_correction = command.get('pan_correction', 0.0) * 4.0  # Doubled from 2.0 to 4.0 for faster pan
                    tilt_correction = command.get('tilt_correction', 0.0)
                    
                    # Only apply corrections above threshold
                    if abs(pan_correction) < MIN_CORRECTION:
                        pan_correction = 0
                    if abs(tilt_correction) < MIN_CORRECTION:
                        tilt_correction = 0
                        
                    # Calculate target positions (add corrections instead of subtracting)
                    target_pan = current_pan_angle + pan_correction
                    target_tilt = current_tilt_angle + tilt_correction
                    
                    # Clamp targets to valid ranges
                    target_pan = max(PAN_ANGLE_MIN, min(PAN_ANGLE_MAX, target_pan))
                    target_tilt = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, target_tilt))
                    
                    # Calculate movement deltas
                    pan_delta = target_pan - current_pan_angle
                    tilt_delta = target_tilt - current_tilt_angle
                    
                    # Limit movement speed
                    pan_delta = max(-MAX_SPEED, min(MAX_SPEED, pan_delta))
                    tilt_delta = max(-MAX_SPEED, min(MAX_SPEED, tilt_delta))
                    
                    # Update positions with smoothing
                    current_pan_angle += pan_delta * SMOOTHING_FACTOR
                    current_tilt_angle += tilt_delta * SMOOTHING_FACTOR
                    
                    # Send servo positions
                    servo_command = {
                        'pan_angle': current_pan_angle,
                        'tilt_angle': current_tilt_angle
                    }
                    client.publish(MQTT_TOPIC_CONTROL, json.dumps(servo_command))
                    
                    # Send relay command separately
                    relay_state = RELAY_ACTIVATE if command.get('trigger', False) else RELAY_DEACTIVATE
                    client.publish(MQTT_TOPIC_RELAY, relay_state)
                    
                else:
                    # In manual mode or initial state, use commanded angles or stay at home
                    current_pan_angle = command.get('pan_angle', HOME_PAN_ANGLE)
                    current_tilt_angle = command.get('tilt_angle', HOME_TILT_ANGLE)
                    
                    # Send servo positions
                    servo_command = {
                        'pan_angle': current_pan_angle,
                        'tilt_angle': current_tilt_angle
                    }
                    client.publish(MQTT_TOPIC_CONTROL, json.dumps(servo_command))
                    
                    # Always deactivate relay in manual mode
                    client.publish(MQTT_TOPIC_RELAY, RELAY_DEACTIVATE)
                
                # Update command with current positions
                command['pan_angle'] = current_pan_angle
                command['tilt_angle'] = current_tilt_angle
                latest_command.update(command)
            
            if current_platform == 'Linux':
                try:
                    # Convert angles to servo positions
                    pan_position = angle_to_servo_position(DXL1_ID, current_pan_angle)
                    tilt_position = angle_to_servo_position(DXL2_ID, current_tilt_angle)
                    
                    # Write positions to servos
                    pan_result = pan_servo.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, pan_position)
                    tilt_result = tilt_servo.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, tilt_position)
                    
                    if pan_result != COMM_SUCCESS or tilt_result != COMM_SUCCESS:
                        logging.error(f"Failed to write to servos - Pan: {pan_result}, Tilt: {tilt_result}")
                    
                except Exception as e:
                    logging.error(f"Error updating servos: {e}")
            
            time.sleep(1.0 / UPDATE_RATE_HZ)  # Maintain consistent update rate
            
    except Exception as e:
        logging.error(f"Error in servo update thread: {e}")
        traceback.print_exc()

def initialize_home_position():
    """Initialize HOME position based on current servo positions"""
    global HOME_PAN_ANGLE, HOME_TILT_ANGLE, latest_command
    
    # On Linux, read actual servo positions
    if current_platform == 'Linux':
        try:
            # Read current positions from servos
            dxl_comm_result1, dxl_error1, pan_pos = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION)
            dxl_comm_result2, dxl_error2, tilt_pos = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result1 == COMM_SUCCESS and dxl_error1 == 0 and dxl_comm_result2 == COMM_SUCCESS and dxl_error2 == 0:
                # Convert to angles using the same conversion as in servo_position_to_angle
                HOME_PAN_ANGLE = servo_position_to_angle(DXL1_ID, pan_pos)
                HOME_TILT_ANGLE = servo_position_to_angle(DXL2_ID, tilt_pos)
                
                # Initialize latest_command with current positions
                with command_lock:
                    latest_command = {
                        'pan_angle': HOME_PAN_ANGLE,
                        'tilt_angle': HOME_TILT_ANGLE,
                        'auto_mode': False
                    }
            else:
                logging.error("Failed to read servo positions")
                return False
            
        except Exception as e:
            logging.error(f"Error reading servo positions: {e}")
            return False
    else:
        # On macOS, use simulated values
        HOME_PAN_ANGLE = 0.0
        HOME_TILT_ANGLE = -180.0
        with command_lock:
            latest_command = {
                'pan_angle': HOME_PAN_ANGLE,
                'tilt_angle': HOME_TILT_ANGLE,
                'auto_mode': False
            }
    
    logging.info(f"Initialized HOME position to: pan={HOME_PAN_ANGLE}, tilt={HOME_TILT_ANGLE}")
    return True

# Add with other globals
detection_lock = threading.Lock()
latest_detection = None

# Add speed control function
def set_speed(speed_setting):
    """Set servo speed based on UI setting (1-10)"""
    global MAX_SPEED_DEG_PER_SEC, MAX_SPEED
    
    # Map speed setting 1-10 to degrees per second
    # Setting of 3 = 45 deg/sec
    # Setting of 5 = 75 deg/sec (middle)
    # Setting of 10 = 150 deg/sec
    MAX_SPEED_DEG_PER_SEC = speed_setting * 15.0  # 15 deg/sec per unit
    MAX_SPEED = MAX_SPEED_DEG_PER_SEC / UPDATE_RATE_HZ
    logging.info(f"Speed set to {speed_setting} ({MAX_SPEED_DEG_PER_SEC} deg/sec)")

# Main execution
if __name__ == "__main__":
    # Initialize MQTT client with protocol v3
    client = mqtt.Client(client_id="turret_server", protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_message = on_message

    # Set MQTT options for better performance
    client.max_queued_messages_set(1)  # Only queue latest message
    client.max_inflight_messages_set(1)  # Only allow one in-flight message
    
    # Connect to MQTT broker
    try:
        broker_ip = get_local_ip()
        print(f"Connecting to MQTT broker at {broker_ip}:1883", flush=True)
        
        # Connect without properties for now
        client.connect(broker_ip, 1883, 60)
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}", flush=True)
        sys.exit(1)

    # Start MQTT loop in background thread
    client.loop_start()

    # Subscribe to topics with QoS 0
    client.subscribe([(MQTT_TOPIC_CONTROL, 0), 
                     (MQTT_TOPIC_PLATFORM_REQUEST, 0),
                     ("server/sensitivity", 0)])

    # Initialize camera
    if not initialize_camera():
        print("Failed to initialize camera")
        sys.exit(1)

    # Initialize hardware if on Linux
    if current_platform == 'Linux':
        try:
            # Initialize Dynamixel servos
            portHandler, packetHandler = find_dynamixel_port()
            if portHandler is None or packetHandler is None:
                print("Failed to initialize Dynamixel servos", flush=True)
                sys.exit(1)
                
            # Initialize servo objects
            if not initialize_servos(portHandler, packetHandler):
                print("Failed to initialize servo objects", flush=True)
                sys.exit(1)

            # Enable Torque for both servos
            for dxl_id in [DXL1_ID, DXL2_ID]:
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                    portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    logging.error(f"Failed to enable torque on Dynamixel#{dxl_id}: "
                                  f"{packetHandler.getTxRxResult(dxl_comm_result)}, "
                                  f"Error: {packetHandler.getRxPacketError(dxl_error)}")
                    sys.exit()
                else:
                    logging.info(f"Torque enabled on Dynamixel#{dxl_id}")

            # Set torque for tilt servo
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
                portHandler, DXL2_ID, ADDR_GOAL_TORQUE, DXL_GOAL_TORQUE)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                logging.error(f"Failed to set goal torque on Dynamixel#{DXL2_ID}: "
                              f"{packetHandler.getTxRxResult(dxl_comm_result)}, "
                              f"Error: {packetHandler.getRxPacketError(dxl_error)}")
            else:
                logging.info(f"Goal torque set to {DXL_GOAL_TORQUE} on Dynamixel#{DXL2_ID}")

            # Initialize GroupSyncWrite instance
            groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

            # Initialize each servo with proper operating mode
            if not initialize_servo(DXL1_ID) or not initialize_servo(DXL2_ID):
                print("Failed to initialize servos", flush=True)
                sys.exit(1)

            # Initialize latest_command with current positions
            with command_lock:
                latest_command = {}
                for dxl_id in [DXL1_ID, DXL2_ID]:
                    present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                        portHandler, dxl_id, ADDR_PRESENT_POSITION)
                    if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                        angle = servo_position_to_angle(dxl_id, present_position)
                        if dxl_id == DXL1_ID:
                            latest_command['pan_angle'] = angle
                        else:
                            latest_command['tilt_angle'] = angle
                        print(f"Servo ID {dxl_id} current position: {present_position} (angle: {angle}°)", flush=True)

            # Initialize HOME position after hardware is ready
            initialize_home_position()

            # Initialize servo objects
            initialize_servos(portHandler, packetHandler)

            print("Hardware initialization complete", flush=True)

        except Exception as e:
            print(f"Error initializing hardware: {e}", flush=True)
            import traceback
            print(traceback.format_exc(), flush=True)
            sys.exit(1)
    else:
        # Initialize HOME position for macOS
        initialize_home_position()

    # Start threads
    camera_thread = threading.Thread(target=camera_feed_thread)
    camera_thread.daemon = True
    camera_thread.start()

    if current_platform == 'Linux':
        servo_thread = threading.Thread(target=servo_update_thread)
        servo_thread.daemon = True
        servo_thread.start()

    if current_platform == 'Darwin':
        # Start simulated servo movement thread
        simulate_thread = threading.Thread(target=simulate_servo_movement)
        simulate_thread.daemon = True
        simulate_thread.start()

    try:
        print("Main loop started. Camera feed should be publishing.", flush=True)
        while True:
            # Publish servo status periodically (even if virtual on macOS)
            with command_lock:
                servo_status = {
                    'pan_angle': latest_command.get('pan_angle', 0.0),  # Use .get() with default values
                    'tilt_angle': latest_command.get('tilt_angle', 0.0)
                }
            
            try:
                status_result = client.publish(MQTT_TOPIC_SERVO_STATUS, json.dumps(servo_status))
                if status_result.is_published():
                    print(f"Published servo status: {servo_status}", flush=True)
            except Exception as e:
                print(f"Error publishing servo status: {e}", flush=True)
            
            time.sleep(1)  # Publish status every second

    except KeyboardInterrupt:
        print("Shutting down", flush=True)
    finally:
        running = False  # Signal threads to stop
        client.loop_stop()
        client.disconnect()
        if current_platform == 'Linux':
            GPIO.cleanup()
            portHandler.closePort()
