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
import traceback
import math

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

# MQTT Topics - Update these to match the client
MQTT_TOPIC_CONTROL = "dpad/commands"   # Changed to match client
MQTT_TOPIC_CAMERA = "camera/feed"
MQTT_TOPIC_BBOX = "camera/bbox"
MQTT_TOPIC_SERVO_STATUS = "servo/status"
MQTT_TOPIC_PLATFORM = "server/platform"
MQTT_TOPIC_PLATFORM_REQUEST = "server/platform_request"
MQTT_TOPIC_DEPTH = "camera/depth"
MQTT_TOPIC_RELAY = "relay/command"
MQTT_TOPIC_COMMAND = "dpad/commands"   # Changed to match client
RELAY_ACTIVATE = 'off'    # Relay is "normally closed" - sending 'off' activates it
RELAY_DEACTIVATE = 'on'   # Sending 'on' deactivates the normally closed relay

# Add with other MQTT topics at the top

# First define the constants
POSITION_CENTER = 2048     # Center position
HOME_PAN_ANGLE = 0.0      # Default home position for pan (center)
HOME_TILT_ANGLE = 0.0     # Default home position for tilt (center)

# Color detection constants
LOWER_GREEN = np.array([45, 100, 100])    # Increased saturation and value minimums for brighter greens
UPPER_GREEN = np.array([85, 255, 255])    # Keep upper bound the same
MIN_CONTOUR_AREA = 15                     # Keep minimum contour area the same

# Dynamixel Configuration
ADDR_TORQUE_ENABLE = 64               # Control table address for torque enable
ADDR_OPERATING_MODE = 11              # Control table address for operating mode
ADDR_GOAL_POSITION = 116             # Control table address for goal position
ADDR_PRESENT_POSITION = 132          # Control table address for present position
ADDR_GOAL_CURRENT = 102              # Control table address for goal current (torque)
ADDR_RESET = 117                     # Control table address for factory reset
LEN_GOAL_POSITION = 4                # Data length for goal position
DXL1_ID = 1                          # Pan servo ID
DXL2_ID = 2                          # Tilt servo ID
BAUDRATE = 1000000                   # Default baudrate
PROTOCOL_VERSION = 2.0               # Protocol version
TORQUE_ENABLE = 1                    # Value for enabling torque
TORQUE_DISABLE = 0                   # Value for disabling torque
DXL_MOVING_STATUS_THRESHOLD = 20     # Threshold for detecting movement
DXL_GOAL_CURRENT = 500              # Torque limit (0-1193)

# Operating mode settings
OPERATING_MODE = 4         # Extended position control mode (multi-turn)
MAX_POSITION_VALUE = 4095  # Maximum position value
MIN_POSITION_VALUE = 0     # Minimum position value
COMM_SUCCESS = 0          # Communication success result value

# Then define the global variables that use these constants
command_lock = threading.Lock()
frame_lock = threading.Lock()
detection_lock = threading.Lock()

# Rest of the global variables
latest_command = {
    'auto_mode': False,
    'pan_angle': HOME_PAN_ANGLE,
    'tilt_angle': HOME_TILT_ANGLE,
    'command_received': False
}
latest_frame = None
latest_detection = None
running = True
client_connected = False
home_pan_position = None
home_tilt_position = None
tof = 0.0
sensitivity = 1.0
last_relay_change = 0
pan_servo = None
tilt_servo = None
cap = None
q_rgb = None
device = None
current_pan_angle = HOME_PAN_ANGLE
current_tilt_angle = HOME_TILT_ANGLE
current_speed = 0
current_auto_mode = False

# Then the rest of the constants
PAN_ANGLE_MIN = -90.0   # 90 degrees left of center
PAN_ANGLE_MAX = 90.0    # 90 degrees right of center
TILT_ANGLE_MIN = -90.0  # 90 degrees down from center
TILT_ANGLE_MAX = 90.0   # 90 degrees up from center

# Near the top of the file with other constants
PAN_ANGLE_PER_PIXEL = 0.1  # Degrees per pixel for pan
TILT_ANGLE_PER_PIXEL = 0.1  # Degrees per pixel for tilt

# Servo Control Parameters
SMOOTHING_FACTOR = 0.3  # Smoothing factor (0.0-1.0)
MAX_SPEED_DEG_PER_SEC = 360.0  # Doubled again to 360 deg/sec
UPDATE_RATE_HZ = 50  # Control loop update rate
MAX_SPEED = MAX_SPEED_DEG_PER_SEC / UPDATE_RATE_HZ  # Max speed per update
MIN_CORRECTION = 0.1  # Minimum correction to apply (degrees)

# Add these with the other Dynamixel Configuration constants
ADDR_POSITION_P_GAIN = 84    # Control table address for Position P Gain
ADDR_POSITION_I_GAIN = 82    # Control table address for Position I Gain  
ADDR_POSITION_D_GAIN = 80    # Control table address for Position D Gain

# Add these constants at the top with other constants
PAN_MIN_ANGLE = -90.0  # Minimum pan angle in degrees 
PAN_MAX_ANGLE = 90.0   # Maximum pan angle in degrees
TILT_MIN_ANGLE = -45.0 # Minimum tilt angle in degrees
TILT_MAX_ANGLE = 45.0  # Maximum tilt angle in degrees

# Add these variables at the top with other globals
initial_pan_position = None
initial_tilt_position = None

# Servo position limits
PAN_MIN_POSITION = 0       # Changed from 1024 to 0 for maximum left movement
PAN_MAX_POSITION = 4095    # Changed from 3072 to 4095 for maximum right movement
TILT_MIN_POSITION = 200    # Keep current tilt limits
TILT_MAX_POSITION = 3800   # Keep current tilt limits

# Add at the top with other globals
previous_error_x = 0
previous_error_y = 0

# Add with other globals at the top
sensitivity = 1.0  # Default sensitivity

# Move this function up, before servo_update_thread
def angle_to_servo_position(servo_id, angle):
    """Convert angle in degrees to servo position"""
    if servo_id == DXL1_ID:  # Pan servo
        # Limit pan angle
        angle = min(max(angle, PAN_MIN_ANGLE), PAN_MAX_ANGLE)
        return int(2048 + (angle * 2048 / 180.0))
    else:  # Tilt servo
        # Limit tilt angle
        angle = min(max(angle, TILT_MIN_ANGLE), TILT_MAX_ANGLE)
        return int(2048 + (angle * 2048 / 90.0))

def find_dynamixel_port():
    """Find the Dynamixel U2D2 controller port."""
    try:
        ports = list(serial.tools.list_ports.comports())
        print(f"Available ports: {[port.device for port in ports]}", flush=True)
        
        # First try to find U2D2 by USB ID
        for port in ports:
            if "FT232R" in port.description or "U2D2" in port.description:
                print(f"Found potential U2D2 device at {port.device}", flush=True)
                port_handler = PortHandler(port.device)
                if port_handler.openPort():
                    print(f"Opened port {port.device}", flush=True)
                    if port_handler.setBaudRate(BAUDRATE):
                        print(f"Set baud rate to {BAUDRATE}", flush=True)
                        packet_handler = PacketHandler(PROTOCOL_VERSION)
                        for dxl_id in [DXL1_ID, DXL2_ID]:
                            dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, dxl_id)
                            if dxl_comm_result == COMM_SUCCESS:
                                print(f"Successfully pinged Dynamixel ID {dxl_id} on port {port.device}", flush=True)
                                return port_handler, packet_handler
                    port_handler.closePort()

        # If U2D2 not found by description, try ttyUSB devices
        for port in ports:
            if "ttyUSB" in port.device:
                print(f"Trying USB device at {port.device}", flush=True)
                port_handler = PortHandler(port.device)
                if port_handler.openPort():
                    if port_handler.setBaudRate(BAUDRATE):
                        packet_handler = PacketHandler(PROTOCOL_VERSION)
                        for dxl_id in [DXL1_ID, DXL2_ID]:
                            dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, dxl_id)
                            if dxl_comm_result == COMM_SUCCESS:
                                return port_handler, packet_handler
                    port_handler.closePort()

        print("Could not find U2D2 device", flush=True)
        return None, None

    except Exception as e:
        print(f"Error finding Dynamixel port: {e}", flush=True)
        traceback.print_exc()
        return None, None

def initialize_servos(portHandler, packetHandler):
    """Initialize servos and store their initial positions"""
    global initial_pan_position, initial_tilt_position
    
    try:
        # Read control table items to debug servo configuration
        def read_control_table(dxl_id, address, size=1):
            if size == 1:
                result, comm_result, error = packetHandler.read1ByteTxRx(portHandler, dxl_id, address)
            elif size == 2:
                result, comm_result, error = packetHandler.read2ByteTxRx(portHandler, dxl_id, address)
            elif size == 4:
                result, comm_result, error = packetHandler.read4ByteTxRx(portHandler, dxl_id, address)
            if comm_result != COMM_SUCCESS or error != 0:
                logging.error(f"Failed to read address {address} from servo {dxl_id}:")
                if comm_result != COMM_SUCCESS:
                    logging.error(f"Comm Error: {packetHandler.getTxRxResult(comm_result)}")
                if error != 0:
                    logging.error(f"Packet Error: {packetHandler.getRxPacketError(error)}")
                return None
            return result

        # Debug servo 2 (tilt) configuration
        logging.info("Reading tilt servo configuration:")
        operating_mode = read_control_table(DXL2_ID, ADDR_OPERATING_MODE)
        logging.info(f"Operating mode: {operating_mode}")
        
        # Try to read position limits if they exist
        min_pos = read_control_table(DXL2_ID, 52, size=4)  # Min position limit
        max_pos = read_control_table(DXL2_ID, 48, size=4)  # Max position limit
        logging.info(f"Position limits from servo: min={min_pos}, max={max_pos}")
        
        # Initialize pan servo (DXL1_ID)
        logging.info("Initializing servo 1")
        
        # Disable torque to allow configuration
        packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        time.sleep(0.1)
        
        # Set operating mode to extended position control (multi-turn)
        packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING_MODE, 4)
        time.sleep(0.1)
        
        # Set torque limit
        packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_CURRENT, DXL_GOAL_CURRENT)
        time.sleep(0.1)
        
        # Enable torque
        packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        logging.info("Successfully initialized servo 1")

        # Initialize tilt servo (DXL2_ID)
        logging.info("Initializing servo 2")
        
        # Disable torque to allow configuration
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        time.sleep(0.1)
        
        # Set operating mode to extended position control (multi-turn)
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERATING_MODE, 4)
        time.sleep(0.1)
        
        # Try to read current position before setting limits
        current_pos = read_control_table(DXL2_ID, ADDR_PRESENT_POSITION, size=4)
        logging.info(f"Current position before configuration: {current_pos}")
        
        # Set torque limit
        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, DXL_GOAL_CURRENT)
        time.sleep(0.1)
        
        # Enable torque
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        logging.info("Successfully initialized servo 2")

        # Read initial positions
        dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler.read4ByteTxRx(
            portHandler, DXL1_ID, ADDR_PRESENT_POSITION)
        dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler.read4ByteTxRx(
            portHandler, DXL2_ID, ADDR_PRESENT_POSITION)
        
        if dxl_comm_result1 == COMM_SUCCESS and dxl_comm_result2 == COMM_SUCCESS:
            initial_pan_position = dxl_present_position1
            initial_tilt_position = dxl_present_position2
            
            logging.info(f"Initialized servos at positions - Pan: {initial_pan_position}, Tilt: {initial_tilt_position}")
            return True
            
        return False

    except Exception as e:
        logging.error(f"Error in initialize_servos: {e}")
        return False

def initialize_servo(dxl_id):
    """Initialize a servo with proper settings."""
    try:
        # 1. Disable torque to allow setting operating mode
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            logging.error(f"Failed to disable torque on ID {dxl_id}")
            return False

        # 2. Set operating mode (Extended Position for pan, Position for tilt)
        mode = OPERATING_MODE if dxl_id == DXL1_ID else 3  # 4 for Extended Position, 3 for Position
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_id, ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            logging.error(f"Failed to set operating mode on ID {dxl_id}")
            return False

        # 3. Set goal current (torque limit)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, dxl_id, ADDR_GOAL_CURRENT, DXL_GOAL_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            logging.error(f"Failed to set goal current on ID {dxl_id}")
            return False

        # 4. Re-enable torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            logging.error(f"Failed to enable torque on ID {dxl_id}")
            return False

        logging.info(f"Successfully initialized servo ID {dxl_id}")
        return True

    except Exception as e:
        logging.error(f"Error initializing servo ID {dxl_id}: {e}")
        return False

def read_servo_position(dxl_id, retries=3):
    """Read the current position of a servo with retries"""
    for attempt in range(retries):
        try:
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                portHandler, dxl_id, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                logging.debug(f"Successfully read position for servo {dxl_id}: {dxl_present_position}")
                return dxl_present_position
            else:
                logging.warning(f"Failed to read servo {dxl_id} position on attempt {attempt + 1}:")
                logging.warning(f"Comm result: {packetHandler.getTxRxResult(dxl_comm_result)}")
                logging.warning(f"Error: {packetHandler.getRxPacketError(dxl_error)}")
                
        except Exception as e:
            logging.error(f"Error reading servo {dxl_id} position on attempt {attempt + 1}: {e}")
            
        # Small delay before retry
        time.sleep(0.1)
    
    logging.error(f"Failed to read servo {dxl_id} position after {retries} attempts")
    return None

def servo_position_to_angle(servo_id, position):
    """Convert servo position to angle in degrees"""
    if servo_id == DXL1_ID:  # Pan servo
        # Pan servo: 2048 is center (0°), 1024 is -90°, 3072 is +90°
        return (position - 2048) * (180.0 / 2048)
    else:  # Tilt servo
        # Tilt servo: 2048 is 0°, range is -45° to +45°
        return (position - 2048) * (90.0 / 2048)

# Add at the top with other globals
last_relay_change = 0
RELAY_DEBOUNCE_TIME = 0.1  # 100ms debounce

def on_connect(client, userdata, flags, rc):
    """Callback when client connects to MQTT broker"""
    global client_connected
    logging.info(f"Connected with result code {rc}")
    
    if rc == 0:  # Only set connected if connection was successful
        # Subscribe to all required topics
        client.subscribe([
            (MQTT_TOPIC_CONTROL, 0),
            (MQTT_TOPIC_COMMAND, 0),
            (MQTT_TOPIC_RELAY, 0),
            ("server/sensitivity", 0)  # Add subscription for sensitivity adjustments
        ])
        logging.info("Subscribed to control topics")
        
        # Initialize with auto mode off and no commands received
        global latest_command
        latest_command = {
            'auto_mode': False,
            'pan_angle': HOME_PAN_ANGLE,
            'tilt_angle': HOME_TILT_ANGLE,
            'command_received': False
        }
        
        # Set client_connected flag
        client_connected = True
        logging.info("Client connected and initialized")
    else:
        logging.error(f"Failed to connect with result code {rc}")
        client_connected = False

def on_command(client, userdata, message):
    try:
        logging.debug(f"Received message on topic {message.topic}")
        logging.debug(f"Message payload: {message.payload}")
        
        if message.topic == "server/sensitivity":
            handle_sensitivity_update(message)
            return
            
        if message.topic == MQTT_TOPIC_RELAY:
            relay_command = message.payload.decode('utf-8')
            if relay_command == "on":
                trigger_relay()
                logging.info("Relay triggered")
            return

        # For all other commands, expect JSON
        payload = json.loads(message.payload)
        logging.debug(f"Processing command: {payload}")

        with command_lock:
            if 'pan_delta' in payload:
                # Update pan angle directly
                latest_command['pan_angle'] = latest_command.get('pan_angle', 0) + payload['pan_delta']
                latest_command['command_received'] = True
            elif 'tilt_delta' in payload:
                # Update tilt angle directly
                latest_command['tilt_angle'] = latest_command.get('tilt_angle', 0) + payload['tilt_delta']
                latest_command['command_received'] = True
            elif 'auto_mode' in payload:
                latest_command['auto_mode'] = payload['auto_mode']
                latest_command['command_received'] = True

        # Debug the final command state
        logging.debug(f"Current command state: {latest_command}")

    except Exception as e:
        logging.error(f"Error processing command: {e}")
        logging.error(traceback.format_exc())

def handle_sensitivity_update(message):
    """Handle updates to the sensitivity value"""
    global sensitivity
    try:
        payload = json.loads(message.payload)
        sensitivity = payload.get('sensitivity', 1.0)
        print(f"Updated sensitivity to: {sensitivity}", flush=True)
    except Exception as e:
        print(f"Error processing sensitivity update: {e}", flush=True)

def on_disconnect(client, userdata, rc):
    """Callback when client disconnects from MQTT broker"""
    global client_connected
    client_connected = False
    print(f"Client disconnected with result code {rc}", flush=True)

# Move the initialize_camera() function before the main execution block
def initialize_camera():
    """Initialize the camera based on platform."""
    global cap, q_rgb, device
    
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
                camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                camRgb.setPreviewSize(640, 480)
                camRgb.setInterleaved(False)
                camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                camRgb.setFps(30)
                
                # Camera controls
                camRgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
                camRgb.initialControl.setAutoExposureEnable()
                camRgb.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)

                # Linking
                camRgb.preview.link(xoutRgb.input)

                # Connect to device and start pipeline
                try:
                    found_devices = dai.Device.getAllAvailableDevices()
                    if len(found_devices) == 0:
                        print("No devices found!")
                        return False
                    
                    for device_info in found_devices:
                        print(f"Found device: {device_info.getMxId()} {device_info.state}")
                    
                    device = dai.Device(pipeline)
                    print(f"Connected to {device.getMxId()}")
                    
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

# Move get_local_ip() before the main execution block, right after the global variables

def get_local_ip():
    """Get local IP address for MQTT broker."""
    if current_platform == 'Darwin':
        return '10.42.0.1'
    
    try:
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

# Move initialize_home_position() before the main execution block
def initialize_home_position():
    """Initialize home position based on current servo positions at boot"""
    global HOME_PAN_ANGLE, HOME_TILT_ANGLE, home_pan_position, home_tilt_position, latest_command
    
    try:
        if current_platform == 'Linux':
            # Use the initial positions we already read
            home_pan_position = initial_pan_position
            home_tilt_position = initial_tilt_position
            
            # Set logical angles to 0,0 (this is our reference point)
            HOME_PAN_ANGLE = 0.0
            HOME_TILT_ANGLE = 0.0
            
            # Initialize latest_command with home positions
            with command_lock:
                latest_command = {
                    'pan_angle': HOME_PAN_ANGLE,
                    'tilt_angle': HOME_TILT_ANGLE,
                    'auto_mode': False
                }
                
                # Also initialize current angles
                global current_pan_angle, current_tilt_angle
                current_pan_angle = HOME_PAN_ANGLE
                current_tilt_angle = HOME_TILT_ANGLE
            
            logging.info(f"Initialized logical home position to: pan={HOME_PAN_ANGLE}, tilt={HOME_TILT_ANGLE}")
            logging.info(f"Physical home positions - Pan: {home_pan_position}, Tilt: {home_tilt_position}")
            return True
            
        else:
            # For non-Linux platforms, use center position
            home_pan_position = POSITION_CENTER
            home_tilt_position = POSITION_CENTER
            return True
            
    except Exception as e:
        logging.error(f"Error initializing home position: {e}")
        return False

# Move these thread functions before the main execution block

def trigger_relay():
    """Trigger the relay with debounce protection"""
    global last_relay_change
    
    if current_platform != 'Linux':
        return
        
    current_time = time.time()
    if current_time - last_relay_change > RELAY_DEBOUNCE_TIME:
        try:
            GPIO.output(RELAY_PIN, GPIO.LOW)  # Activate relay
            time.sleep(0.1)  # Brief delay
            GPIO.output(RELAY_PIN, GPIO.HIGH)  # Deactivate relay
            last_relay_change = current_time
            logging.debug("Relay triggered")
        except Exception as e:
            logging.error(f"Error triggering relay: {e}")

def detect_green_objects(frame):
    """Detect green objects in the frame with target persistence."""
    global last_valid_target, target_lost_time
    
    height, width = frame.shape[:2]
    frame_center_x = width / 2
    frame_center_y = height / 2
    
    try:
        # Pre-process to handle bright lighting
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Multiple HSV ranges to catch different shades of green
        # Main neon green range
        NEON_GREEN_LOW = np.array([35, 50, 50])
        NEON_GREEN_HIGH = np.array([85, 255, 255])
        
        # Bright/washed out green range
        BRIGHT_GREEN_LOW = np.array([35, 30, 150])
        BRIGHT_GREEN_HIGH = np.array([85, 120, 255])
        
        # Create masks for each range and combine
        mask1 = cv2.inRange(frame_hsv, NEON_GREEN_LOW, NEON_GREEN_HIGH)
        mask2 = cv2.inRange(frame_hsv, BRIGHT_GREEN_LOW, BRIGHT_GREEN_HIGH)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Noise reduction based on sensitivity
        # Higher sensitivity = less noise reduction
        kernel_size = max(2, int(5 / sensitivity))  # Smaller kernel for higher sensitivity
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Adjust minimum area based on sensitivity
        min_area = int(100 / (sensitivity * 2))  # Higher sensitivity = detect smaller targets
        valid_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
        
        current_time = time.time()
        
        if valid_contours:
            # Sort contours by area
            valid_contours.sort(key=cv2.contourArea, reverse=True)
            
            # If we have a previous valid target, try to find the closest matching contour
            if hasattr(detect_green_objects, 'last_target'):
                last_x, last_y, last_w, last_h = detect_green_objects.last_target
                last_center = (last_x + last_w/2, last_y + last_h/2)
                
                # Find contour closest to last known position
                min_dist = float('inf')
                best_contour = None
                
                for contour in valid_contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    center = (x + w/2, y + h/2)
                    dist = ((center[0] - last_center[0])**2 + (center[1] - last_center[1])**2)**0.5
                    
                    # Only consider contours within reasonable distance
                    if dist < min(width/4, height/4):  # Max tracking distance
                        if dist < min_dist:
                            min_dist = dist
                            best_contour = contour
                
                if best_contour is not None:
                    largest_contour = best_contour
                else:
                    largest_contour = valid_contours[0]
            else:
                largest_contour = valid_contours[0]
            
            x, y, w, h = cv2.boundingRect(largest_contour)
            detect_green_objects.last_target = (x, y, w, h)
            detect_green_objects.last_target_time = current_time
            
            # Calculate target center
            target_center_x = x + w/2
            target_center_y = y + h/2
            
            # Calculate normalized error based on true center
            error_x = (target_center_x - frame_center_x) / (width/2)
            error_y = (target_center_y - frame_center_y) / (height/2)
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Check if frame center is inside bounding box for trigger control
            crosshair_in_box = (x < frame_center_x < (x + w)) and (y < frame_center_y < (y + h))
            
            with command_lock:
                if crosshair_in_box and latest_command.get('auto_mode', False):
                    trigger_relay()
            
            return frame, error_x, error_y
            
        else:
            # Clear tracking if target lost for more than 0.5 seconds
            if (hasattr(detect_green_objects, 'last_target_time') and 
                current_time - detect_green_objects.last_target_time > 0.5):
                if hasattr(detect_green_objects, 'last_target'):
                    delattr(detect_green_objects, 'last_target')
            
        return frame, None, None
        
    except Exception as e:
        logging.error(f"Error in detect_green_objects: {e}")
        logging.error(traceback.format_exc())
        return frame, None, None

def camera_feed_thread():
    """Thread to handle camera feed and processing."""
    global latest_frame, running, latest_detection
    
    while running:
        try:
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
            
            # Draw small green crosshair at frame center
            height, width = processed_frame.shape[:2]
            center_x = int(width/2)
            center_y = int(height/2)
            crosshair_size = 10  # Small 10-pixel lines
            cv2.line(processed_frame, (center_x - crosshair_size, center_y), 
                    (center_x + crosshair_size, center_y), (0, 255, 0), 1)
            cv2.line(processed_frame, (center_x, center_y - crosshair_size), 
                    (center_x, center_y + crosshair_size), (0, 255, 0), 1)
            
            # Store detection results
            with detection_lock:
                latest_detection = {
                    'frame': processed_frame,
                    'target_x': target_x,
                    'target_y': target_y
                }
            
            # Encode and publish frame with reduced quality for better performance
            _, img_encoded = cv2.imencode('.jpg', processed_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            client.publish(MQTT_TOPIC_CAMERA, img_encoded.tobytes(), qos=0)
            
            # Add a small delay to control frame rate
            time.sleep(0.033)  # ~30fps
            
        except Exception as e:
            logging.error(f"Error in camera thread: {e}")
            logging.error(traceback.format_exc())
            time.sleep(1)

def control_servos():
    try:
        global previous_error_x, previous_error_y
        daisy_pattern_time = 0
        daisy_radius = 0.25  # Reduced from 2.0 to 0.25 degrees - much smaller pattern
        daisy_period = 4.0  # Increased from 2.0 to 4.0 seconds - slower pattern
        time_near_target = 0
        last_target_time = time.time()
        
        while running:
            # Get current positions
            dxl_present_position1 = read_servo_position(DXL1_ID)
            dxl_present_position2 = read_servo_position(DXL2_ID)
            
            if dxl_present_position1 is not None and dxl_present_position2 is not None:
                with command_lock:
                    if 'command_received' in latest_command and latest_command['command_received']:
                        if latest_command.get('auto_mode', False):
                            with detection_lock:
                                if latest_detection is None:
                                    target_pan_position = initial_pan_position
                                    target_tilt_position = initial_tilt_position
                                    time_near_target = 0  # Reset time when no target
                                else:
                                    error_x = latest_detection.get('target_x')
                                    error_y = latest_detection.get('target_y')
                                    target_detected = error_x is not None and error_y is not None
                                    
                                    if target_detected:
                                        current_time = time.time()
                                        
                                        # Even smaller deadzone for precise centering
                                        pan_deadzone = 0.005  # Reduced from 0.01 for tighter centering
                                        tilt_deadzone = 0.008  # Reduced from 0.015 for tighter centering
                                        
                                        # Check if we're close to the target
                                        if abs(error_x) < pan_deadzone and abs(error_y) < tilt_deadzone:
                                            if time_near_target == 0:
                                                time_near_target = current_time
                                            elif current_time - time_near_target > 1.0:
                                                # More aggressive fine-tuning multiplier for final centering
                                                error_x *= 2.0  # Increased from 1.5
                                                error_y *= 1.5  # Increased from 1.3
                                        else:
                                            time_near_target = 0
                                            
                                        # Movement adjustments with finer control
                                        if abs(error_x) > pan_deadzone:
                                            pan_adjustment = error_x * 180  # Doubled from 90 for faster pan
                                        else:
                                            # More precise movements when close
                                            pan_adjustment = error_x * 90  # Doubled from 45 for faster fine adjustments
                                            
                                        if abs(error_y) > tilt_deadzone:
                                            tilt_adjustment = error_y * 20  # Keep tilt speed
                                        else:
                                            # More precise movements when close
                                            tilt_adjustment = error_y * 10  # Keep tilt fine adjustment speed
                                        
                                        # Increased maximum pan adjustment
                                        max_pan_adjustment = 100  # Doubled from 50
                                        max_tilt_adjustment = 25  # Kept the same
                                        
                                        pan_delta = int(pan_adjustment * 2048 / 180.0)
                                        pan_delta = max(min(pan_delta, max_pan_adjustment), -max_pan_adjustment)
                                        target_pan_position = dxl_present_position1 + pan_delta
                                        
                                        tilt_delta = int(tilt_adjustment * 2048 / 180.0)
                                        tilt_delta = max(min(tilt_delta, max_tilt_adjustment), -max_tilt_adjustment)
                                        
                                        # Add movement dampening for tilt
                                        if hasattr(control_servos, 'last_tilt_delta'):
                                            # If direction changed, reduce movement
                                            if (tilt_delta * control_servos.last_tilt_delta) < 0:
                                                tilt_delta *= 0.5  # Reduce movement by half when changing directions
                                        control_servos.last_tilt_delta = tilt_delta
                                        
                                        target_tilt_position = dxl_present_position2 + tilt_delta
                                        
                                        logging.debug(f"Auto mode - Errors: x={error_x:.3f}, y={error_y:.3f}")
                                        logging.debug(f"Adjustments - Pan: {pan_adjustment:.2f}°, Tilt: {tilt_adjustment:.2f}°")
                                        last_target_time = current_time
                                    else:
                                        target_pan_position = initial_pan_position
                                        target_tilt_position = initial_tilt_position
                                        time_near_target = 0
                        else:
                            # Manual control logic
                            logging.debug("Manual mode active")
                            target_pan_position = initial_pan_position + int(latest_command.get('pan_angle', 0) * 2048 / 180.0)
                            target_tilt_position = initial_tilt_position + int(latest_command.get('tilt_angle', 0) * 2048 / 180.0)
                            logging.debug(f"Manual mode - Target angles: Pan={latest_command.get('pan_angle', 0)}, Tilt={latest_command.get('tilt_angle', 0)}")

                        # Apply position limits with more logging
                        original_pan = target_pan_position
                        target_pan_position = max(PAN_MIN_POSITION, min(PAN_MAX_POSITION, target_pan_position))
                        if original_pan != target_pan_position:
                            logging.debug(f"Pan position limited from {original_pan} to {target_pan_position}")
                        logging.debug(f"Writing pan position: {target_pan_position}")
                        write_servo_position(DXL1_ID, target_pan_position)

                        original_tilt = target_tilt_position
                        target_tilt_position = max(TILT_MIN_POSITION, min(TILT_MAX_POSITION, target_tilt_position))
                        if original_tilt != target_tilt_position:
                            logging.debug(f"Tilt position limited from {original_tilt} to {target_tilt_position}")
                        logging.debug(f"Writing tilt position: {target_tilt_position}")
                        write_servo_position(DXL2_ID, target_tilt_position)

            time.sleep(0.01)  # Small delay to prevent CPU overload

    except Exception as e:
        logging.error(f"Error in control loop: {e}")
        logging.error(traceback.format_exc())
        # Restart the control loop if it crashes
        time.sleep(1)
        control_servos()

def write_servo_position(dxl_id, position, retries=3):
    """Write position to servo with retries"""
    for attempt in range(retries):
        try:
            original_position = position
            if dxl_id == DXL1_ID:  # Pan servo valid range 0–4095
                position = max(PAN_MIN_POSITION, min(position, PAN_MAX_POSITION))
                logging.debug(f"Pan position before/after limits: {original_position}/{position}")
            else:  # Tilt servo
                position = max(TILT_MIN_POSITION, min(position, TILT_MAX_POSITION))
                logging.debug(f"Tilt position before/after limits: {original_position}/{position}")
                logging.debug(f"Tilt servo write attempt {attempt + 1}:")
                logging.debug(f"  Initial position: {initial_tilt_position}")
                logging.debug(f"  Current limits: TILT_MIN_POSITION={TILT_MIN_POSITION}, TILT_MAX_POSITION={TILT_MAX_POSITION}")
                logging.debug(f"  Writing position value: {position}")
            # Use 4-byte write for both servos in extended position mode
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, dxl_id, ADDR_GOAL_POSITION, position)
            
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                logging.debug(f"Successfully wrote position {position} to servo {dxl_id}")
                return True
            else:
                logging.warning(f"Failed to write servo {dxl_id} position on attempt {attempt + 1}:")
                logging.warning(f"Comm result: {packetHandler.getTxRxResult(dxl_comm_result)}")
                logging.warning(f"Error: {packetHandler.getRxPacketError(dxl_error)}")
                logging.warning(f"Position value that failed: {position}")
                
        except Exception as e:
            logging.error(f"Error writing servo {dxl_id} position on attempt {attempt + 1}: {e}")
            
        # Small delay before retry
        time.sleep(0.1)
    
    logging.error(f"Failed to write servo {dxl_id} position after {retries} attempts")
    return False

# Then the main execution block
if __name__ == "__main__":
    # Initialize MQTT client with protocol v3
    client = mqtt.Client(client_id="turret_server", protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_message = on_command
    client.on_disconnect = on_disconnect
    
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
                
            # Initialize servos
            if not initialize_servos(portHandler, packetHandler):
                print("Failed to initialize servos", flush=True)
                sys.exit(1)

            # Initialize home position
            if not initialize_home_position():
                print("Warning: Using default home position values", flush=True)

            print("Hardware initialization complete", flush=True)

        except Exception as e:
            print(f"Error initializing hardware: {e}", flush=True)
            traceback.print_exc()
            sys.exit(1)
    else:
        # Initialize HOME position for macOS
        initialize_home_position()

    # Start threads
    camera_thread = threading.Thread(target=camera_feed_thread)
    camera_thread.daemon = True
    camera_thread.start()

    if current_platform == 'Linux':
        servo_thread = threading.Thread(target=control_servos)
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
