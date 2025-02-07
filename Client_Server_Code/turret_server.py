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
PAN_MIN_ANGLE = -540.0  # Triple range for pan (1.5 full rotations)
PAN_MAX_ANGLE = 540.0   # Triple range for pan (1.5 full rotations)
TILT_MIN_ANGLE = -45.0  # Keep current tilt angle limits
TILT_MAX_ANGLE = 45.0   # Keep current tilt angle limits

# Near the top of the file with other constants
PAN_ANGLE_PER_PIXEL = 0.2  # Increased for faster response
TILT_ANGLE_PER_PIXEL = 0.1  # Keep tilt speed the same

# Servo Control Parameters
MAX_SPEED_DEG_PER_SEC = 720.0  # Increased for faster pan movement
UPDATE_RATE_HZ = 50  # Control loop update rate
MAX_SPEED = MAX_SPEED_DEG_PER_SEC / UPDATE_RATE_HZ  # Max speed per update
MIN_CORRECTION = 0.1  # Minimum correction to apply (degrees)

# Add these with the other Dynamixel Configuration constants
ADDR_POSITION_P_GAIN = 84    # Control table address for Position P Gain
ADDR_POSITION_I_GAIN = 82    # Control table address for Position I Gain  
ADDR_POSITION_D_GAIN = 80    # Control table address for Position D Gain

# Add these constants at the top with other constants
PAN_MIN_POSITION = -6144     # Restored extended range for pan (1.5 rotations CCW)
PAN_MAX_POSITION = 6144      # Restored extended range for pan (1.5 rotations CW)
TILT_MIN_POSITION = 0        # Keep standard range for tilt
TILT_MAX_POSITION = 4095     # Keep standard range for tilt

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
        # Convert angle to position (4095 per 360 degrees)
        return int(2048 + (angle * 4095 / 360.0))
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
                # Handle overflow for values near max uint32
                if dxl_present_position > 0xFFFF0000:  # If value is close to uint32 max
                    dxl_present_position = dxl_present_position - 0xFFFFFFFF - 1  # Convert to negative
                
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
        # Pan servo: 2048 is center (0°), multi-turn enabled
        return (position - 2048) * (360.0 / 4095)
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
            # Handle keyboard commands
            if message.topic == MQTT_TOPIC_COMMAND:
                if 'pan_delta' in payload:
                    # Get current pan angle, defaulting to 0 if not set
                    current_pan = latest_command.get('pan_angle', 0)
                    # Add the delta and store back
                    latest_command['pan_angle'] = current_pan + payload['pan_delta']
                    latest_command['command_received'] = True
                    logging.info(f"Pan delta {payload['pan_delta']} applied, new angle: {latest_command['pan_angle']}")
                if 'tilt_delta' in payload:
                    # Get current tilt angle, defaulting to 0 if not set
                    current_tilt = latest_command.get('tilt_angle', 0)
                    # Add the delta (reduced to 1/8) and store back
                    latest_command['tilt_angle'] = current_tilt + (payload['tilt_delta'] * 0.125)  # Reduced to 1/8
                    latest_command['command_received'] = True
                    logging.info(f"Tilt delta {payload['tilt_delta'] * 0.125} applied, new angle: {latest_command['tilt_angle']}")
                if 'auto_mode' in payload:
                    latest_command['auto_mode'] = payload['auto_mode']
                    latest_command['command_received'] = True
                    logging.info(f"Auto mode set to {payload['auto_mode']} from command topic")

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
                
                # Camera controls - Critical for LED detection
                camRgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.OFF)  # Turn off autofocus
                camRgb.initialControl.setManualFocus(130)  # Set to a reasonable fixed focus distance
                camRgb.initialControl.setManualExposure(1000, 800)  # 1ms exposure to make LEDs pop more
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
    """Detect green objects in the frame with target persistence and multi-target tracking."""
    # Add class variables for target persistence and multi-target tracking
    if not hasattr(detect_green_objects, 'current_target_center'):
        detect_green_objects.current_target_center = None
    if not hasattr(detect_green_objects, 'target_lock_time'):
        detect_green_objects.target_lock_time = 0
    if not hasattr(detect_green_objects, 'target_switch_cooldown'):
        detect_green_objects.target_switch_cooldown = 0
    if not hasattr(detect_green_objects, 'secondary_targets'):
        detect_green_objects.secondary_targets = []
    if not hasattr(detect_green_objects, 'time_near_target'):
        detect_green_objects.time_near_target = 0
    if not hasattr(detect_green_objects, 'last_valid_detection_time'):
        detect_green_objects.last_valid_detection_time = 0
    
    height, width = frame.shape[:2]
    frame_center_x = width / 2
    frame_center_y = height / 2
    
    try:
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Optimized HSV ranges for LED detection
        NEON_GREEN_LOW = np.array([45, 50, 150])  # Higher value minimum for LED brightness
        NEON_GREEN_HIGH = np.array([85, 255, 255])
        
        # Create mask
        mask = cv2.inRange(frame_hsv, NEON_GREEN_LOW, NEON_GREEN_HIGH)
        
        # Find individual LED points
        kernel = np.ones((3, 3), np.uint8)
        led_mask = cv2.dilate(mask, kernel, iterations=1)
        
        # Find LED contours
        led_contours, _ = cv2.findContours(led_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter LED contours by minimum area
        min_led_area = max(5, int(10 / (sensitivity * 2)))
        valid_leds = [c for c in led_contours if cv2.contourArea(c) >= min_led_area]
        
        logging.debug(f"Found {len(valid_leds)} valid LEDs")
        
        if len(valid_leds) >= 2:  # Need at least 2 LEDs to form a target
            # Get centers of all valid LEDs
            led_centers = []
            for contour in valid_leds:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    led_centers.append((cx, cy))
            
            # Group LED centers into potential targets
            targets = []
            used_centers = set()
            
            for i, center1 in enumerate(led_centers):
                if i in used_centers:
                    continue
                    
                group = [center1]
                used_centers.add(i)
                
                # Find nearby LEDs within reasonable distance
                for j, center2 in enumerate(led_centers):
                    if j in used_centers or j == i:
                        continue
                    
                    # Check if this LED is close to any LED in the group
                    for group_center in group:
                        dist = np.sqrt((center2[0] - group_center[0])**2 + 
                                     (center2[1] - group_center[1])**2)
                        # Increased distance threshold for more lenient grouping
                        if dist < 60:  # Increased from 40 to 60 pixels for more lenient grouping
                            if len(group) < 5:  # Still maintain max of 5 LEDs
                                group.append(center2)
                                used_centers.add(j)
                                break
                
                if len(group) >= 3:  # Accept groups with 3 or more LEDs (changed from requiring exactly 5)
                    # Calculate group properties
                    center_x = int(np.mean([c[0] for c in group]))
                    center_y = int(np.mean([c[1] for c in group]))
                    
                    # Check if LEDs form a roughly circular pattern
                    distances_to_center = [np.sqrt((c[0] - center_x)**2 + (c[1] - center_y)**2) 
                                         for c in group]
                    avg_distance = np.mean(distances_to_center)
                    distance_variance = np.std(distances_to_center)
                    
                    # Adjust circularity requirements based on LED count
                    max_variance_ratio = {
                        3: 0.7,  # More lenient for 3 LEDs
                        4: 0.6,  # Moderately strict for 4 LEDs
                        5: 0.5   # Most strict for 5 LEDs
                    }[len(group)]
                    
                    # More lenient circularity check based on LED count
                    if distance_variance < avg_distance * max_variance_ratio:
                        # Calculate angles between LEDs to verify spacing
                        angles = []
                        for led_pos in group:
                            dx = led_pos[0] - center_x
                            dy = led_pos[1] - center_y
                            angle = math.atan2(dy, dx)
                            angles.append(angle)
                        
                        # Sort angles and calculate differences
                        angles.sort()
                        expected_angle = 2 * math.pi / len(group)  # Adjust expected angle based on LED count
                        angle_diffs = [(angles[(i+1)%len(group)] - angles[i]) % (2*math.pi) for i in range(len(group))]
                        avg_angle_diff = sum(angle_diffs) / len(group)
                        angle_variance = sum((diff - expected_angle)**2 for diff in angle_diffs) / len(group)
                        
                        # Adjust angle spacing requirements based on LED count
                        max_angle_variance = {
                            3: 0.8,  # More lenient for 3 LEDs
                            4: 0.6,  # Moderately strict for 4 LEDs
                            5: 0.5   # Most strict for 5 LEDs
                        }[len(group)]
                        
                        if angle_variance < max_angle_variance:
                            # Calculate confidence based on LED count and pattern quality
                            led_count_weight = len(group) / 5.0  # Base confidence on LED count
                            circularity_weight = 1.0 - (distance_variance / (avg_distance * max_variance_ratio))
                            angle_weight = 1.0 - (angle_variance / max_angle_variance)
                            
                            # Combined confidence score
                            confidence = (led_count_weight * 0.4 +  # 40% weight on LED count
                                        circularity_weight * 0.3 +  # 30% weight on circularity
                                        angle_weight * 0.3)        # 30% weight on angle spacing
                            
                            targets.append({
                                'center': (center_x, center_y),
                                'led_count': len(group),
                                'spread': max(distances_to_center),
                                'led_positions': group,
                                'circularity': distance_variance / avg_distance,
                                'angle_variance': angle_variance,
                                'confidence': confidence
                            })
                            # Don't break here - continue looking for other potential targets
            
            logging.debug(f"Found {len(targets)} valid targets")
            
            if targets:
                current_time = time.time()
                detect_green_objects.last_valid_detection_time = current_time
                
                # Sort targets by LED count (closer to 5) and circularity
                targets.sort(key=lambda x: (abs(5 - x['led_count']), x['circularity']))
                
                # Track primary and secondary targets
                primary_target = None
                secondary_targets = []
                
                # If we have a current target, try to maintain it
                if detect_green_objects.current_target_center is not None:
                    prev_x, prev_y = detect_green_objects.current_target_center
                    
                    # Look for current target in new targets
                    for target in targets:
                        tx, ty = target['center']
                        dist = np.sqrt((tx - prev_x)**2 + (ty - prev_y)**2)
                        
                        if dist < 100:  # Increased tracking window for better persistence
                            # Update with less smoothing for more responsive tracking
                            smooth_factor = 0.3
                            new_x = int(prev_x * smooth_factor + tx * (1 - smooth_factor))
                            new_y = int(prev_y * smooth_factor + ty * (1 - smooth_factor))
                            primary_target = target
                            primary_target['center'] = (new_x, new_y)
                            logging.debug(f"Maintaining current target at ({new_x}, {new_y})")
                            break
                
                # If no primary target found or if we don't have one yet
                if primary_target is None:
                    # Only switch targets if cooldown expired
                    if current_time - detect_green_objects.target_switch_cooldown > 0.5:
                        # Pick the best target (closest to 5 LEDs and most circular)
                        primary_target = targets[0]
                        detect_green_objects.target_switch_cooldown = current_time
                        detect_green_objects.current_target_center = primary_target['center']
                        logging.debug(f"Switching to new target at {primary_target['center']}")
                
                # Update current target and store secondary targets
                if primary_target is not None:
                    detect_green_objects.current_target_center = primary_target['center']
                    
                    # Store other targets as secondary, sorted by LED count and distance to center
                    secondary_targets = [t for t in targets if t != primary_target]
                    if secondary_targets:
                        for target in secondary_targets:
                            tx, ty = target['center']
                            target['distance_to_center'] = np.sqrt((tx - frame_center_x)**2 + 
                                                                 (ty - frame_center_y)**2)
                        secondary_targets.sort(key=lambda x: (-x['led_count'], x['distance_to_center']))
                    detect_green_objects.secondary_targets = secondary_targets
                    
                    # Draw all targets with different colors and numbers
                    for i, target in enumerate(targets):
                        is_primary = (target == primary_target)
                        if is_primary:
                            color = (0, 255, 0)  # Green for primary
                            thickness = 2
                            label = "Target 1"
                        else:
                            color = (0, 165, 255)  # Orange for secondary
                            thickness = 2
                            label = f"Target {i+2}"  # Number the secondary targets
                        
                        # Draw circle at center
                        cv2.circle(frame, target['center'], 20, color, thickness)
                        
                        # Draw small circles at each LED position
                        for led_pos in target['led_positions']:
                            cv2.circle(frame, led_pos, 5, color, 1)
                        
                        # Draw target number and LED count
                        cv2.putText(frame, f"{label} ({target['led_count']} LEDs)", 
                                  (target['center'][0] - 40, target['center'][1] - 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Calculate error for primary target
                    center_x, center_y = primary_target['center']
                    # Add a small correction factor (0.015 = 1.5% of frame width) to compensate for leftward bias
                    error_x = ((center_x - frame_center_x) / (width/2)) + 0.015
                    error_y = (center_y - frame_center_y) / (height/2)
                    
                    logging.debug(f"Target errors: x={error_x:.3f}, y={error_y:.3f}")
                    
                    return frame, error_x, error_y
        
        # Reset if no valid targets found
        current_time = time.time()
        if current_time - detect_green_objects.last_valid_detection_time > 2.0:
            detect_green_objects.current_target_center = None
            detect_green_objects.secondary_targets = []
            logging.debug("No valid targets found - resetting tracking")
        else:
            logging.debug("No targets in this frame, but maintaining last known position")
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
        time_near_target = 0
        last_target_time = time.time()
        
        # Initialize target positions
        target_pan_position = initial_pan_position
        target_tilt_position = initial_tilt_position
        
        # Add variables for tracking error changes
        if not hasattr(control_servos, 'prev_error_x'):
            control_servos.prev_error_x = 0
        if not hasattr(control_servos, 'prev_error_y'):
            control_servos.prev_error_y = 0
            
        # Default frame dimensions (will be updated from actual frame)
        width = 640  # Default width
        height = 480  # Default height
        
        while running:
            # Get current positions
            dxl_present_position1 = read_servo_position(DXL1_ID)
            dxl_present_position2 = read_servo_position(DXL2_ID)
            
            if dxl_present_position1 is not None and dxl_present_position2 is not None:
                with command_lock:
                    if 'command_received' in latest_command and latest_command['command_received']:
                        if latest_command.get('auto_mode', False):
                            with detection_lock:
                                if latest_detection is None or 'frame' not in latest_detection:
                                    target_pan_position = initial_pan_position
                                    target_tilt_position = initial_tilt_position
                                    time_near_target = 0
                                    control_servos.prev_error_x = 0
                                    control_servos.prev_error_y = 0
                                else:
                                    error_x = latest_detection.get('target_x')
                                    error_y = latest_detection.get('target_y')
                                    frame = latest_detection['frame']
                                    height, width = frame.shape[:2]  # Update frame dimensions
                                    target_detected = error_x is not None and error_y is not None
                                    
                                    if target_detected:
                                        current_time = time.time()
                                        
                                        # Calculate error derivatives (rate of change)
                                        error_dx = error_x - control_servos.prev_error_x
                                        error_dy = error_y - control_servos.prev_error_y
                                        
                                        # Update previous errors
                                        control_servos.prev_error_x = error_x
                                        control_servos.prev_error_y = error_y
                                        
                                        # Very small deadzones for precise centering
                                        pan_deadzone = 0.0005  # Reduced from 0.002 for much more precise centering
                                        tilt_deadzone = 0.0005  # Reduced from 0.002 for much more precise centering
                                        
                                        # Check if crosshair is inside the target circle (20 pixel radius)
                                        # Convert normalized errors back to pixels
                                        error_x_pixels = error_x * (width/2)
                                        error_y_pixels = error_y * (height/2)
                                        distance_to_target = np.sqrt(error_x_pixels**2 + error_y_pixels**2)
                                        
                                        # Separate targeting from firing
                                        # Keep moving to center even when within firing range
                                        if distance_to_target < 20:  # Close enough to fire
                                            if time_near_target == 0:
                                                time_near_target = current_time
                                            elif current_time - time_near_target > 0.05:  # Quick 50ms confirmation
                                                trigger_relay()  # Fire when inside circle!
                                                time_near_target = current_time + 0.1  # Small cooldown after shooting
                                        else:
                                            time_near_target = 0
                                        
                                        # Adaptive speed and damping based on error magnitude
                                        if abs(error_x) > pan_deadzone:
                                            # Base speed calculation with adaptive speeds
                                            if abs(error_x) > 0.3:  # Far from target
                                                base_speed = 800
                                                damping_factor = 0.3  # Less damping when far
                                            elif abs(error_x) > 0.1:  # Medium distance
                                                base_speed = 400
                                                damping_factor = 0.4  # Moderate damping
                                            else:  # Close to target
                                                base_speed = 200
                                                damping_factor = 0.5  # Reduced damping for finer control
                                            
                                            # Apply speed based on error with adaptive damping
                                            pan_adjustment = error_x * base_speed
                                            damping = -error_dx * 300  # Reduced from 400 for less aggressive damping
                                            
                                            # Apply proportional damping
                                            pan_adjustment = pan_adjustment * (1 - damping_factor) + damping * damping_factor
                                        else:
                                            pan_adjustment = 0
                                            
                                        # Similar approach for tilt with reduced speeds
                                        if abs(error_y) > tilt_deadzone:
                                            if abs(error_y) > 0.3:  # Far from target
                                                base_speed = 60  # Reduced from 80 for smoother movement
                                                damping_factor = 0.6  # Increased damping
                                            elif abs(error_y) > 0.1:  # Medium distance
                                                base_speed = 30  # Reduced from 40
                                                damping_factor = 0.7  # More damping
                                            else:  # Close to target
                                                base_speed = 15  # Reduced from 20
                                                damping_factor = 0.8  # Much heavier damping for fine adjustments
                                            
                                            tilt_adjustment = error_y * base_speed
                                            damping = -error_dy * 150  # Increased derivative damping
                                            
                                            # Apply proportional damping with stronger effect when close
                                            tilt_adjustment = tilt_adjustment * (1 - damping_factor) + damping * damping_factor
                                            
                                            # Additional velocity limiting when close to target
                                            if abs(error_y) < 0.1:
                                                tilt_adjustment = max(min(tilt_adjustment, 10), -10)  # Strict velocity limit when close
                                        else:
                                            tilt_adjustment = 0
                                        
                                        # Convert adjustments to servo positions with reduced max adjustments
                                        pan_delta = int(pan_adjustment * 11.377)  # 4095/360 ≈ 11.377 steps per degree
                                        max_pan_adjustment = 300  # Reduced from 400
                                        pan_delta = max(min(pan_delta, max_pan_adjustment), -max_pan_adjustment)
                                        
                                        tilt_delta = int(tilt_adjustment * 22.75)  # 4095/180 ≈ 22.75 steps per degree
                                        max_tilt_adjustment = 50  # Reduced from 80
                                        tilt_delta = max(min(tilt_delta, max_tilt_adjustment), -max_tilt_adjustment)
                                        
                                        # Calculate target positions with adaptive smoothing
                                        raw_target_pan_position = dxl_present_position1 + pan_delta
                                        raw_target_tilt_position = dxl_present_position2 + tilt_delta
                                        
                                        # More smoothing when close to target
                                        smooth_factor = 0.3 + (0.4 * (1 - min(1.0, max(abs(error_x), abs(error_y)) / 0.3)))
                                        target_pan_position = int(dxl_present_position1 * smooth_factor + raw_target_pan_position * (1 - smooth_factor))
                                        target_tilt_position = int(dxl_present_position2 * smooth_factor + raw_target_tilt_position * (1 - smooth_factor))
                                        
                                        # Minimal movement threshold
                                        min_movement = 1  # Keep fine control
                                        if abs(target_pan_position - dxl_present_position1) < min_movement:
                                            target_pan_position = dxl_present_position1
                                        if abs(target_tilt_position - dxl_present_position2) < min_movement:
                                            target_tilt_position = dxl_present_position2
                                        
                                        # Store last known good position
                                        control_servos.last_known_pan = target_pan_position
                                        control_servos.last_known_tilt = target_tilt_position
                                        control_servos.last_detection_time = current_time
                                        
                                        logging.info(f"Auto mode - Errors: x={error_x:.3f}, y={error_y:.3f}")
                                        logging.info(f"Error derivatives: dx={error_dx:.3f}, dy={error_dy:.3f}")
                                        logging.info(f"Adjustments - Pan: {pan_adjustment:.2f}°, Tilt: {tilt_adjustment:.2f}°")
                                        logging.info(f"Target positions - Pan: {target_pan_position}, Tilt: {target_tilt_position}")
                                        last_target_time = current_time
                                    else:
                                        current_time = time.time()
                                        if (hasattr(control_servos, 'last_known_pan') and 
                                            hasattr(control_servos, 'last_detection_time') and
                                            current_time - control_servos.last_detection_time < 2.0):
                                            target_pan_position = control_servos.last_known_pan
                                            target_tilt_position = control_servos.last_known_tilt
                                            logging.debug("Target lost - maintaining last known position")
                                        else:
                                            target_pan_position = initial_pan_position
                                            target_tilt_position = initial_tilt_position
                                            time_near_target = 0
                                            logging.debug("Target lost timeout - resetting to home position")
                        else:
                            # Manual control logic remains unchanged
                            pan_angle = latest_command.get('pan_angle', 0)
                            tilt_angle = latest_command.get('tilt_angle', 0)
                            
                            # Convert angles to servo positions with proper scaling
                            pan_steps_per_degree = 11.377  # 4095/360 ≈ 11.377
                            tilt_steps_per_degree = 22.75  # 4095/180 ≈ 22.75
                            
                            target_pan_position = initial_pan_position + int(pan_angle * pan_steps_per_degree)
                            target_tilt_position = initial_tilt_position + int(tilt_angle * tilt_steps_per_degree)
                            
                            logging.info(f"Manual mode - Target angles: Pan={pan_angle:.2f}, Tilt={tilt_angle:.2f}")
                            logging.info(f"Manual mode - Target positions: Pan={target_pan_position}, Tilt={target_tilt_position}")

                        # Apply position limits
                        target_pan_position = max(PAN_MIN_POSITION, min(PAN_MAX_POSITION, target_pan_position))
                        target_tilt_position = max(TILT_MIN_POSITION, min(TILT_MAX_POSITION, target_tilt_position))
                        
                        # Write positions
                        write_servo_position(DXL1_ID, target_pan_position)
                        write_servo_position(DXL2_ID, target_tilt_position)

            time.sleep(0.01)  # Small delay to prevent CPU overload

    except Exception as e:
        logging.error(f"Error in control loop: {e}")
        logging.error(traceback.format_exc())
        time.sleep(1)
        control_servos()  # Restart the control loop if it crashes

def write_servo_position(dxl_id, position, retries=3):
    """Write position to servo with retries"""
    for attempt in range(retries):
        try:
            # Convert position to integer first
            position = int(position)
            original_position = position
            
            if dxl_id == DXL1_ID:  # Pan servo
                # For pan servo, we want to allow full rotation while maintaining direction
                # First, handle the case where position is outside our range
                if position > PAN_MAX_POSITION:
                    # Calculate how many full rotations we're over
                    rotations = (position - PAN_MAX_POSITION) // 4096
                    # Keep the remainder, maintaining direction
                    position = position - (rotations * 4096)
                elif position < PAN_MIN_POSITION:
                    # Calculate how many full rotations we're under
                    rotations = (PAN_MIN_POSITION - position) // 4096
                    # Keep the remainder, maintaining direction
                    position = position + (rotations * 4096)
                
                # Now ensure we're within absolute limits
                position = max(PAN_MIN_POSITION, min(position, PAN_MAX_POSITION))
                logging.debug(f"Pan position before/after mapping: {original_position}/{position}")
            else:  # Tilt servo
                position = max(TILT_MIN_POSITION, min(position, TILT_MAX_POSITION))
                logging.debug(f"Tilt position before/after limits: {original_position}/{position}")
                logging.debug(f"Tilt servo write attempt {attempt + 1}:")
                logging.debug(f"  Initial position: {initial_tilt_position}")
                logging.debug(f"  Current limits: TILT_MIN_POSITION={TILT_MIN_POSITION}, TILT_MAX_POSITION={TILT_MAX_POSITION}")
                logging.debug(f"  Writing position value: {position}")
            
            # Use 4-byte write for both servos
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
