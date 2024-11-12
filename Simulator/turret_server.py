#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import paho.mqtt.client as mqtt
import json
import platform
import socket  # Added for IP address detection
import netifaces

# Detect the platform
current_platform = platform.system()
print(f"Running on {current_platform}", flush=True)

# Common imports
import cv2  # OpenCV is used on both platforms

if current_platform == 'Linux':
    # Import Linux-specific modules (Jetson)
    import Jetson.GPIO as GPIO
    import depthai as dai
    import numpy as np
    from dynamixel_sdk import *
    import serial.tools.list_ports  # For automatic port detection

elif current_platform == 'Darwin':
    # macOS-specific imports (no additional imports needed)
    pass

else:
    print(f"Unsupported platform: {current_platform}", flush=True)
    sys.exit(1)

# Function to get the local IP address of the machine
def get_local_ip():
    """Retrieve the local IP address of the machine."""
    try:
        # Get the default gateway interface
        default_gateway = netifaces.gateways()['default'][netifaces.AF_INET][1]
        
        # Get the IP address for this interface
        interface_addresses = netifaces.ifaddresses(default_gateway)
        local_ip = interface_addresses[netifaces.AF_INET][0]['addr']
        
        print(f"Detected local IP address: {local_ip}", flush=True)
        return local_ip
    except Exception as e:
        print(f"Unable to detect local IP address: {e}", flush=True)
        # Default to localhost if IP detection fails
        return '127.0.0.1'

# MQTT Broker Configuration
MQTT_BROKER = get_local_ip()  # Use the detected local IP address
MQTT_PORT = 1883
MQTT_TOPIC_CONTROL = "dpad/commands"       # Topic for receiving commands
MQTT_TOPIC_CAMERA = "camera/feed"          # Topic for publishing camera feed
MQTT_TOPIC_SERVO_STATUS = "servo/status"   # Topic for publishing servo status
MQTT_TOPIC_DEPTH = "camera/depth"          # Topic for publishing depth feed (only on Jetson)
MQTT_TOPIC_PLATFORM = "server/platform"
MQTT_TOPIC_PLATFORM_REQUEST = "server/platform_request"

# Variables to store the latest command
latest_command = {'pan_angle': 0.0, 'tilt_angle': -90.0}  # Initial angles
command_lock = threading.Lock()  # To synchronize access to latest_command

# Setup for GPIO and servos only if on Linux (Jetson)
if current_platform == 'Linux':
    # Initialize the GPIO pin for relay
    RELAY_PIN = 7
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    # Definitions from your previous code and the provided code
    # Dynamixel servo configurations
    ADDR_PRESENT_POSITION = 132
    LEN_PRESENT_POSITION = 4
    MY_DXL = 'MX_SERIES'
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    LEN_GOAL_POSITION = 4
    BAUDRATE = 1000000
    PROTOCOL_VERSION = 2.0
    DXL1_ID = 1  # Pan servo
    DXL2_ID = 2  # Tilt servo
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    DXL_MOVING_STATUS_THRESHOLD = 40
    ADDR_GOAL_TORQUE = 102
    LEN_GOAL_TORQUE = 2
    DXL_GOAL_TORQUE = 500  # Be careful with this value

    # Use the automatic detection function to find Dynamixel port
    def find_dynamixel_port():
        """Automatically detect the Dynamixel servo controller port."""
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            try:
                port_handler = PortHandler(port.device)
                if port_handler.openPort():
                    if port_handler.setBaudRate(BAUDRATE):
                        packet_handler = PacketHandler(PROTOCOL_VERSION)
                        dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, DXL1_ID)
                        if dxl_comm_result == COMM_SUCCESS:
                            print(f"Found Dynamixel servo at port {port.device}", flush=True)
                            return port_handler, packet_handler
                    port_handler.closePort()
            except Exception as e:
                print(f"Error checking port {port.device}: {e}", flush=True)
                continue
        print("Failed to find a valid Dynamixel port!", flush=True)
        sys.exit()

    # Initialize portHandler and packetHandler
    portHandler, packetHandler = find_dynamixel_port()

    # Enable Torque for both servos
    for dxl_id in [DXL1_ID, DXL2_ID]:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"Failed to enable torque on Dynamixel#{dxl_id}: "
                  f"{packetHandler.getTxRxResult(dxl_comm_result)}, "
                  f"Error: {packetHandler.getRxPacketError(dxl_error)}", flush=True)
            sys.exit()
        else:
            print(f"Torque enabled on Dynamixel#{dxl_id}", flush=True)

    # Dynamixel Torque setup
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, DXL2_ID, ADDR_GOAL_TORQUE, DXL_GOAL_TORQUE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result), flush=True)
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error), flush=True)
    else:
        print(f"Goal torque set to {DXL_GOAL_TORQUE} on Dynamixel#{DXL2_ID}", flush=True)

    groupSyncWrite = GroupSyncWrite(
        portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Function to convert angle to servo position (only on Linux)
def angle_to_servo_position(dxl_id, angle_in_degrees):
    if current_platform == 'Linux':
        # Define constants
        degrees_per_unit = 360.0 / 4096  # Degrees per unit
        center_position = 2048  # Center position for 0 degrees

        # Map angle_in_degrees to position
        dxl_goal_position = center_position + int(angle_in_degrees / degrees_per_unit)

        # Clamp the position within valid range
        min_position = 0
        max_position = 4095
        dxl_goal_position = max(min_position, min(dxl_goal_position, max_position))
        return dxl_goal_position
    else:
        # Placeholder for other platforms
        return 0

# Servo update thread function
def servo_update_thread():
    if current_platform == 'Linux':
        # Move to initial position immediately
        initial_tilt_position = angle_to_servo_position(DXL2_ID, -90.0)
        param_initial_tilt = [
            DXL_LOBYTE(DXL_LOWORD(initial_tilt_position)),
            DXL_HIBYTE(DXL_LOWORD(initial_tilt_position)),
            DXL_LOBYTE(DXL_HIWORD(initial_tilt_position)),
            DXL_HIBYTE(DXL_HIWORD(initial_tilt_position))
        ]
        groupSyncWrite.addParam(DXL2_ID, param_initial_tilt)
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Initial tilt position set failed: {packetHandler.getTxRxResult(dxl_comm_result)}", flush=True)
        else:
            print("Initial tilt position set to -90 degrees", flush=True)
        groupSyncWrite.clearParam()

        while True:
            try:
                # Get the latest command
                with command_lock:
                    command = latest_command.copy()

                pan_angle = command.get('pan_angle', 0.0)
                tilt_angle = command.get('tilt_angle', 0.0)

                # Debug messages for angles
                print(f"Servo Update Thread - Pan Angle: {pan_angle}, Tilt Angle: {tilt_angle}", flush=True)

                dxl1_goal_position = angle_to_servo_position(DXL1_ID, pan_angle)
                dxl2_goal_position = angle_to_servo_position(DXL2_ID, tilt_angle)

                # Debug messages for goal positions
                print(f"Calculated Goal Positions - DXL1_ID: {dxl1_goal_position}, DXL2_ID: {dxl2_goal_position}", flush=True)

                # Create goal position byte arrays
                param_goal_position_dxl1 = [
                    DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)),
                    DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)),
                    DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)),
                    DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))
                ]

                param_goal_position_dxl2 = [
                    DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)),
                    DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
                    DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)),
                    DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))
                ]

                # Add parameters to groupSyncWrite
                if not groupSyncWrite.addParam(DXL1_ID, param_goal_position_dxl1):
                    print(f"[ID:{DXL1_ID}] groupSyncWrite addparam failed", flush=True)
                if not groupSyncWrite.addParam(DXL2_ID, param_goal_position_dxl2):
                    print(f"[ID:{DXL2_ID}] groupSyncWrite addparam failed", flush=True)

                # Syncwrite goal position
                dxl_comm_result = groupSyncWrite.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"SyncWrite failed: {packetHandler.getTxRxResult(dxl_comm_result)}", flush=True)
                else:
                    print("SyncWrite successful", flush=True)

                # Clear Syncwrite parameter storage
                groupSyncWrite.clearParam()

                # Sleep for a short duration
                time.sleep(0.05)  # Adjust the sleep time as needed

            except Exception as e:
                print(f"Error in servo update thread: {e}", flush=True)
                # Optionally, attempt to reconnect or handle the exception
                break
    else:
        # On other platforms, we might simulate servo updates or skip this thread
        while True:
            # Simulate some processing
            time.sleep(1)

# Callback function for MQTT connection
def on_connect(client, userdata, flags, reasonCode, properties):
    if reasonCode == 0:
        print("Connected to MQTT broker successfully.", flush=True)
        # Subscribe to necessary topics
        client.subscribe(MQTT_TOPIC_CONTROL)
        print(f"Subscribed to topic {MQTT_TOPIC_CONTROL}", flush=True)
        client.subscribe(MQTT_TOPIC_PLATFORM_REQUEST)
        print(f"Subscribed to platform request topic", flush=True)
    else:
        print(f"Failed to connect to MQTT broker. Reason code: {reasonCode}", flush=True)

# Callback function for MQTT message handling
def on_message(client, userdata, msg):
    global latest_command
    try:
        command = json.loads(msg.payload.decode("utf-8"))
        print(f"Received MQTT command: {command}", flush=True)

        # Control the relay if included in the command (only on Jetson)
        if current_platform == 'Linux':
            if 'relay' in command:
                if command['relay'] == 'on':
                    GPIO.output(RELAY_PIN, GPIO.HIGH)
                    print("Relay turned ON", flush=True)
                elif command['relay'] == 'off':
                    GPIO.output(RELAY_PIN, GPIO.LOW)
                    print("Relay turned OFF", flush=True)

        # Update the latest command
        with command_lock:
            latest_command['pan_angle'] = command.get('pan_angle', 0.0)
            latest_command['tilt_angle'] = command.get('tilt_angle', 0.0)

        # Handle platform request
        if msg.topic == MQTT_TOPIC_PLATFORM_REQUEST:
            response = {
                'platform': current_platform
            }
            client.publish(MQTT_TOPIC_PLATFORM, json.dumps(response))

    except Exception as e:
        print(f"Error processing MQTT message: {e}", flush=True)

# MQTT client setup with MQTTv5 protocol
client = mqtt.Client(protocol=mqtt.MQTTv5)

# Set up the callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
try:
    print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}", flush=True)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
except Exception as e:
    print(f"Connection failed: {e}", flush=True)
    print(f"Error type: {type(e)}", flush=True)
    print(f"Error args: {e.args}", flush=True)
    sys.exit(1)

# Start the MQTT client loop in a separate thread
client.loop_start()

# Start the servo update thread
servo_thread = threading.Thread(target=servo_update_thread)
servo_thread.daemon = True
servo_thread.start()

# Function to capture and publish camera frames
def publish_camera_feed():
    print("Starting camera feed publishing...", flush=True)
    if current_platform == 'Linux':
        # Existing code for depthai camera on Jetson
        pipeline = dai.Pipeline()

        # RGB camera setup
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setPreviewSize(640, 480)  # Reduced size
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Depth camera setup
        mono_left = pipeline.createMonoCamera()
        mono_right = pipeline.createMonoCamera()
        stereo = pipeline.createStereoDepth()

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.createXLinkOut()
        xout_depth = pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        try:
            with dai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

                while True:
                    in_rgb = q_rgb.get()
                    in_depth = q_depth.get()

                    frame_rgb = in_rgb.getCvFrame()
                    frame_depth = in_depth.getFrame()

                    # Rotate RGB frame 180 degrees
                    frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)

                    # Encode and publish RGB frame
                    _, buffer_rgb = cv2.imencode('.jpg', frame_rgb, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    client.publish(MQTT_TOPIC_CAMERA, buffer_rgb.tobytes(), retain=False)

                    # Normalize and encode depth frame
                    frame_depth = (frame_depth * 255 / np.max(frame_depth)).astype(np.uint8)
                    _, buffer_depth = cv2.imencode('.jpg', frame_depth, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    client.publish(MQTT_TOPIC_DEPTH, buffer_depth.tobytes(), retain=False)

                    print("Published RGB and depth frames to MQTT", flush=True)

                    time.sleep(0.1)  # Adjust as needed for desired frame rate

        except Exception as e:
            print(f"Error in camera feed: {e}", flush=True)

    elif current_platform == 'Darwin':
        # Code to capture camera feed from built-in webcam on macOS
        try:
            print("Attempting to open webcam...", flush=True)
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Cannot access webcam on macOS", flush=True)
                return

            print("Webcam opened successfully. Starting frame capture loop.", flush=True)
            frame_count = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame from webcam", flush=True)
                    break

                # Encode and publish frame
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                publish_result = client.publish(MQTT_TOPIC_CAMERA, buffer.tobytes(), retain=False)
                
                if publish_result.is_published():
                    print(f"Published webcam frame {frame_count} to MQTT", flush=True)
                else:
                    print(f"Failed to publish frame {frame_count}. Error: {publish_result.rc}", flush=True)

                frame_count += 1
                time.sleep(0.1)  # Adjust as needed for desired frame rate

        except Exception as e:
            print(f"Error in camera feed: {e}", flush=True)
        finally:
            if 'cap' in locals() and cap.isOpened():
                cap.release()
                print("Webcam released.", flush=True)
    else:
        print(f"No camera support for platform: {current_platform}", flush=True)

# Start the camera feed publishing in a separate thread
camera_thread = threading.Thread(target=publish_camera_feed)
camera_thread.daemon = True  # Ensure thread exits when main program does
camera_thread.start()

try:
    print("Main loop started. Camera feed should be publishing.", flush=True)
    loop_count = 0
    while True:
        # Publish servo status periodically
        servo_status = {
            'pan_angle': latest_command['pan_angle'],
            'tilt_angle': latest_command['tilt_angle']
        }
        status_result = client.publish(MQTT_TOPIC_SERVO_STATUS, json.dumps(servo_status))
        if status_result.is_published():
            print(f"Published servo status: {servo_status}", flush=True)
        else:
            print(f"Failed to publish servo status. Error: {status_result.rc}", flush=True)
        
        print(f"Main loop iteration {loop_count}", flush=True)
        loop_count += 1
        time.sleep(1)  # Publish status every second

except KeyboardInterrupt:
    print("Shutting down", flush=True)

finally:
    # Cleanup on exit
    client.loop_stop()
    client.disconnect()
    if current_platform == 'Linux':
        GPIO.cleanup()
        portHandler.closePort()
