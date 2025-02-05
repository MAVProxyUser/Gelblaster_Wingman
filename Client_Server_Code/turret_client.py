#!/usr/bin/env python3

import os
import sys
import cv2
import numpy as np
import time
import select
import random
import threading
import paho.mqtt.client as mqtt
import json
import pygame
import math
import platform
import traceback
import logging

# Determine the current platform
CURRENT_PLATFORM = platform.system()

# Import evdev only if on Linux
if CURRENT_PLATFORM == 'Linux':
    from evdev import InputDevice, ecodes, list_devices

# Constants
STEAM_DECK_WIDTH = 1280
STEAM_DECK_HEIGHT = 800
CONTROL_CENTER_WIDTH = 200  # Width of the control center
GAME_SCREEN_WIDTH = STEAM_DECK_WIDTH - CONTROL_CENTER_WIDTH  # Width of the game area
SCREEN_WIDTH = STEAM_DECK_WIDTH
SCREEN_HEIGHT = STEAM_DECK_HEIGHT

# MQTT Configuration
MQTT_BROKER = "10.42.0.1"
MQTT_PORT = 1883
MQTT_TOPIC_CONTROL = "dpad/commands"   # Topic to publish commands
MQTT_TOPIC_CAMERA = "camera/feed"      # Topic to subscribe to camera feed
MQTT_TOPIC_SERVO_STATUS = "servo/status"  # Topic to subscribe to servo status
MQTT_TOPIC_BBOX = "camera/bbox"        # Topic to subscribe to bounding box data
MQTT_TOPIC_COMMAND = "dpad/commands"    # Added for the new command
MQTT_TOPIC_RELAY = "relay/command"  # Topic for relay control
RELAY_ACTIVATE = 'off'    # Relay is "normally closed" - sending 'off' activates it
RELAY_DEACTIVATE = 'on'   # Sending 'on' deactivates the relay

# Lock for thread-safe operations
frame_lock = threading.Lock()
latest_frame = None
latest_bbox = None

# Global variables for sounds
whistle_sound = None
laser_sound = None

# MQTT connection status
mqtt_connected = False

# Add these constants after the other constants
KEY_MAP = {
    ord('a'): 'left',
    ord('d'): 'right',
    ord('w'): 'up',
    ord('s'): 'down',
    ord(' '): 'trigger',  # spacebar for trigger
}

# Add at the top with other imports and constants
CROSSHAIR_COLOR = (0, 255, 0)  # Green crosshair

# Add these at the top with other constants
PADDING = 10
BUTTON_HEIGHT = 40
SLIDER_HEIGHT = 50
SPACING = 20

# Add with other constants at the top
SLIDER_WIDTH = 180  # Width of sliders in control panel
SLIDER_HEIGHT = 30  # Height of slider bars
BUTTON_SPACING = 15  # Space between UI elements

class Button:
    def __init__(self, x, y, width, height, text, action=None):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.text = text
        self.action = action
        self.active = False

class Slider:
    def __init__(self, x, y, width, height, min_val, max_val, initial_val, label):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial_val
        self.label = label
        self.dragging = False

    def draw(self, panel):
        # Draw label
        cv2.putText(panel, self.label, (self.x, self.y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # Draw slider background
        cv2.rectangle(panel, (self.x, self.y),
                     (self.x + self.width, self.y + self.height),
                     (50, 50, 50), -1)
        
        # Draw slider position
        pos = int(self.x + (self.value - self.min_val) * self.width / (self.max_val - self.min_val))
        cv2.rectangle(panel, (self.x, self.y),
                     (pos, self.y + self.height),
                     (0, 200, 0), -1)
        
        # Draw value text
        value_text = f"{self.value}"
        text_size = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
        text_x = self.x + self.width + 5
        text_y = self.y + self.height // 2 + text_size[1] // 2
        cv2.putText(panel, value_text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    def handle_mouse(self, x, y, clicked):
        if clicked and self.is_inside(x, y):
            self.dragging = True
        elif not clicked:
            self.dragging = False
            
        if self.dragging:
            rel_x = max(0, min(x - self.x, self.width))
            self.value = int(self.min_val + (rel_x / self.width) * (self.max_val - self.min_val))
            return True
        return False

    def is_inside(self, x, y):
        return (self.x <= x <= self.x + self.width and
                self.y <= y <= self.y + self.height)

class ControllerState:
    def __init__(self):
        # Initialize dot position to center of screen
        self.dot_x = GAME_SCREEN_WIDTH // 2
        self.dot_y = SCREEN_HEIGHT // 2
        self.pan_speed = 200  # Start at speed level 1 (200/200 = 1.0)
        self.reticle_speed = 400
        self.mode = "manual"  # Start in manual mode
        self.relay = False
        self.reverse_pan = False
        self.reverse_tilt = True
        self.pan_aggr = 8.0
        self.tilt_aggr = 4.0
        self.auto_mode = False
        self.pan_angle = None  # Initialize to None, will be set from servo status
        self.tilt_angle = None  # Initialize to None, will be set from servo status
        self.trigger_pressed = False
        self.pan_correction = 0
        self.tilt_correction = 0
        self.accuracy = 100  # Default accuracy
        self.tilt_speed = 200  # Add if needed
        self.left_pressed = False
        self.right_pressed = False
        self.up_pressed = False
        self.down_pressed = False
        self.invert_auto_trigger = False
        self.target_x = GAME_SCREEN_WIDTH // 2  # Initialize to center of screen
        self.target_y = SCREEN_HEIGHT // 2
        self.figure8_enabled = False  # Add this line for the new setting
        self.tof = 0.25  # Add this line for the new setting
        self.sensitivity = 1.0  # Default sensitivity
        self.bbox_drawn = False
        self.current_frame_id = 0
        self.current_bbox = None
        self.servo_status = None
        self.auto_mode_changed = False
        self.manual_control_active = False
        self.speed = 5  # Initialize speed to middle value (5)
        self.pan_speed = 200 * (self.speed / 5.0)  # Scale pan speed based on speed setting
        self.tilt_speed = 200 * (self.speed / 5.0)  # Scale tilt speed based on speed setting
        self.sensitivity_slider = Slider(PADDING, PADDING + BUTTON_HEIGHT + SPACING, 
                                      SLIDER_WIDTH, SLIDER_HEIGHT, 1, 20, 10, "Sensitivity")

# At the top of the file with other globals
latest_frame = None
frame_lock = threading.Lock()
controller_state = None  # Will be initialized in main_loop

def on_connect(client, userdata, flags, reasonCode, properties):
    global mqtt_connected
    if reasonCode == 0:
        mqtt_connected = True
        logging.info("Connected to MQTT broker successfully")
        # Subscribe to topics
        client.subscribe(MQTT_TOPIC_CAMERA, qos=0)
        client.subscribe(MQTT_TOPIC_SERVO_STATUS, qos=0)
        client.subscribe(MQTT_TOPIC_BBOX, qos=0)
        client.subscribe(MQTT_TOPIC_RELAY, qos=0)
        logging.info("Connected and subscribed to topics")
    else:
        mqtt_connected = False
        logging.error(f"Failed to connect to MQTT broker. Reason code: {reasonCode}")

def on_disconnect(client, userdata, rc, properties=None):
    global mqtt_connected
    print(f"Disconnected from MQTT broker with result code: {rc}")
    if properties:
        print(f"Disconnect properties: {properties}")
    mqtt_connected = False

def on_message(client, userdata, msg):
    """MQTT message callback"""
    global latest_frame, controller_state
    
    print(f"Received message on topic: {msg.topic}")  # Add this debug line
    
    if msg.topic == MQTT_TOPIC_RELAY:
        try:
            relay_state = msg.payload.decode()
            if controller_state:
                controller_state.trigger_pressed = (relay_state == 'off')  # 'off' means activate
                logging.info(f"Relay state updated to: {relay_state}")
        except Exception as e:
            logging.error(f"Error processing relay command: {e}")
            traceback.print_exc()
    elif msg.topic == MQTT_TOPIC_CAMERA:
        try:
            # Decode image
            nparr = np.frombuffer(msg.payload, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                print(f"Decoded frame shape: {frame.shape}")  # Add shape info
                with frame_lock:
                    latest_frame = frame.copy()
                    if controller_state:
                        controller_state.current_frame_id += 1
                print(f"Stored frame {controller_state.current_frame_id}")
            else:
                print("Received empty frame")
                
        except Exception as e:
            print(f"Error decoding camera frame: {str(e)}")
            traceback.print_exc()
    
    elif msg.topic == MQTT_TOPIC_BBOX:
        try:
            bbox_data = json.loads(msg.payload)
            if controller_state:  # Check if controller_state exists
                controller_state.current_bbox = bbox_data
        except Exception as e:
            print(f"Error processing bbox data: {str(e)}")
            traceback.print_exc()
    elif msg.topic == MQTT_TOPIC_SERVO_STATUS:
        try:
            servo_status = json.loads(msg.payload.decode())
            if controller_state:
                # Update controller state with current servo positions
                controller_state.servo_status = servo_status
                # Initialize angles from first servo status
                if controller_state.pan_angle is None:
                    controller_state.pan_angle = servo_status.get('pan_angle')
                if controller_state.tilt_angle is None:
                    controller_state.tilt_angle = servo_status.get('tilt_angle')
                logging.info(f"Updated servo status: {servo_status}")
            else:
                logging.warning("Received servo status but controller_state is None")
        except Exception as e:
            logging.error(f"Error processing servo status: {e}")
            traceback.print_exc()
    elif msg.topic == "server/tof":
        try:
            data = json.loads(msg.payload.decode())
            tof = data.get('tof', 0.25)
            print(f"Updated TOF to: {tof}", flush=True)
        except json.JSONDecodeError:
            print("Error decoding TOF JSON", flush=True)
    else:
        print(f"Received message on unexpected topic: {msg.topic}")

def send_mqtt_command(client, controller_state):
    """Send the current state as an MQTT command."""
    command = {
        'pan_angle': controller_state.pan_angle,
        'tilt_angle': controller_state.tilt_angle,
        'auto_mode': controller_state.auto_mode,
        'speed': controller_state.pan_speed / 200.0  # Convert speed to 1-10 range
    }
    print(f"Sending command: {command}")
    client.publish(MQTT_TOPIC_CONTROL, json.dumps(command))

def draw_control_panel(panel, state):
    """Draw the control panel with buttons and sliders."""
    # Draw panel background
    cv2.rectangle(panel, (0, 0), (CONTROL_CENTER_WIDTH, SCREEN_HEIGHT), (30, 30, 30), -1)
    
    # Draw Auto/Manual Toggle Button
    button_color = (0, 200, 0) if state.auto_mode else (100, 100, 100)
    cv2.rectangle(panel, 
                 (PADDING, PADDING), 
                 (CONTROL_CENTER_WIDTH - PADDING, PADDING + BUTTON_HEIGHT),
                 button_color, -1)
    cv2.putText(panel, 
                f"{'AUTO' if state.auto_mode else 'MANUAL'}", 
                (PADDING + 10, PADDING + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Draw sensitivity slider below the auto/manual button
    state.sensitivity_slider.y = PADDING + BUTTON_HEIGHT + SPACING  # Update slider position
    state.sensitivity_slider.draw(panel)

def handle_speed_click(x, y, controller_state):
    """Handle clicks on the speed slider with improved touch handling"""
    PADDING = 10
    BUTTON_HEIGHT = 40
    SLIDER_HEIGHT = 50
    SPACING = 20
    
    # Calculate slider y position (same as in draw_control_panel)
    slider_y = PADDING + BUTTON_HEIGHT + SPACING + 25
    
    # Larger touch area for the slider
    touch_area_top = slider_y - 10
    touch_area_bottom = slider_y + SLIDER_HEIGHT + 10
    
    if touch_area_top <= y <= touch_area_bottom:
        # Calculate new speed based on x position
        slider_width = CONTROL_CENTER_WIDTH - (2 * PADDING)
        x_pos = max(PADDING, min(x, CONTROL_CENTER_WIDTH - PADDING))
        new_speed = 1 + int((x_pos - PADDING) * 9 / slider_width)
        new_speed = max(1, min(10, new_speed))
        
        # Only update if speed changed
        if new_speed != controller_state.speed:
            controller_state.speed = new_speed
            # Update movement speeds based on new speed setting
            controller_state.pan_speed = 200 * (new_speed / 5.0)
            controller_state.tilt_speed = 200 * (new_speed / 5.0)
            if mqtt_connected:
                client.publish(MQTT_TOPIC_CONTROL, json.dumps({
                    'speed': new_speed,
                    'pan_speed': controller_state.pan_speed,
                    'tilt_speed': controller_state.tilt_speed
                }))
            return True  # Indicate the speed was changed
    return False  # Indicate no change

# Define buttons before main_loop()
buttons = [
    Button(10, 150, 180, 30, "Manual Mode"),
    Button(10, 190, 180, 30, "Auto Mode"),
    Button(10, 230, 180, 30, "Reset Position")
]

def find_input_devices():
    """Find input devices, using evdev on Linux and keyboard input on macOS."""
    if CURRENT_PLATFORM == 'Darwin':  # macOS
        return []  # Use keyboard input instead
    else:
        # Original Linux implementation
        try:
            devices = [InputDevice(path) for path in list_devices()]
            input_devices = []
            for device in devices:
                capabilities = device.capabilities(verbose=True)
                is_input_device = False
                for event_type, event_codes in capabilities.items():
                    if event_type == 'EV_KEY':
                        for code, _ in event_codes:
                            if code in ('BTN_GAMEPAD', 'BTN_SOUTH', 'BTN_A', 'BTN_START', 'BTN_EAST', 'BTN_B',
                                        'BTN_LEFT', 'BTN_RIGHT', 'BTN_DPAD_LEFT', 'BTN_DPAD_RIGHT', 'BTN_DPAD_UP', 'BTN_DPAD_DOWN'):
                                is_input_device = True
                                break
                    if is_input_device:
                        break
                if is_input_device or 'gamepad' in device.name.lower() or 'controller' in device.name.lower() \
                        or 'touchscreen' in device.name.lower():
                    print(f"Found input device: {device.path}, name: {device.name}")
                    input_devices.append(device)
                else:
                    print(f"Skipping device: {device.path}, name: {device.name}")
            return input_devices
        except Exception as e:
            print(f"Error finding input devices: {e}")
            return []

def load_sounds():
    global whistle_sound, laser_sound
    whistle_sound = None
    laser_sound = None
    try:
        whistle_sound = pygame.mixer.Sound('whistle.wav')
    except (pygame.error, FileNotFoundError):
        print("Warning: 'whistle.wav' not found. Whistle sound will be disabled.")
    
    try:
        laser_sound = pygame.mixer.Sound('laser.wav')
    except (pygame.error, FileNotFoundError):
        print("Warning: 'laser.wav' not found. Laser sound will be disabled.")

def mouse_callback(event, x, y, flags, param):
    """Handle mouse events for the control panel."""
    global controller_state, client
    
    if x < CONTROL_CENTER_WIDTH:  # Click is in control panel
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check Auto/Manual button
            if (PADDING <= x <= CONTROL_CENTER_WIDTH - PADDING and 
                PADDING <= y <= PADDING + BUTTON_HEIGHT):
                controller_state.auto_mode = not controller_state.auto_mode
                command = {'auto_mode': controller_state.auto_mode}
                client.publish(MQTT_TOPIC_COMMAND, json.dumps(command))
            
            # Check slider
            controller_state.sensitivity_slider.handle_mouse(x, y, True)
            
        elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):
            if controller_state.sensitivity_slider.handle_mouse(x, y, True):
                sensitivity = controller_state.sensitivity_slider.value / 10.0  # Convert to 0.1-2.0 range
                command = {'sensitivity': sensitivity}
                client.publish("server/sensitivity", json.dumps(command))
                
        elif event == cv2.EVENT_LBUTTONUP:
            controller_state.sensitivity_slider.handle_mouse(x, y, False)

def draw_crosshair(frame, x, y, color):
    """Draw a crosshair at the specified position with the given color."""
    cv2.line(frame, (x - 10, y), (x + 10, y), color, 2)
    cv2.line(frame, (x, y - 10), (x, y + 10), color, 2)

def send_tof_update(client, tof_value):
    """Send TOF update to the server."""
    try:
        payload = json.dumps({'tof': tof_value})
        result = client.publish("server/tof", payload)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Published TOF update: {payload} to topic 'server/tof'", flush=True)
        else:
            print(f"Failed to publish TOF update: {mqtt.error_string(result.rc)}", flush=True)
    except Exception as e:
        print(f"Error publishing TOF update: {e}", flush=True)

def send_sensitivity_update(client, sensitivity_value):
    """Send sensitivity update to the server."""
    try:
        payload = json.dumps({'sensitivity': sensitivity_value})
        result = client.publish("server/sensitivity", payload)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Published sensitivity update: {payload} to topic 'server/sensitivity'", flush=True)
        else:
            print(f"Failed to publish sensitivity update: {mqtt.error_string(result.rc)}", flush=True)
    except Exception as e:
        print(f"Error publishing sensitivity update: {e}", flush=True)

def enable_auto_mode():
    command = {
        'auto_mode': True
    }
    client.publish(MQTT_TOPIC_COMMAND, json.dumps(command))

def main_loop():
    global controller_state, mqtt_connected, client
    
    # Initialize MQTT client
    client = mqtt.Client(protocol=mqtt.MQTTv5)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except Exception as e:
        logging.error(f"Failed to connect to MQTT broker: {e}")
        mqtt_connected = False
    
    # Initialize controller state
    controller_state = ControllerState()
    
    # Initialize display window
    cv2.namedWindow('Turret Client', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Turret Client', mouse_callback)
    
    try:
        while True:
            # Create the display
            current_frame = None
            with frame_lock:
                if latest_frame is not None:
                    current_frame = latest_frame.copy()

            if current_frame is not None:
                # Resize the frame
                current_frame = cv2.resize(current_frame, (GAME_SCREEN_WIDTH, SCREEN_HEIGHT))
                
                # Draw bbox if available
                if controller_state.current_bbox:
                    scale_x = GAME_SCREEN_WIDTH / 640
                    scale_y = SCREEN_HEIGHT / 480

                    x = int(controller_state.current_bbox['x'] * scale_x)
                    y = int(controller_state.current_bbox['y'] * scale_y)
                    w = int(controller_state.current_bbox['width'] * scale_x)
                    h = int(controller_state.current_bbox['height'] * scale_y)

                    cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    if controller_state.auto_mode:
                        controller_state.target_x = x + w // 2
                        controller_state.target_y = y + h // 2

                # Draw crosshair
                draw_crosshair(current_frame, int(controller_state.dot_x), int(controller_state.dot_y), CROSSHAIR_COLOR)

                # Draw control panel
                control_center = np.zeros((SCREEN_HEIGHT, CONTROL_CENTER_WIDTH, 3), dtype=np.uint8)
                draw_control_panel(control_center, controller_state)

                # Combine displays
                combined_display = np.hstack((control_center, current_frame))

                # Show the frame
                cv2.imshow('Turret Client', combined_display)
            else:
                # Show blank screen with message if no frame
                blank_display = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH, 3), dtype=np.uint8)
                cv2.putText(blank_display, "Waiting for camera feed...", 
                          (SCREEN_WIDTH//4, SCREEN_HEIGHT//2),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow('Turret Client', blank_display)

            # Handle keyboard input for manual mode
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # If a key was pressed
                print(f"Debug - Key code pressed: {key}, Hex: {hex(key)}")  # Enhanced debug print
                if key == ord(' '):  # Spacebar
                    print("Spacebar pressed - firing")  # Debug print
                    client.publish(MQTT_TOPIC_RELAY, "off")  # Send raw string instead of JSON
                    time.sleep(0.1)  # Brief delay
                    client.publish(MQTT_TOPIC_RELAY, "on")   # Send raw string instead of JSON
                elif key == 27:  # ESC key
                    break
                elif key in [ord('a'), ord('A'), 2, 81]:  # Left arrow or 'a'
                    print("Left pressed")  # Debug print
                    command = {'pan_delta': -5}
                    client.publish(MQTT_TOPIC_COMMAND, json.dumps(command))
                elif key in [ord('d'), ord('D'), 3, 83]:  # Right arrow or 'd'
                    print("Right pressed")  # Debug print
                    command = {'pan_delta': 5}
                    client.publish(MQTT_TOPIC_COMMAND, json.dumps(command))
                elif key in [ord('w'), ord('W'), 0, 82]:  # Up arrow or 'w'
                    print("Up pressed")  # Debug print
                    command = {'tilt_delta': -5}
                    client.publish(MQTT_TOPIC_COMMAND, json.dumps(command))
                elif key in [ord('s'), ord('S'), 1, 84]:  # Down arrow or 's'
                    print("Down pressed")  # Debug print
                    command = {'tilt_delta': 5}
                    client.publish(MQTT_TOPIC_COMMAND, json.dumps(command))

            # Process mouse events
            cv2.setMouseCallback('Turret Client', mouse_callback)

            # Only send command when auto mode changes or manual control is used
            if controller_state.auto_mode_changed or controller_state.manual_control_active:
                command = {
                    'auto_mode': controller_state.auto_mode,
                    'pan_angle': controller_state.servo_status.get('pan_angle', 0.0) if controller_state.servo_status else 0.0,
                    'tilt_angle': controller_state.servo_status.get('tilt_angle', -180.0) if controller_state.servo_status else -180.0
                }
                client.publish(MQTT_TOPIC_CONTROL, json.dumps(command))
                logging.info(f"Published command: {command}")
                controller_state.auto_mode_changed = False
                controller_state.manual_control_active = False
            
            time.sleep(0.05)  # 20Hz update rate

    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
        client.loop_stop()
        client.disconnect()

if __name__ == '__main__':
    main_loop()