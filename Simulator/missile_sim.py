#!/usr/bin/env python3

import os
import sys
import cv2
import numpy as np
import time
import select
import random
import threading
from evdev import InputDevice, ecodes, list_devices
from math import hypot, sqrt, atan2
import paho.mqtt.client as mqtt
import json
import pygame
import math

# Constants
STEAM_DECK_WIDTH = 1280
STEAM_DECK_HEIGHT = 800
CONTROL_CENTER_WIDTH = 200  # Width of the control center
GAME_SCREEN_WIDTH = STEAM_DECK_WIDTH - CONTROL_CENTER_WIDTH  # Width of the game area
SCREEN_WIDTH = STEAM_DECK_WIDTH
SCREEN_HEIGHT = STEAM_DECK_HEIGHT
BUILDING_COUNT = 5

# Physical parameters
PAN_ANGLE_MIN = -80   # Changed from -60 to -40 (shifted right by 20 degrees)
PAN_ANGLE_MAX = 320    # Changed from 60 to 80 (shifted right by 20 degrees)
PAN_ANGLE_RANGE = PAN_ANGLE_MAX - PAN_ANGLE_MIN  # Still 120 degrees total

TILT_ANGLE_MAX = -160   # Keep current tilt settings
TILT_ANGLE_MIN = -340
TILT_ANGLE_RANGE = TILT_ANGLE_MAX - TILT_ANGLE_MIN

# MQTT Configuration
MQTT_BROKER = "10.42.0.1"
MQTT_PORT = 1883
MQTT_TOPIC_CONTROL = "dpad/commands"   # Topic to publish commands
MQTT_TOPIC_CAMERA = "camera/feed"      # Topic to subscribe to camera feed
MQTT_TOPIC_SERVO_STATUS = "servo/status"  # Topic to subscribe to servo status

# Lock for thread-safe operations
frame_lock = threading.Lock()
latest_frame = None

# Global variables for sounds
whistle_sound = None
laser_sound = None

# MQTT connection status
mqtt_connected = False

# At the top of the file with other constants
RELAY_ACTIVATE = 'off'    # Relay is "normally closed" - sending 'off' activates it
RELAY_DEACTIVATE = 'on'   # Sending 'on' deactivates the normally closed relay

class GameState:
    def __init__(self):
        # Initialize buildings
        self.buildings = [{'x': x, 'intact': True} for x in np.linspace(50, GAME_SCREEN_WIDTH - 50, BUILDING_COUNT)]
        # Initialize missiles
        self.incoming_missiles = []
        self.player_missiles = []
        # Explosion effects
        self.explosions = []
        # Game over flag
        self.game_over = False
        # Difficulty level
        self.difficulty = 1  # Default difficulty level
        self.handicap = 0  # Default handicap (no reduction in shots)
        # Wave count
        self.wave_number = 1  # Start from wave 1
        self.display_mode = "Game"  # Track display mode
        self.current_target = None  # Track current target missile
        self.shot_time = None      # Track when we fired at current target
        self.expected_intercept_time = None  # Track when we expect the intercept

class ControllerState:
    def __init__(self):
        # Initialize dot position to center of screen
        self.dot_x = GAME_SCREEN_WIDTH // 2
        self.dot_y = SCREEN_HEIGHT // 2
        self.pan_speed = 200
        self.reticle_speed = 400
        self.mode = "manual"  # Start in manual mode
        self.relay = False
        self.reverse_pan = False
        self.reverse_tilt = True
        self.pan_aggr = 8.0
        self.tilt_aggr = 4.0
        self.auto_mode = False
        # Add these if they're used elsewhere
        self.pan_angle = 0
        self.tilt_angle = 0
        self.trigger_pressed = False
        self.pan_correction = 0
        self.tilt_correction = 0
        self.accuracy = 100  # Default accuracy
        self.tilt_speed = 200  # Add if needed

        # Add directional button states
        self.left_pressed = False
        self.right_pressed = False
        self.up_pressed = False
        self.down_pressed = False

        self.invert_auto_trigger = False

        # Add target coordinates initialization
        self.target_x = GAME_SCREEN_WIDTH // 2  # Initialize to center of screen
        self.target_y = SCREEN_HEIGHT // 2

        self.figure8_enabled = False  # Add this line for the new setting

def on_connect(client, userdata, flags, rc, properties=None):
    global mqtt_connected
    if rc == 0:
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(MQTT_TOPIC_CAMERA)
        client.subscribe(MQTT_TOPIC_SERVO_STATUS)
        print(f"Subscribed to topics: {MQTT_TOPIC_CAMERA}, {MQTT_TOPIC_SERVO_STATUS}")
        mqtt_connected = True
    else:
        print(f"Failed to connect to MQTT broker, return code {rc}")
        mqtt_connected = False
    if properties:
        print(f"Connection properties: {properties}")

def on_disconnect(client, userdata, rc, properties=None):
    global mqtt_connected
    print(f"Disconnected from MQTT broker with result code: {rc}")
    if properties:
        print(f"Disconnect properties: {properties}")
    mqtt_connected = False

def on_message(client, userdata, msg):
    global latest_frame

    print(f"Received message on topic: {msg.topic}")

    if msg.topic == MQTT_TOPIC_CAMERA:
        try:
            # Decode the JPEG image
            nparr = np.frombuffer(msg.payload, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                with frame_lock:
                    latest_frame = frame
                print(f"Processed frame: shape={frame.shape}, dtype={frame.dtype}")
            else:
                print("Received empty frame from MQTT")
        except Exception as e:
            print(f"Error decoding camera frame: {e}")
    elif msg.topic == MQTT_TOPIC_SERVO_STATUS:
        try:
            servo_status = json.loads(msg.payload.decode())
            print(f"Received servo status: {servo_status}")
        except json.JSONDecodeError:
            print("Error decoding servo status JSON")
    else:
        print(f"Received message on unexpected topic: {msg.topic}")

def send_mqtt_command(client, controller_state):
    """Send pan/tilt angles and trigger state to the MQTT server."""
    if mqtt_connected:
        # Prepare the data to send
        data = {
            'pan_angle': controller_state.pan_angle,
            'tilt_angle': controller_state.tilt_angle,
            'relay': 'on' if controller_state.trigger_pressed else 'off'
        }

        # Send the data as a message to the MQTT topic
        result = client.publish(MQTT_TOPIC_CONTROL, json.dumps(data))
        # Add debug statements
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Sent MQTT command: {data}")
        else:
            print(f"Failed to publish MQTT command: {mqtt.error_string(result.rc)}")

def find_input_devices():
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

def spawn_incoming_missiles(game_state):
    """Spawn incoming missiles based on the current difficulty."""
    intact_buildings = [b for b in game_state.buildings if b['intact']]
    if not intact_buildings:
        return  # No intact buildings to target

    # Clear any existing missiles
    game_state.incoming_missiles = []
    
    # Number of missiles is equal to the difficulty level
    num_missiles = game_state.difficulty

    for _ in range(num_missiles):
        start_x = random.randint(0, GAME_SCREEN_WIDTH)
        target_building = random.choice(intact_buildings)  # This is safe now since we checked if intact_buildings is empty
        speed = random.uniform(50, 100 + game_state.difficulty * 10)
        missile = {
            'start_x': start_x,
            'start_y': 0,
            'end_x': target_building['x'],
            'end_y': SCREEN_HEIGHT - 40,
            'current_x': start_x,
            'current_y': 0,
            'speed': speed
        }
        game_state.incoming_missiles.append(missile)

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

def update_servo_position(target_x, target_y, current_pan, current_tilt, controller_state, mode="game"):
    """
    Update servo position based on target coordinates and current angles
    Returns new pan and tilt angles
    """
    # Screen center coordinates
    center_x = GAME_SCREEN_WIDTH // 2
    center_y = SCREEN_HEIGHT // 2

    if mode == "live":
        # Calculate offset from center as a percentage of screen width/height
        x_offset_pct = (target_x - center_x) / center_x
        y_offset_pct = (target_y - center_y) / center_y

        # Scale adjustments by aggressiveness (now using 1-10 scale)
        pan_adjustment = x_offset_pct * controller_state.pan_aggr * 2.0  # Doubled the pan multiplier
        tilt_adjustment = -y_offset_pct * controller_state.tilt_aggr

        # Apply reverse settings
        if controller_state.reverse_pan:
            pan_adjustment = -pan_adjustment
        if controller_state.reverse_tilt:
            tilt_adjustment = -tilt_adjustment

        # Add smoothing to reduce overshooting (reduced smoothing for faster response)
        smoothing_factor = 0.7  # Increased from 0.5 for faster response
        new_pan = current_pan + (pan_adjustment * smoothing_factor)
        new_tilt = current_tilt + (tilt_adjustment * smoothing_factor)
    else:
        # For game mode, map screen coordinates directly to full pan range
        x_ratio = (target_x / GAME_SCREEN_WIDTH)  # Will be 0 to 1
        y_ratio = (target_y / SCREEN_HEIGHT)
        
        # Map x_ratio from 0-1 to full pan range
        new_pan = PAN_ANGLE_MIN + (x_ratio * (PAN_ANGLE_MAX - PAN_ANGLE_MIN))
        new_tilt = TILT_ANGLE_MIN + (y_ratio * (TILT_ANGLE_MAX - TILT_ANGLE_MIN))

        # Apply reverse settings
        if controller_state.reverse_pan:
            new_pan = -new_pan
        if controller_state.reverse_tilt:
            new_tilt = -new_tilt

    # Clamp angles to valid ranges
    new_pan = max(PAN_ANGLE_MIN, min(PAN_ANGLE_MAX, new_pan))
    new_tilt = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, new_tilt))

    return new_pan, new_tilt

def main_loop():
    global controller_state, game_state, client
    global latest_frame, mqtt_connected, whistle_sound, laser_sound
    
    control_center = np.zeros((SCREEN_HEIGHT, CONTROL_CENTER_WIDTH, 3), dtype=np.uint8)
    
    running = True  # Add this line to define running variable
    
    # Load sounds (this will not crash if files are missing)
    load_sounds()
    
    mqtt_connected = False  # Track MQTT connection status
    client = mqtt.Client(protocol=mqtt.MQTTv5)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    # Initialize MQTT Client with better error handling
    try:
        print(f"Attempting to connect to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        print("MQTT client loop started")
    except Exception as e:
        print(f"Error connecting to MQTT broker: {e}")
        mqtt_connected = False
        # Continue running even without MQTT connection
        pass

    # Set up input devices
    input_devices = find_input_devices()
    input_fds = {dev.fd: dev for dev in input_devices}
    print(f"Using input devices: {[dev.path for dev in input_devices]}")

    # Create display window
    cv2.namedWindow('Missile Command Game', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('Missile Command Game', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setWindowProperty('Missile Command Game', cv2.WND_PROP_TOPMOST, 1)

    # Load the logo
    logo = cv2.imread('logo.png', cv2.IMREAD_UNCHANGED)
    if logo is None:
        print("Warning: Could not load logo.png")
    else:
        # Resize logo to fit in the game area
        logo_height = int(SCREEN_HEIGHT * 0.2)  # 20% of screen height
        logo_width = int(logo.shape[1] * (logo_height / logo.shape[0]))
        logo = cv2.resize(logo, (logo_width, logo_height))

    # Create background
    background = np.zeros((SCREEN_HEIGHT, GAME_SCREEN_WIDTH, 3), dtype=np.uint8)
    if logo is not None:
        # Calculate position to center the logo
        x_offset = (GAME_SCREEN_WIDTH - logo_width) // 2
        y_offset = (SCREEN_HEIGHT - logo_height) // 2
        # Add logo to background
        if logo.shape[2] == 4:  # If logo has an alpha channel
            alpha_s = logo[:, :, 3] / 255.0
            alpha_l = 1.0 - alpha_s
            for c in range(3):
                background[y_offset:y_offset+logo_height, x_offset:x_offset+logo_width, c] = \
                    (alpha_s * logo[:, :, c] + alpha_l * background[y_offset:y_offset+logo_height, x_offset:x_offset+logo_width, c])
        else:
            background[y_offset:y_offset+logo_height, x_offset:x_offset+logo_width] = logo[:, :, :3]

    # Initialize game and controller state
    game_state = GameState()
    controller_state = ControllerState()
    # Spawn initial missiles based on difficulty
    spawn_incoming_missiles(game_state)

    last_time = time.time()

    # Initialize buttons with all required keys
    buttons = {
        'auto': {
            'rect': (10, 10, CONTROL_CENTER_WIDTH//2 - 5, 40),
            'label': 'Auto',
            'active': controller_state.auto_mode
        },
        'manual': {
            'rect': (CONTROL_CENTER_WIDTH//2 + 5, 10, CONTROL_CENTER_WIDTH - 10, 40),
            'label': 'Manual',
            'active': not controller_state.auto_mode
        },
        'display_mode': {
            'rect': (10, 50, CONTROL_CENTER_WIDTH - 10, 80),
            'label': 'Mode'  # Added label
        },
        'accuracy': {
            'rect': (10, 90, CONTROL_CENTER_WIDTH - 10, 120),
            'label': 'Accuracy',
            'value': controller_state.accuracy,
            'minus': (10, 90, 30, 110),
            'plus': (CONTROL_CENTER_WIDTH - 30, 90, CONTROL_CENTER_WIDTH - 10, 110)
        },
        'difficulty': {
            'rect': (10, 130, CONTROL_CENTER_WIDTH - 10, 160),
            'label': 'Difficulty',
            'value': game_state.difficulty,
            'minus': (10, 130, 30, 150),
            'plus': (CONTROL_CENTER_WIDTH - 30, 130, CONTROL_CENTER_WIDTH - 10, 150)
        },
        'pan_aggr': {
            'rect': (10, 170, CONTROL_CENTER_WIDTH - 10, 200),
            'label': 'Pan Aggr',
            'value': controller_state.pan_aggr,
            'minus': (10, 170, 30, 190),
            'plus': (CONTROL_CENTER_WIDTH - 30, 170, CONTROL_CENTER_WIDTH - 10, 190)
        },
        'tilt_aggr': {
            'rect': (10, 210, CONTROL_CENTER_WIDTH - 10, 240),
            'label': 'Tilt Aggr',
            'value': controller_state.tilt_aggr,
            'minus': (10, 210, 30, 230),
            'plus': (CONTROL_CENTER_WIDTH - 30, 210, CONTROL_CENTER_WIDTH - 10, 230)
        },
        'restart': {
            'rect': (10, SCREEN_HEIGHT - 80, CONTROL_CENTER_WIDTH - 10, SCREEN_HEIGHT - 50),
            'label': 'Restart'  # Added label
        },
        'exit': {
            'rect': (10, SCREEN_HEIGHT - 40, CONTROL_CENTER_WIDTH - 10, SCREEN_HEIGHT - 10),
            'label': 'Exit'  # Added label
        },
        'figure8': {
            'rect': (10, 250, CONTROL_CENTER_WIDTH - 10, 280),
            'label': 'Figure-8',
            'active': controller_state.figure8_enabled
        },
    }

    def draw_control_panel(control_center, buttons, controller_state, game_state):
        """Draw a clean, organized control panel based on display mode"""
        # Clear the panel
        control_center.fill(64)  # Dark gray background

        # Common controls at top (mode switch)
        button = buttons['display_mode']
        cv2.rectangle(control_center,
                     (button['rect'][0], button['rect'][1]),
                     (button['rect'][2], button['rect'][3]),
                     (0, 255, 0) if game_state.display_mode == "Live" else (0, 100, 0),
                     -1)
        cv2.putText(control_center, f"Mode: {game_state.display_mode}",
                   (button['rect'][0] + 10, button['rect'][1] + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Auto/Manual toggle for both modes
        for mode in ['auto', 'manual']:
            button = buttons[mode]
            cv2.rectangle(control_center, 
                         (button['rect'][0], button['rect'][1]),
                         (button['rect'][2], button['rect'][3]),
                         (0, 255, 0) if button['active'] else (0, 100, 0), 
                         -1)
            cv2.putText(control_center, button['label'], 
                       (button['rect'][0] + 10, button['rect'][1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if game_state.display_mode == "Game":
            # Game Mode Controls
            # Draw difficulty control
            button = buttons['difficulty']
            cv2.putText(control_center, f"Difficulty: {button['value']}", 
                       (button['rect'][0] + 10, button['rect'][1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            # Draw +/- buttons for difficulty
            for control_button in ['minus', 'plus']:
                cv2.rectangle(control_center, 
                             (button[control_button][0], button[control_button][1]),
                             (button[control_button][2], button[control_button][3]),
                             (0, 100, 0), -1)
                cv2.putText(control_center, '-' if control_button == 'minus' else '+',
                           (button[control_button][0] + 8, button[control_button][1] + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw accuracy control
            button = buttons['accuracy']
            cv2.putText(control_center, f"Accuracy: {button['value']}", 
                       (button['rect'][0] + 10, button['rect'][1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            # Draw +/- buttons for accuracy
            for control_button in ['minus', 'plus']:
                cv2.rectangle(control_center, 
                             (button[control_button][0], button[control_button][1]),
                             (button[control_button][2], button[control_button][3]),
                             (0, 100, 0), -1)
                cv2.putText(control_center, '-' if control_button == 'minus' else '+',
                           (button[control_button][0] + 8, button[control_button][1] + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        else:  # Live Mode Controls
            # Draw pan/tilt aggressiveness controls
            for control in ['pan_aggr', 'tilt_aggr']:
                button = buttons[control]
                cv2.putText(control_center, f"{button['label']}: {button['value']}", 
                           (button['rect'][0] + 10, button['rect'][1] + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                # Draw +/- buttons
                for control_button in ['minus', 'plus']:
                    cv2.rectangle(control_center, 
                                 (button[control_button][0], button[control_button][1]),
                                 (button[control_button][2], button[control_button][3]),
                                 (0, 100, 0), -1)
                    cv2.putText(control_center, '-' if control_button == 'minus' else '+',
                               (button[control_button][0] + 8, button[control_button][1] + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Common controls at bottom
        # Draw restart and exit buttons
        for control in ['restart', 'exit']:
            button = buttons[control]
            cv2.rectangle(control_center, 
                         (button['rect'][0], button['rect'][1]),
                         (button['rect'][2], button['rect'][3]),
                         (0, 100, 0), -1)
            cv2.putText(control_center, button['label'], 
                       (button['rect'][0] + 10, button['rect'][1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Draw MQTT status at the bottom
        mqtt_status = "Connected" if mqtt_connected else "Disconnected"
        cv2.putText(control_center, f"MQTT: {mqtt_status}", 
                   (10, SCREEN_HEIGHT - 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        if game_state.display_mode == "Live":  # Only show figure-8 option in Live mode
            # Draw figure-8 checkbox
            button = buttons['figure8']
            cv2.rectangle(control_center, 
                         (button['rect'][0], button['rect'][1]),
                         (button['rect'][2], button['rect'][3]),
                         (0, 255, 0) if button['active'] else (0, 100, 0), 
                         -1)
            cv2.putText(control_center, button['label'], 
                       (button['rect'][0] + 10, button['rect'][1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Update the main loop to use the new control panel
    while running:
        current_time = time.time()
        elapsed_time = current_time - last_time
        last_time = current_time

        # Handle input events
        if input_fds:
            r, w, x = select.select(input_fds.keys(), [], [], 0)
            for fd in r:
                input_device = input_fds[fd]
                device_name = input_device.name.lower()
                for event in input_device.read():
                    # Skip processing if the event is from a mouse device
                    if 'mouse' in device_name:
                        continue  # Ignore events from mouse devices

                    if event.type == ecodes.EV_KEY:
                        if event.value == 1:  # Key pressed
                            if event.code == ecodes.KEY_LEFT:
                                controller_state.left_pressed = True
                            elif event.code == ecodes.KEY_RIGHT:
                                controller_state.right_pressed = True
                            elif event.code == ecodes.KEY_UP:
                                controller_state.up_pressed = True
                            elif event.code == ecodes.KEY_DOWN:
                                controller_state.down_pressed = True
                            elif event.code == ecodes.KEY_ENTER or event.code == ecodes.BTN_START:
                                controller_state.trigger_pressed = True
                            elif event.code == ecodes.BTN_EAST:  # 'B' button pressed
                                pass  # No action needed
                            elif event.code == ecodes.BTN_SOUTH:
                                controller_state.trigger_pressed = True
                        elif event.value == 0:  # Key released
                            if event.code == ecodes.KEY_LEFT:
                                controller_state.left_pressed = False
                            elif event.code == ecodes.KEY_RIGHT:
                                controller_state.right_pressed = False
                            elif event.code == ecodes.KEY_UP:
                                controller_state.up_pressed = False
                            elif event.code == ecodes.KEY_DOWN:
                                controller_state.down_pressed = False
                            elif event.code == ecodes.KEY_ENTER or event.code == ecodes.BTN_START:
                                controller_state.trigger_pressed = False
                            elif event.code == ecodes.BTN_SOUTH:
                                controller_state.trigger_pressed = False
                    elif event.type == ecodes.EV_ABS:
                        absevent = ecodes.ABS[event.code]
                        value = event.value
                        if absevent == 'ABS_HAT0X':  # D-pad horizontal
                            if value == -1:
                                controller_state.left_pressed = True
                                controller_state.right_pressed = False
                            elif value == 1:
                                controller_state.right_pressed = True
                                controller_state.left_pressed = False
                            else:
                                controller_state.left_pressed = False
                                controller_state.right_pressed = False
                        elif absevent == 'ABS_HAT0Y':  # D-pad vertical
                            if value == -1:
                                controller_state.up_pressed = True
                                controller_state.down_pressed = False
                            elif value == 1:
                                controller_state.down_pressed = True
                                controller_state.up_pressed = False
                            else:
                                controller_state.up_pressed = False
                                controller_state.down_pressed = False
                    elif event.type == ecodes.EV_REL:
                        # Optional: Remove or comment out this section to ignore EV_REL events
                        pass  # Ignore relative movement events

        # Handle keyboard input (exit on 'q' or ESC)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("Exiting game.")
            break

        # Handle mouse clicks and touch events
        def mouse_callback(event, x, y, flags, param):
            global controller_state, game_state
            nonlocal running

            # Only process touchscreen taps (left button down) in the control panel area
            if event == cv2.EVENT_LBUTTONDOWN and x < CONTROL_CENTER_WIDTH:
                # Process control panel button presses
                for button_name, button_info in buttons.items():
                    x1, y1, x2, y2 = button_info['rect']
                    if x1 <= x <= x2 and y1 <= y <= y2:
                        if button_name == 'auto':
                            controller_state.auto_mode = True
                            buttons['auto']['active'] = True
                            buttons['manual']['active'] = False
                        elif button_name == 'manual':
                            controller_state.auto_mode = False
                            buttons['auto']['active'] = False
                            buttons['manual']['active'] = True
                        elif button_name == 'display_mode':
                            game_state.display_mode = "Live" if game_state.display_mode == "Game" else "Game"
                        elif button_name == 'restart':
                            game_state = GameState()
                            spawn_incoming_missiles(game_state)
                        elif button_name == 'exit':
                            running = False
                        elif button_name == 'accuracy':
                            if x <= x1 + 30:  # Minus button
                                controller_state.accuracy = max(0, controller_state.accuracy - 10)
                                buttons['accuracy']['value'] = controller_state.accuracy
                            elif x >= x2 - 30:  # Plus button
                                controller_state.accuracy = min(100, controller_state.accuracy + 10)
                                buttons['accuracy']['value'] = controller_state.accuracy
                        elif button_name == 'difficulty':
                            if x <= x1 + 30:  # Minus button
                                game_state.difficulty = max(1, game_state.difficulty - 1)
                                buttons['difficulty']['value'] = game_state.difficulty
                            elif x >= x2 - 30:  # Plus button
                                game_state.difficulty = min(10, game_state.difficulty + 1)
                                buttons['difficulty']['value'] = game_state.difficulty
                        elif button_name == 'pan_aggr':
                            if x <= x1 + 30:  # Minus button
                                controller_state.pan_aggr = max(1.0, controller_state.pan_aggr - 1.0)
                                buttons['pan_aggr']['value'] = controller_state.pan_aggr
                                print(f"Pan aggressiveness decreased to {controller_state.pan_aggr}")
                            elif x >= x2 - 30:  # Plus button
                                controller_state.pan_aggr = min(20.0, controller_state.pan_aggr + 1.0)
                                buttons['pan_aggr']['value'] = controller_state.pan_aggr
                                print(f"Pan aggressiveness increased to {controller_state.pan_aggr}")
                        elif button_name == 'tilt_aggr':
                            if x <= x1 + 30:  # Minus button
                                controller_state.tilt_aggr = max(1.0, controller_state.tilt_aggr - 1.0)
                                buttons['tilt_aggr']['value'] = controller_state.tilt_aggr
                                print(f"Tilt aggressiveness decreased to {controller_state.tilt_aggr}")
                            elif x >= x2 - 30:  # Plus button
                                controller_state.tilt_aggr = min(20.0, controller_state.tilt_aggr + 1.0)
                                buttons['tilt_aggr']['value'] = controller_state.tilt_aggr
                                print(f"Tilt aggressiveness increased to {controller_state.tilt_aggr}")
                        elif button_name == 'figure8':
                            controller_state.figure8_enabled = not controller_state.figure8_enabled
                            buttons['figure8']['active'] = controller_state.figure8_enabled

            # Ignore all other mouse events
            return

        cv2.setMouseCallback('Missile Command Game', mouse_callback)

        # Update controller state based on mode
        if game_state.display_mode == "Game":
            if controller_state.auto_mode:
                # Remove any player missiles that have reached their target
                remove_completed_player_missiles(game_state)

                # Check if there are any intact buildings before proceeding
                intact_buildings = [b for b in game_state.buildings if b['intact']]
                if intact_buildings and game_state.incoming_missiles:  
                    current_time = time.time()
                    
                    # Sort incoming missiles by proximity to the buildings
                    incoming_sorted = sorted(game_state.incoming_missiles, 
                                          key=lambda m: min(hypot(m['end_x'] - b['x'], m['end_y'] - (SCREEN_HEIGHT - 40)) 
                                                              for b in intact_buildings))
                    
                    # At 100% accuracy, implement precise targeting
                    if controller_state.accuracy == 100:
                        # Find the first active incoming missile that needs attention
                        if game_state.current_target is None and incoming_sorted:
                            # Pick new target
                            game_state.current_target = incoming_sorted[0]
                            # Calculate perfect intercept point
                            intercept_x, intercept_y = calculate_intercept_point(
                                game_state.current_target,
                                GAME_SCREEN_WIDTH // 2,
                                SCREEN_HEIGHT - 10,
                                400  # Player missile speed
                            )
                            
                            # Take one perfect shot
                            player_missile = fire_player_missile(game_state, controller_state, client, intercept_x, intercept_y)
                            if player_missile:
                                game_state.current_target = None  # Clear target and move to next
                    else:
                        # Existing lower accuracy behavior remains unchanged
                        untargeted_missiles = [m for m in incoming_sorted if not any(pm for pm in game_state.player_missiles 
                                                           if abs(pm['end_x'] - m['current_x']) < 10 and abs(pm['end_y'] - m['current_y']) < 10)]
                        
                        for target_missile in untargeted_missiles[:available_shots]:
                            intercept_x, intercept_y = calculate_intercept_point(
                                target_missile,
                                GAME_SCREEN_WIDTH // 2,
                                SCREEN_HEIGHT - 10,
                                400
                            )
                            nearby_targets = [pm for pm in game_state.player_missiles 
                                              if abs(pm['end_x'] - intercept_x) < 60 and abs(pm['end_y'] - intercept_y) < 60]
                            if len(nearby_targets) < max(1, int((100 - controller_state.accuracy) / 20)):
                                controller_state.target_x = intercept_x
                                controller_state.target_y = intercept_y
                                fire_player_missile(game_state, controller_state, client, intercept_x, intercept_y)

                # Move the reticle smoothly towards the target
                dx = controller_state.target_x - controller_state.dot_x
                dy = controller_state.target_y - controller_state.dot_y
                distance = hypot(dx, dy)
                if distance > 0:
                    move_distance = min(distance, controller_state.reticle_speed * elapsed_time)
                    controller_state.dot_x += (dx / distance) * move_distance
                    controller_state.dot_y += (dy / distance) * move_distance

                # Calculate pan and tilt angles based on the reticle position
                controller_state.pan_angle, controller_state.tilt_angle = calculate_pan_tilt_angles(
                    controller_state.dot_x, controller_state.dot_y, controller_state
                )

                # Send the controller state to MQTT
                send_mqtt_command(client, controller_state)

            else:
                # Manual control in Game mode
                if controller_state.left_pressed:
                    controller_state.dot_x -= controller_state.pan_speed * elapsed_time
                if controller_state.right_pressed:
                    controller_state.dot_x += controller_state.pan_speed * elapsed_time
                if controller_state.up_pressed:
                    controller_state.dot_y -= controller_state.tilt_speed * elapsed_time
                if controller_state.down_pressed:
                    controller_state.dot_y += controller_state.tilt_speed * elapsed_time

                # Ensure aiming reticle stays within bounds
                controller_state.dot_x = max(0, min(controller_state.dot_x, GAME_SCREEN_WIDTH - 1))
                controller_state.dot_y = max(0, min(controller_state.dot_y, SCREEN_HEIGHT - 1))

                # Handle firing
                if controller_state.trigger_pressed:
                    fire_player_missile(game_state, controller_state, client)
                    controller_state.trigger_pressed = False  # Reset trigger to prevent continuous firing

                # Calculate pan and tilt angles based on the reticle position
                controller_state.pan_angle, controller_state.tilt_angle = calculate_pan_tilt_angles(
                    controller_state.dot_x, controller_state.dot_y, controller_state
                )

                # Send the controller state to MQTT
                send_mqtt_command(client, controller_state)

        else:  # Live mode
            if controller_state.auto_mode:
                # Live mode auto-tracking logic
                with frame_lock:
                    if latest_frame is not None:
                        current_frame = latest_frame.copy()
                        current_frame = cv2.resize(current_frame, (GAME_SCREEN_WIDTH, SCREEN_HEIGHT))
                        # Process frame to detect green objects
                        display_image, center_x, center_y, bounding_box = detect_green_object(current_frame, controller_state)
                        if center_x is not None and center_y is not None:
                            # Update servo position to track green object
                            new_pan, new_tilt = update_servo_position(
                                center_x, 
                                center_y, 
                                controller_state.pan_angle, 
                                controller_state.tilt_angle,
                                controller_state,
                                mode="live"
                            )
                            # Update controller state
                            controller_state.pan_angle = new_pan
                            controller_state.tilt_angle = new_tilt

                            # Check if aim is within bounding box
                            aim_x = GAME_SCREEN_WIDTH // 2
                            aim_y = SCREEN_HEIGHT // 2
                            x, y, w, h = bounding_box
                            if x <= aim_x <= x + w and y <= aim_y <= y + h:
                                controller_state.trigger_pressed = True
                            else:
                                controller_state.trigger_pressed = False

                            # Send MQTT command
                            send_mqtt_command(client, controller_state)
                        else:
                            # No green object detected
                            controller_state.trigger_pressed = False
                            # Optionally handle no detection
                            # Send MQTT command to turn off relay if needed
                            send_mqtt_command(client, controller_state)
                    else:
                        # No frame available
                        controller_state.trigger_pressed = False
                        # Optionally handle no frame
                        # Send MQTT command to turn off relay if needed
                        send_mqtt_command(client, controller_state)
            else:
                # Manual control in Live mode
                if controller_state.left_pressed:
                    controller_state.dot_x -= controller_state.pan_speed * elapsed_time
                if controller_state.right_pressed:
                    controller_state.dot_x += controller_state.pan_speed * elapsed_time
                if controller_state.up_pressed:
                    controller_state.dot_y -= controller_state.tilt_speed * elapsed_time
                if controller_state.down_pressed:
                    controller_state.dot_y += controller_state.tilt_speed * elapsed_time

                # Ensure aiming reticle stays within bounds
                controller_state.dot_x = max(0, min(controller_state.dot_x, GAME_SCREEN_WIDTH - 1))
                controller_state.dot_y = max(0, min(controller_state.dot_y, SCREEN_HEIGHT - 1))

                # Calculate pan and tilt angles based on the reticle position
                controller_state.pan_angle, controller_state.tilt_angle = calculate_pan_tilt_angles(
                    controller_state.dot_x, controller_state.dot_y, controller_state
                )

                # Send the controller state to MQTT
                send_mqtt_command(client, controller_state)

        # Update game objects if in Game mode
        if game_state.display_mode == "Game" and not game_state.game_over:
            update_incoming_missiles(game_state, elapsed_time)
            update_player_missiles(game_state, elapsed_time)
            update_explosions(game_state, elapsed_time)

            # Collision detection
            handle_collisions(game_state)

            # Check for game over
            if all(not b['intact'] for b in game_state.buildings):
                game_state.game_over = True
            elif not game_state.incoming_missiles:
                # All incoming missiles destroyed, spawn next wave
                game_state.wave_number += 1
                spawn_incoming_missiles(game_state)

        # Draw the control panel
        draw_control_panel(control_center, buttons, controller_state, game_state)

        # Create the game or live display
        if game_state.display_mode == "Game":
            display_image = create_game_display(game_state, controller_state, background)
        else:  # Live mode
            with frame_lock:
                if latest_frame is not None:
                    # Create a copy of the frame to avoid threading issues
                    current_frame = latest_frame.copy()
                    # Resize frame to match game display size
                    current_frame = cv2.resize(current_frame, (GAME_SCREEN_WIDTH, SCREEN_HEIGHT))
                    # Process frame to detect green objects
                    display_image, center_x, center_y, bounding_box = detect_green_object(current_frame, controller_state)
                else:
                    # Create blank display if no frame is available
                    display_image = np.zeros((SCREEN_HEIGHT, GAME_SCREEN_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(display_image, "No camera feed available", 
                              (GAME_SCREEN_WIDTH//4, SCREEN_HEIGHT//2), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Combine the control center and display
        combined_display = np.hstack((control_center, display_image))

        # Display the combined image
        cv2.imshow('Missile Command Game', combined_display)

        # Sleep to cap the frame rate
        time.sleep(0.01)

    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()
    for input_device in input_devices:
        input_device.close()
    pygame.mixer.quit()  # Quit the Pygame mixer

def calculate_pan_tilt_angles(dot_x, dot_y, controller_state):
    """
    Calculate the pan and tilt angles required to aim at the reticle position.
    """
    # Map dot_x from 0 to GAME_SCREEN_WIDTH to pan angle from PAN_ANGLE_MIN to PAN_ANGLE_MAX
    pan_angle = ((dot_x / GAME_SCREEN_WIDTH) * PAN_ANGLE_RANGE) + PAN_ANGLE_MIN

    # Map dot_y from 0 to SCREEN_HEIGHT to tilt angle from TILT_ANGLE_MAX to TILT_ANGLE_MIN
    tilt_angle = ((dot_y / SCREEN_HEIGHT) * (-TILT_ANGLE_RANGE)) + TILT_ANGLE_MAX

    # Apply corrections
    pan_angle += controller_state.pan_correction
    tilt_angle += controller_state.tilt_correction

    # Apply reverse settings
    if controller_state.reverse_pan:
        pan_angle = -pan_angle
    if controller_state.reverse_tilt:
        tilt_angle = TILT_ANGLE_MAX - (tilt_angle - TILT_ANGLE_MIN)

    # Use the defined constants here
    pan_angle = max(PAN_ANGLE_MIN, min(PAN_ANGLE_MAX, pan_angle))
    tilt_angle = max(TILT_ANGLE_MIN, min(TILT_ANGLE_MAX, tilt_angle))

    return pan_angle, tilt_angle

def calculate_intercept_point(enemy_missile, player_start_x, player_start_y, player_missile_speed):
    """
    Calculate the intercept point where the player's missile should aim to intercept the enemy missile.
    """
    # Relative position and velocity
    rx = enemy_missile['current_x'] - player_start_x
    ry = enemy_missile['current_y'] - player_start_y

    enemy_dx = enemy_missile['end_x'] - enemy_missile['start_x']
    enemy_dy = enemy_missile['end_y'] - enemy_missile['start_y']
    enemy_total_distance = hypot(enemy_dx, enemy_dy)
    if enemy_total_distance == 0:
        return enemy_missile['current_x'], enemy_missile['current_y']
    enemy_direction_x = enemy_dx / enemy_total_distance
    enemy_direction_y = enemy_dy / enemy_total_distance
    enemy_speed = enemy_missile['speed']
    rvx = enemy_direction_x * enemy_speed
    rvy = enemy_direction_y * enemy_speed

    # Quadratic coefficients
    a = rvx**2 + rvy**2 - player_missile_speed**2
    b = 2 * (rvx * rx + rvy * ry)
    c = rx**2 + ry**2

    # Discriminant
    discriminant = b**2 - 4 * a * c

    if discriminant < 0:
        # Cannot catch up; aim at current position
        return enemy_missile['current_x'], enemy_missile['current_y']
    else:
        sqrt_discriminant = sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)
        t_candidates = [t for t in [t1, t2] if t > 0]
        if not t_candidates:
            # No positive time solution; aim at current position
            return enemy_missile['current_x'], enemy_missile['current_y']
        t = min(t_candidates)

        intercept_x = enemy_missile['current_x'] + rvx * t
        intercept_y = enemy_missile['current_y'] + rvy * t
        return intercept_x, intercept_y

def remove_completed_player_missiles(game_state):
    """Remove player missiles that have reached their target."""
    missiles_to_remove = []
    for missile in game_state.player_missiles:
        dx = missile['end_x'] - missile['start_x']
        dy = missile['end_y'] - missile['start_y']
        total_distance = hypot(dx, dy)
        distance_traveled = hypot(missile['current_x'] - missile['start_x'], missile['current_y'] - missile['start_y'])
        if distance_traveled >= total_distance:
            missiles_to_remove.append(missile)
    for missile in missiles_to_remove:
        game_state.player_missiles.remove(missile)

def calculate_turret_end(building_x, building_y, reticle_x, reticle_y):
    """Calculate the position of the end of the turret."""
    dx = reticle_x - building_x
    dy = reticle_y - building_y
    angle = math.atan2(dy, dx)
    turret_length = 30
    turret_end_x = int(building_x + turret_length * math.cos(angle))
    turret_end_y = int(building_y + turret_length * math.sin(angle))
    return turret_end_x, turret_end_y

def fire_player_missile(game_state, controller_state, mqtt_client, target_x=None, target_y=None):
    """Fire a missile from the turret towards the aiming reticle or a specified target."""
    if len(game_state.player_missiles) < game_state.difficulty + game_state.handicap:
        # Calculate turret position (center building)
        center_building = game_state.buildings[len(game_state.buildings) // 2]
        turret_base_x = center_building['x']
        turret_base_y = SCREEN_HEIGHT - 60 - 20  # Adjust based on your building height

        # Calculate start point (end of turret)
        start_x, start_y = calculate_turret_end(turret_base_x, turret_base_y, 
                                                  controller_state.dot_x, controller_state.dot_y)

        # Determine end point
        if target_x is not None and target_y is not None:
            end_x, end_y = target_x, target_y
        else:
            # Use controller's aiming reticle
            end_x = controller_state.dot_x
            end_y = controller_state.dot_y

        # Adjust for inaccuracy if in auto mode
        if controller_state.auto_mode:
            # Introduce inaccuracy into the missile's end point
            inaccuracy = (100 - controller_state.accuracy) / 100.0
            # Max offset proportional to distance to target
            distance_to_target = hypot(end_x - start_x, end_y - start_y)
            max_offset = distance_to_target * inaccuracy
            angle = random.uniform(0, 2 * np.pi)
            offset_x = max_offset * np.cos(angle)
            offset_y = max_offset * np.sin(angle)

            end_x += offset_x
            end_y += offset_y

            # Ensure end point stays within bounds
            end_x = max(0, min(end_x, GAME_SCREEN_WIDTH - 1))
            end_y = max(0, min(end_y, SCREEN_HEIGHT - 1))

        missile = {
            'start_x': start_x,
            'start_y': start_y,
            'end_x': end_x,
            'end_y': end_y,
            'current_x': start_x,
            'current_y': start_y,
            'target_id': None,  # Add this field to track which missile we're targeting
            'speed': 400  # Add this line to set the missile speed
        }
        
        game_state.player_missiles.append(missile)
        
        if laser_sound:
            laser_sound.play()
            
        return missile  # Return the missile object

    else:
        print("Maximum number of player missiles in the air. Wait for some to explode.")

def update_incoming_missiles(game_state, elapsed_time):
    """Update positions of incoming missiles."""
    missiles_to_remove = []
    for missile in game_state.incoming_missiles:
        dx = missile['end_x'] - missile['start_x']
        dy = missile['end_y'] - missile['start_y']
        total_distance = hypot(dx, dy)
        if total_distance == 0:
            continue  # Avoid division by zero
        direction_x = dx / total_distance
        direction_y = dy / total_distance

        missile['current_x'] += direction_x * missile['speed'] * elapsed_time
        missile['current_y'] += direction_y * missile['speed'] * elapsed_time

        # Calculate distance traveled
        distance_traveled = hypot(missile['current_x'] - missile['start_x'], missile['current_y'] - missile['start_y'])

        # Check if missile has reached or passed its target
        if distance_traveled >= total_distance:
            missiles_to_remove.append(missile)
            # Destroy the building
            for building in game_state.buildings:
                if abs(building['x'] - missile['end_x']) < 1 and building['intact']:
                    building['intact'] = False
                    game_state.explosions.append({'x': missile['end_x'], 'y': missile['end_y'], 'time': 0, 'radius': 0})
                    break

        # Play whistle sound if missile is newly spawned
        if whistle_sound and 'sound_played' not in missile:
            whistle_sound.play()
            missile['sound_played'] = True  # Add a key to the dictionary instead of setting an attribute

    for missile in missiles_to_remove:
        game_state.incoming_missiles.remove(missile)

def update_player_missiles(game_state, elapsed_time):
    """Update positions of player missiles."""
    missiles_to_remove = []
    for missile in game_state.player_missiles:
        dx = missile['end_x'] - missile['start_x']
        dy = missile['end_y'] - missile['start_y']
        total_distance = hypot(dx, dy)
        if total_distance == 0:
            continue  # Avoid division by zero
        direction_x = dx / total_distance
        direction_y = dy / total_distance

        missile['current_x'] += direction_x * missile['speed'] * elapsed_time
        missile['current_y'] += direction_y * missile['speed'] * elapsed_time

        # Calculate distance traveled
        distance_traveled = hypot(missile['current_x'] - missile['start_x'], missile['current_y'] - missile['start_y'])

        # Check if missile has reached or passed its target
        if distance_traveled >= total_distance:
            missiles_to_remove.append(missile)
            # Create an explosion at the missile's position
            game_state.explosions.append({'x': missile['end_x'], 'y': missile['end_y'], 'time': 0, 'radius': 0})
    for missile in missiles_to_remove:
        game_state.player_missiles.remove(missile)

def update_explosions(game_state, elapsed_time):
    """Update explosions and remove them after some time."""
    explosions_to_remove = []
    for explosion in game_state.explosions:
        explosion['time'] += elapsed_time
        # Explosion expands over 0.5 seconds to its maximum radius
        if explosion['radius'] < 30:
            explosion['radius'] += 100 * elapsed_time  # Adjust this value for explosion growth speed
            if explosion['radius'] > 30:
                explosion['radius'] = 30  # Max radius
        if explosion['time'] > 0.6:  # Explosion lasts for 0.6 seconds
            explosions_to_remove.append(explosion)
    for explosion in explosions_to_remove:
        game_state.explosions.remove(explosion)

def handle_collisions(game_state):
    """Detect and handle collisions between explosions and incoming missiles."""
    missiles_to_remove = []

    # Check for collisions between explosions and incoming missiles
    for explosion in game_state.explosions:
        for missile in game_state.incoming_missiles:
            distance = hypot(missile['current_x'] - explosion['x'], missile['current_y'] - explosion['y'])
            if distance < explosion['radius']:
                if missile not in missiles_to_remove:
                    missiles_to_remove.append(missile)
                    # Optionally, create a secondary explosion
                    game_state.explosions.append({'x': missile['current_x'], 'y': missile['current_y'], 'time': 0, 'radius': 0})

    # Check if any missiles have reached their targets
    for missile in game_state.incoming_missiles:
        if missile['current_y'] >= SCREEN_HEIGHT - 40:  # Assuming buildings are 40 pixels tall
            if missile not in missiles_to_remove:
                missiles_to_remove.append(missile)
                # Find the closest building
                closest_building = min(game_state.buildings, key=lambda b: abs(b['x'] - missile['current_x']))
                if closest_building['intact']:
                    closest_building['intact'] = False
                    game_state.explosions.append({'x': missile['current_x'], 'y': missile['current_y'], 'time': 0, 'radius': 0})
                    
                    # Check if the destroyed building was the one with the turret
                    if game_state.buildings.index(closest_building) == len(game_state.buildings) // 2:
                        game_state.game_over = True

    for missile in missiles_to_remove:
        game_state.incoming_missiles.remove(missile)

def draw_buildings(display_image, game_state, controller_state):
    """Draw the buildings at the bottom of the screen."""
    building_width = 40
    building_height = 60
    window_size = 5
    
    for i, building in enumerate(game_state.buildings):
        if building['intact']:
            # Draw main building structure
            cv2.rectangle(display_image, 
                          (int(building['x'] - building_width//2), SCREEN_HEIGHT - building_height),
                          (int(building['x'] + building_width//2), SCREEN_HEIGHT),
                          (0, 255, 0), -1)
            
            # Draw windows
            for row in range(3):
                for col in range(2):
                    window_x = int(building['x'] - building_width//4 + col * building_width//2)
                    window_y = SCREEN_HEIGHT - building_height + 10 + row * 20
                    cv2.rectangle(display_image,
                                  (window_x - window_size, window_y - window_size),
                                  (window_x + window_size, window_y + window_size),
                                  (255, 255, 255), -1)
            
            # Draw turret on the center building
            if i == len(game_state.buildings) // 2:
                turret_base_y = SCREEN_HEIGHT - building_height - 20
                cv2.circle(display_image, (int(building['x']), turret_base_y), 15, (100, 100, 100), -1)
                
                # Calculate turret end position
                turret_end_x, turret_end_y = calculate_turret_end(building['x'], turret_base_y, 
                                                                  controller_state.dot_x, controller_state.dot_y)
                
                # Draw rotating turret
                cv2.line(display_image, (int(building['x']), turret_base_y), 
                         (turret_end_x, turret_end_y), (50, 50, 50), 5)
        else:
            # Draw rubble for destroyed buildings
            for _ in range(20):
                rubble_x = int(building['x'] + random.uniform(-building_width//2, building_width//2))
                rubble_y = int(SCREEN_HEIGHT - random.uniform(0, 20))
                cv2.circle(display_image, (rubble_x, rubble_y), 3, (100, 100, 100), -1)

def draw_incoming_missiles(display_image, game_state):
    """Draw incoming missiles."""
    for missile in game_state.incoming_missiles:
        cv2.line(display_image, 
                 (int(missile['start_x']), int(missile['start_y'])),
                 (int(missile['current_x']), int(missile['current_y'])), (0, 0, 255), 2)
        cv2.circle(display_image, (int(missile['current_x']), int(missile['current_y'])), 3, (0, 0, 255), -1)

def draw_player_missiles(display_image, game_state):
    """Draw player missiles."""
    for missile in game_state.player_missiles:
        cv2.line(display_image, (int(missile['start_x']), int(missile['start_y'])),
                                     (int(missile['current_x']), int(missile['current_y'])), (255, 255, 0), 2)
        cv2.circle(display_image, (int(missile['current_x']), int(missile['current_y'])), 3, (255, 255, 0), -1)

def draw_explosions(display_image, game_state):
    """Draw explosions."""
    for explosion in game_state.explosions:
        radius = int(explosion['radius'])
        if radius > 0:
            cv2.circle(display_image, (int(explosion['x']), int(explosion['y'])), radius, (255, 0, 0), 2)

def create_game_display(game_state, controller_state, background):
    display_image = background.copy()
    draw_buildings(display_image, game_state, controller_state)
    draw_incoming_missiles(display_image, game_state)
    draw_player_missiles(display_image, game_state)
    draw_explosions(display_image, game_state)
    cv2.circle(display_image, (int(controller_state.dot_x), int(controller_state.dot_y)), 5, (0, 0, 255), -1)
    return display_image

def calculate_figure8_offset(time_val, amplitude=2.0):
    """Calculate figure-8 pattern offsets based on time."""
    # Lissajous curve parameters for figure-8
    freq_x = 2.0  # Horizontal frequency
    freq_y = 1.0  # Vertical frequency - half of horizontal for figure-8
    
    # Calculate offsets using parametric equations
    x_offset = amplitude * math.sin(freq_x * time_val)
    y_offset = amplitude * math.sin(freq_y * time_val * 2)
    
    return x_offset, y_offset

def detect_green_object(frame, controller_state):
    # Initialize all variables at the start
    target_x = None
    target_y = None
    x = y = w = h = None
    contours = None
    target_acquired = False

    try:
        # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define green color range
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])

        # Create mask for green color
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If contours found, get the largest one
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate center of the contour
            target_x = x + w//2
            target_y = y + h//2
            
            # Draw rectangle around the target
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Check if aim point is within bounding box
            aim_x = frame.shape[1] // 2
            aim_y = frame.shape[0] // 2
            if x <= aim_x <= x + w and y <= aim_y <= y + h:
                # When crosshair is inside box - turn red AND cycle trigger
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                target_acquired = True
                
                # Handle MQTT when target acquired
                if mqtt_connected and controller_state.auto_mode:
                    current_time = time.time()
                    
                    # Base angles (without figure-8)
                    adjusted_pan = controller_state.pan_angle
                    adjusted_tilt = controller_state.tilt_angle
                    
                    # Add figure-8 motion only if enabled
                    if controller_state.figure8_enabled:
                        x_offset, y_offset = calculate_figure8_offset(current_time)
                        adjusted_pan += x_offset
                        adjusted_tilt += y_offset
                    
                    # Always cycle the trigger when on target
                    data = {
                        'pan_angle': adjusted_pan,
                        'tilt_angle': adjusted_tilt,
                        'relay': 'off' if time.time() % 0.2 < 0.1 else 'on'  # Toggle every 0.1 seconds
                    }
                    client.publish(MQTT_TOPIC_CONTROL, json.dumps(data))
            else:
                # When crosshair is outside box - turn green AND stop trigger
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # BGR format: Green
                target_acquired = False
                
                # Handle MQTT - keep trigger off when not on target
                if mqtt_connected and controller_state.auto_mode:
                    data = {
                        'pan_angle': controller_state.pan_angle,
                        'tilt_angle': controller_state.tilt_angle,
                        'relay': 'off'  # Not shooting when outside box
                    }
                    client.publish(MQTT_TOPIC_CONTROL, json.dumps(data))

        # Draw crosshair at center
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (0, 255, 0), 1)
        cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (0, 255, 0), 1)

    except Exception as e:
        print(f"Error in detect_green_object: {e}")
        # Return safe default values in case of error
        return frame, None, None, (None, None, None, None)

    return frame, target_x, target_y, (x, y, w, h) if contours else (None, None, None, None)

if __name__ == '__main__':
    main_loop()