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

# Constants
STEAM_DECK_WIDTH = 1280
STEAM_DECK_HEIGHT = 800
CONTROL_CENTER_WIDTH = 200
GAME_SCREEN_WIDTH = STEAM_DECK_WIDTH - CONTROL_CENTER_WIDTH
SCREEN_WIDTH = STEAM_DECK_WIDTH
SCREEN_HEIGHT = STEAM_DECK_HEIGHT

# MQTT Configuration
MQTT_BROKER = "10.42.0.1"
MQTT_PORT = 1883
MQTT_TOPIC_CONTROL = "dpad/commands"
MQTT_TOPIC_CAMERA = "camera/feed"
MQTT_TOPIC_SERVO_STATUS = "servo/status"

# Lock for thread-safe operations
frame_lock = threading.Lock()
latest_frame = None

class GameState:
    def __init__(self):
        self.ducks = []
        self.shots = []
        self.explosions = []
        self.game_over = False
        self.score = 0
        self.level = 1
        self.ammo = 10
        self.display_mode = "Game"

class ControllerState:
    def __init__(self):
        self.dot_x = GAME_SCREEN_WIDTH // 2
        self.dot_y = SCREEN_HEIGHT // 2
        self.pan_speed = 600
        self.tilt_speed = 600
        self.pan_angle = 0.0
        self.tilt_angle = 0.0
        self.trigger_pressed = False
        self.left_pressed = False
        self.right_pressed = False
        self.up_pressed = False
        self.down_pressed = False
        self.target_x = GAME_SCREEN_WIDTH // 2
        self.target_y = SCREEN_HEIGHT // 2
        self.reticle_speed = 500

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(MQTT_TOPIC_CAMERA)
        client.subscribe(MQTT_TOPIC_SERVO_STATUS)
        mqtt_connected = True
    else:
        print(f"Failed to connect to MQTT broker, return code {rc}")
        mqtt_connected = False

def on_message(client, userdata, msg):
    global latest_frame
    if msg.topic == MQTT_TOPIC_CAMERA:
        try:
            nparr = np.frombuffer(msg.payload, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                with frame_lock:
                    latest_frame = frame
            else:
                print("Failed to decode frame from MQTT")
        except Exception as e:
            print(f"Error decoding frame: {e}")

def send_mqtt_command(client, controller_state):
    if mqtt_connected:
        data = {
            'pan_angle': controller_state.pan_angle,
            'tilt_angle': controller_state.tilt_angle,
            'relay': 'on' if controller_state.trigger_pressed else 'off'
        }
        result = client.publish(MQTT_TOPIC_CONTROL, json.dumps(data))
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
                                'BTN_LEFT', 'BTN_RIGHT', 'BTN_MIDDLE'):
                        is_input_device = True
                        break
            if is_input_device:
                break
        if is_input_device or 'gamepad' in device.name.lower() or 'controller' in device.name.lower() \
                or 'touchscreen' in device.name.lower() or 'mouse' in device.name.lower():
            print(f"Found input device: {device.path}, name: {device.name}")
            input_devices.append(device)
    return input_devices

def spawn_ducks(game_state):
    num_ducks = game_state.level + 2
    for _ in range(num_ducks):
        duck = {
            'x': random.randint(0, GAME_SCREEN_WIDTH),
            'y': random.randint(0, SCREEN_HEIGHT // 2),
            'speed_x': random.randint(-100, 100),
            'speed_y': random.randint(-50, 50),
            'size': 30
        }
        game_state.ducks.append(duck)

def update_ducks(game_state, elapsed_time):
    for duck in game_state.ducks:
        duck['x'] += duck['speed_x'] * elapsed_time
        duck['y'] += duck['speed_y'] * elapsed_time

        if duck['x'] < 0 or duck['x'] > GAME_SCREEN_WIDTH:
            duck['speed_x'] *= -1
        if duck['y'] < 0 or duck['y'] > SCREEN_HEIGHT:
            duck['speed_y'] *= -1

def fire_shot(game_state, controller_state):
    if game_state.ammo > 0:
        shot = {
            'x': controller_state.dot_x,
            'y': controller_state.dot_y,
            'time': 0
        }
        game_state.shots.append(shot)
        game_state.ammo -= 1
        print(f"Shot fired at ({shot['x']}, {shot['y']})! Ammo left: {game_state.ammo}")  # Debug print
        
        # Check for direct hits on ducks
        ducks_hit = [duck for duck in game_state.ducks if hypot(duck['x'] - shot['x'], duck['y'] - shot['y']) < duck['size']]
        for duck in ducks_hit:
            game_state.ducks.remove(duck)
            game_state.score += 100
            game_state.explosions.append({'x': duck['x'], 'y': duck['y'], 'time': 0})
            print(f"Duck hit at ({duck['x']}, {duck['y']})! Score: {game_state.score}")  # Debug print
    else:
        print("Out of ammo!")  # Debug print

def update_shots(game_state, elapsed_time):
    shots_to_remove = []
    for shot in game_state.shots:
        shot['time'] += elapsed_time
        if shot['time'] > 0.2:
            shots_to_remove.append(shot)
    for shot in shots_to_remove:
        game_state.shots.remove(shot)

def update_explosions(game_state, elapsed_time):
    explosions_to_remove = []
    for explosion in game_state.explosions:
        explosion['time'] += elapsed_time
        if explosion['time'] > 0.5:
            explosions_to_remove.append(explosion)
    for explosion in explosions_to_remove:
        game_state.explosions.remove(explosion)

def create_game_display(game_state, controller_state, background):
    display_image = background.copy()
    
    # Draw ducks
    for duck in game_state.ducks:
        cv2.circle(display_image, (int(duck['x']), int(duck['y'])), duck['size'], (0, 255, 255), -1)
    
    # Draw explosions
    for explosion in game_state.explosions:
        radius = int(30 * (1 - explosion['time'] / 0.5))
        cv2.circle(display_image, (int(explosion['x']), int(explosion['y'])), radius, (255, 0, 0), -1)
    
    # Draw reticle
    cv2.circle(display_image, (int(controller_state.dot_x), int(controller_state.dot_y)), 20, (0, 255, 0), 2)
    cv2.line(display_image, (int(controller_state.dot_x - 25), int(controller_state.dot_y)),
             (int(controller_state.dot_x + 25), int(controller_state.dot_y)), (0, 255, 0), 2)
    cv2.line(display_image, (int(controller_state.dot_x), int(controller_state.dot_y - 25)),
             (int(controller_state.dot_x), int(controller_state.dot_y + 25)), (0, 255, 0), 2)
    
    return display_image

def create_control_center(game_state):
    control_center = np.ones((SCREEN_HEIGHT, CONTROL_CENTER_WIDTH, 3), dtype=np.uint8) * 128
    cv2.putText(control_center, f"Score: {game_state.score}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(control_center, f"Level: {game_state.level}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(control_center, f"Ammo: {game_state.ammo}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return control_center

def main_loop():
    global controller_state, game_state, client
    global latest_frame, mqtt_connected

    mqtt_connected = False
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        print("MQTT client started")
    except Exception as e:
        print(f"Could not connect to MQTT broker: {e}")
        mqtt_connected = False

    input_devices = find_input_devices()
    input_fds = {dev.fd: dev for dev in input_devices}

    cv2.namedWindow('Duck Hunt Game', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('Duck Hunt Game', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    game_state = GameState()
    controller_state = ControllerState()

    background = np.zeros((SCREEN_HEIGHT, GAME_SCREEN_WIDTH, 3), dtype=np.uint8)
    cv2.rectangle(background, (0, SCREEN_HEIGHT - 100), (GAME_SCREEN_WIDTH, SCREEN_HEIGHT), (34, 139, 34), -1)  # Green ground

    spawn_ducks(game_state)

    last_time = time.time()
    running = True
    while running:
        current_time = time.time()
        elapsed_time = current_time - last_time
        last_time = current_time

        if input_fds:
            r, w, x = select.select(input_fds.keys(), [], [], 0)
            for fd in r:
                for event in input_fds[fd].read():
                    print(f"Event: type={event.type}, code={event.code}, value={event.value}")  # Debug print
                    if event.type == ecodes.EV_KEY:
                        if event.code == ecodes.KEY_LEFT:
                            controller_state.left_pressed = (event.value == 1)
                        elif event.code == ecodes.KEY_RIGHT:
                            controller_state.right_pressed = (event.value == 1)
                        elif event.code == ecodes.KEY_UP:
                            controller_state.up_pressed = (event.value == 1)
                        elif event.code == ecodes.KEY_DOWN:
                            controller_state.down_pressed = (event.value == 1)
                        elif event.code in [ecodes.BTN_SOUTH, ecodes.KEY_SPACE, ecodes.BTN_A, ecodes.BTN_GAMEPAD]:
                            controller_state.trigger_pressed = (event.value == 1)
                            if event.value == 1:
                                fire_shot(game_state, controller_state)

        # Update reticle position
        if controller_state.left_pressed:
            controller_state.dot_x -= controller_state.reticle_speed * elapsed_time
        if controller_state.right_pressed:
            controller_state.dot_x += controller_state.reticle_speed * elapsed_time
        if controller_state.up_pressed:
            controller_state.dot_y -= controller_state.reticle_speed * elapsed_time
        if controller_state.down_pressed:
            controller_state.dot_y += controller_state.reticle_speed * elapsed_time

        controller_state.dot_x = max(0, min(controller_state.dot_x, GAME_SCREEN_WIDTH))
        controller_state.dot_y = max(0, min(controller_state.dot_y, SCREEN_HEIGHT))

        update_ducks(game_state, elapsed_time)
        update_shots(game_state, elapsed_time)
        update_explosions(game_state, elapsed_time)

        if not game_state.ducks:
            game_state.level += 1
            game_state.ammo = 10
            spawn_ducks(game_state)

        display_image = create_game_display(game_state, controller_state, background)
        control_center = create_control_center(game_state)
        combined_display = np.hstack((display_image, control_center))

        cv2.imshow('Duck Hunt Game', combined_display)

        if game_state.ammo <= 0 and not game_state.shots:
            game_state.game_over = True

        if game_state.game_over:
            cv2.putText(combined_display, "GAME OVER", (SCREEN_WIDTH // 2 - 100, SCREEN_HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('Duck Hunt Game', combined_display)
            cv2.waitKey(3000)
            break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        time.sleep(0.01)

    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()

if __name__ == '__main__':
    main_loop()
