# File: main.py

import time
import concurrent.futures
import queue
import threading
import cv2
import users
import numpy as np
import apriltag
import Jetson.GPIO as GPIO
from dynamixel_controller import DynamixelController
from motion_tracker import MotionTracker
from coordinate_system import CoordinateSystem

class Application:
    def __init__(self):
        # Create settings window
        self.device_port = "/dev/ttyUSB0"
        self.baudrate = 1000000 
        self.pan_servo_id = 1
        self.tilt_servo_id = 2
        self.tilt_offset = 10
        self.relay_pin = 7
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)  # Setting it to LOW initially

        self.frame_semaphore = threading.Semaphore()
        self.current_frame = None

        self.nnPath = "models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"  # Set the correct path to the YOLO model blob file

        # Initialize components
        self.dynamixel_controller = DynamixelController(self.device_port, self.baudrate, self.pan_servo_id, self.tilt_servo_id)
        self.motion_tracker = MotionTracker(self.nnPath)
        self.coordinate_system = CoordinateSystem()
        self.capturing = True  # This indicates if the capture_frames method is still capturing frames

        self._terminate = False  # Add this line to initialize the attribute
        self.frame_queue = queue.Queue(maxsize=1000)
        self.camera_ready = threading.Event()

        # Initialize Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.process_noise_cov = 6
        self.measurement_noise_cov = 4
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * self.process_noise_cov
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * self.measurement_noise_cov
        self.kalman.statePost = np.zeros((4,1), np.float32)
        self.kalman.statePre = np.zeros((4,1), np.float32)
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        self.kalman.errorCovPre = np.eye(4, dtype=np.float32)


        self.MAX_VALID_PREDICTION = 1000
        self.MIN_VALID_PREDICTION = 0

        self.display_queue = queue.Queue(maxsize=100)

        # Set home and detection timer
        self.home_position = (self.dynamixel_controller.PAN_CENTER_POSITION, self.dynamixel_controller.TILT_CENTER_POSITION)
        print(f"Home Position: {self.home_position}", flush=True)
        self.last_positions = []
        self.start_time = None
        self.last_centroid = None
        self.THRESHOLD_DISTANCE = 5
        self.TIME_LIMIT = 3.0

        self.april_detector = apriltag.Detector()
        self.april_tag_visible = False
        self.flip_horizontal = 1
        self.flip_vertical = 1
        self.tag_confidence_threshold = .7
        self.servo_scale = 1
        self.show_frame = 1
        self.servo_speed = 200
        self.reverse_pan = 0
        self.reverse_tilt = 0
        self.prev_x_pixels = None
        self.prev_y_pixels = None
        self.prev_vx_pixels = None
        self.prev_vy_pixels = None

    def stop(self):
        self._terminate = True

    def monitor_queue(self):
        while not self._terminate:
            print(f"Frame queue size: {self.frame_queue.qsize()}")
            time.sleep(1)

    def capture_frames(self, frame_queue):
        try:
            while self.capturing and not self._terminate:
                for frame, detections in self.motion_tracker.run():
                    frame_queue.put((frame, detections))
                    time.sleep(0.033)  # Introduce a 30 fps rate (or adjust accordingly)
            print("Capture Frames method has stopped.")
        except Exception as e:
            print(f"Error in capture_frames: {e}")
        finally:
            self.capturing = False
            self.camera_ready.set()

    def process_frames(self, frame_queue):
        try:
            while self.capturing or not frame_queue.empty():
                frame, detections = frame_queue.get()
                if frame is not None and frame.any():
                    print("Processing a frame...")
    
                    # Draw detections on frame
                    for detection in detections:
                        xmin = int(detection.xmin * frame.shape[1])
                        ymin = int(detection.ymin * frame.shape[0])
                        xmax = int(detection.xmax * frame.shape[1])
                        ymax = int(detection.ymax * frame.shape[0])
                        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)  # Draw detection in green
                        
                    # Set the current frame
                    self.current_frame = frame
        except Exception as e:
            print(f"Error in process_frames: {e}")

    def update_kalman_filter(self):
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * self.process_noise_cov
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * self.measurement_noise_cov

    def display_frames(self, display_queue):
        self.camera_ready.wait()
        while not display_queue.empty():
            frame = display_queue.get()
            print("Displaying a frame...")
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        print(f"Display Queue Size (End of Display): {display_queue.qsize()}",flush=True)

    def activate_relay(self, duration=2):
            GPIO.output(self.relay_pin, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(self.relay_pin, GPIO.LOW)

    def set_servo_speed(self, servo_id, speed):
        try:
            self.dynamixel_controller.set_speed(servo_id, speed)
        except Exception as e:
            print(f"Failed to set servo speed for servo ID {servo_id}: {e}", flush=True)

    def clamp_servo_position(self, goal, min_position, max_position):
        return self.dynamixel_controller.clamp_servo_position(goal, min_position, max_position)

    def get_bbox_coordinates(self, detection):
        return [detection.xmin, detection.ymin, detection.xmax, detection.ymax]

    def draw_centroid(self, frame, centroid):
        centroid_px = (int(centroid[0] * frame.shape[1]), int(centroid[1] * frame.shape[0]))
        cv2.circle(frame, centroid_px, 5, (0, 255, 0), -1) #GREEN

    def draw_prediction(self, frame, prediction):
        prediction_px = (int(prediction[0]), int(prediction[1]))
        cv2.circle(frame, prediction_px, 5, (255, 0, 0), -1) #BLUE

    def draw_orange_circle(self, frame, centroid):
        center = (int(centroid[0] * frame.shape[1]), int(centroid[1] * frame.shape[0]))
        radius = 10  # Example radius; adjust as necessary
        color = (255, 165, 0)  # RGB for orange 
        thickness = 2  # Thickness of the circle outline
        cv2.circle(frame, center, radius, color, thickness)

    def calculate_velocity(self, centroid):
        if self.prev_x_pixels is not None and self.prev_y_pixels is not None:
            velocity = self.coordinate_system.calculate_velocity(centroid[0], centroid[1], self.prev_x_pixels, self.prev_y_pixels, dt=2)
            print(f"Calculated Velocity: {velocity}", flush=True)
            return velocity
        print("Returning None for velocity", flush=True)
        return None, None

    def calculate_centroid(self, detection):
        return tuple(np.float32(val) for val in [(detection.xmax + detection.xmin) / 2, (detection.ymax + detection.ymin) / 2])

    def is_authorized(self, frame, badge_id):
        # Check if badge ID exists in the database
        if badge_id in users.database:
            font = cv2.FONT_HERSHEY_SIMPLEX
            frame_height, frame_width, _ = frame.shape
    
            # Draw the name
            name = users.database[badge_id]["last_name"] + ", " + users.database[badge_id]["first_name"]
            name_font_scale = 1.5  # Adjust this value to change the size of the name text
            name_text_size = cv2.getTextSize(name, font, name_font_scale, 2)[0]
            name_text_x = (frame_width - name_text_size[0]) // 2
            name_text_y = ((frame_height - name_text_size[1]) // 2)
            cv2.putText(frame, name, (name_text_x, name_text_y), font, name_font_scale, (0, 255, 0), 2, cv2.LINE_AA)
    
            # Draw "AUTHORIZED" below the name
            auth_text_size = cv2.getTextSize("AUTHORIZED", font, 2, 2)[0]
            auth_text_x = (frame_width - auth_text_size[0]) // 2
            auth_text_y = ((frame_height - auth_text_size[1]) // 2) + auth_text_size[1] + 40  # added 40 for spacing
            cv2.putText(frame, "AUTHORIZED", (auth_text_x, auth_text_y), font, 2, (0, 255, 0), 2, cv2.LINE_AA)
    
        return frame

app = Application()

monitor_thread = threading.Thread(target=app.monitor_queue)
monitor_thread.start()

try:
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        future_capture = executor.submit(app.capture_frames, app.frame_queue)
        future_process = executor.submit(app.process_frames, app.frame_queue)
        
        # Display the frames in the main thread
        while not app._terminate:
            if app.current_frame is not None:
                cv2.imshow("Processed Frame", app.current_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
except KeyboardInterrupt:
    print("Stopping capture...")
    app.capturing = False
    future_capture.cancel()  # This will attempt to cancel the capture_frames task
    print("Waiting for all threads to finish...")
    future_capture.result()  # This will wait for the task to actually finish
    future_process.result()
    print("All threads stopped.")
    app.stop()
finally:
    cv2.destroyAllWindows()

