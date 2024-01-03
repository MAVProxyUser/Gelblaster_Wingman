import zmq
import cv2
import numpy as np
from dynamixel_controller import DynamixelController
from coordinate_system import CoordinateSystem
from simple_pid import PID  # You might need to install this package

# Setup ZeroMQ for frame receiving
context = zmq.Context()
frame_socket = context.socket(zmq.SUB)
frame_socket.connect("tcp://localhost:5555")
frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Setup ZeroMQ for command sending
command_socket = context.socket(zmq.REQ)
command_socket.connect("tcp://localhost:5557")

# Initialize Dynamixel Controller
dynamixel_controller = DynamixelController("/dev/ttyUSB0", 1000000, 1, 2)

# Create a FAST feature detector object
fast_detector = cv2.FastFeatureDetector_create(threshold=80, nonmaxSuppression=True)

# Optical flow parameters
lk_params = dict(winSize=(10, 10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 25, 0.03))
MOVEMENT_THRESHOLD = 3  # Initial movement threshold
MIN_NUM_FEATURES = 1  # Minimum number of features to track

old_frame = None
p0 = None

# Initialize PID controllers for pan and tilt
pan_pid = PID(0.5, 0.0, 0.0, setpoint=0)  # These constants are arbitrary and need tuning
tilt_pid = PID(0.1, 0.0, 0.0, setpoint=0)  # These constants are arbitrary and need tuning

# Home position for pan and tilt
PAN_HOME = (2500 + 9000) // 2
TILT_HOME = (400 + 1400) // 2

# Detection source camera (left or right)
DETECTION_SOURCE = 'left'  # Change to 'right' to use the right camera

# Function to send transformations to the server
def send_transformation(camera, action, details):
    command = {
        'camera': camera,
        'action': action,
        'details': details,
        'apply_once': True,
        'linger_duration': 1
    }
    command_socket.send_pyobj(command)
    response = command_socket.recv_pyobj()

def clear_transformations(camera):
    send_transformation(camera, 'clear', {})

# Clear transformations for all cameras upon connecting
clear_transformations('left')
clear_transformations('right')
clear_transformations('color')

def drive_dynamixels_and_display_roi():
    global old_frame, p0  # Use the global variables

    # Set Dynamixels to home position
    dynamixel_controller.set_goal_position(PAN_HOME, TILT_HOME)

    while True:
        frame_data = frame_socket.recv_pyobj()
        frame_bytes = np.frombuffer(frame_data["frame"], dtype=np.uint8)
        
        # Select the camera feed based on DETECTION_SOURCE
        frame = cv2.imdecode(frame_bytes, cv2.IMREAD_GRAYSCALE)  # Decode the frame from the selected camera

        if frame is None or frame.size == 0:
            continue

        if old_frame is not None:
            frame = cv2.resize(frame, (old_frame.shape[1], old_frame.shape[0]))

        if old_frame is None or len(p0) < MIN_NUM_FEATURES:
            old_frame = frame.copy()
            keypoints = fast_detector.detect(old_frame, None)
            p0 = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)

        p1, st, err = cv2.calcOpticalFlowPyrLK(old_frame, frame, p0, None, **lk_params)

        if p1 is not None and st.any():
            good_new = p1[st == 1]
            good_old = p0[st == 1]

            movement = np.linalg.norm(good_new - good_old, axis=1)
            significant_movement = movement > MOVEMENT_THRESHOLD

            if significant_movement.any():
                # Detected significant movement
                significant_points = good_new[significant_movement]
                center_point = significant_points.mean(axis=0)

                # Convert the displacement from the center to servo commands
                pan_error = center_point[0] - frame.shape[1] / 2
                tilt_error = center_point[1] - frame.shape[0] / 2
                
                # Use PID controllers to calculate adjustments
                pan_adjustment = pan_pid(pan_error)
                tilt_adjustment = tilt_pid(tilt_error)

                # Update Dynamixel positions
                new_pan_position = PAN_HOME + pan_adjustment
                new_tilt_position = TILT_HOME + tilt_adjustment

                # Apply limits and send commands to Dynamixels
                new_pan_position = max(2500, min(new_pan_position, 9000))
                new_tilt_position = max(400, min(new_tilt_position, 1400))
                dynamixel_controller.set_goal_position(new_pan_position, new_tilt_position)

                # Only sending dot details for significant movements
                dot_details = {
                    'type': 'dot',
                    'center': (int(center_point[0]), int(center_point[1])),
                    'radius': 5,
                    'color': (255, 0, 0),
                    'thickness': -1
                }
                send_transformation(DETECTION_SOURCE, 'dot', dot_details)  # Send to the camera used for detection

            p0 = good_new.reshape(-1, 1, 2)

        old_frame = frame.copy()

if __name__ == "__main__":
    try:
        drive_dynamixels_and_display_roi()  # Start the main function
    except KeyboardInterrupt:
        print("Interrupted by user, closing...")
    finally:
        dynamixel_controller.close()

