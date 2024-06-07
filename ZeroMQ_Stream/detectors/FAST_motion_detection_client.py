import zmq
import cv2
import numpy as np

# ZeroMQ setup for receiving frames
context = zmq.Context()
frame_socket = context.socket(zmq.SUB)
frame_socket.connect("tcp://localhost:5555")
frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# ZeroMQ setup for sending commands
command_socket = context.socket(zmq.REQ)
command_socket.connect("tcp://localhost:5557")

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

# Create a FAST feature detector object
fast_detector = cv2.FastFeatureDetector_create(threshold=80, nonmaxSuppression=True)

# Optical flow parameters
lk_params = dict(winSize=(10, 10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.02))
MOVEMENT_THRESHOLD = 5  # Initial movement threshold
MIN_NUM_FEATURES = 1  # Minimum number of features to track

old_frame = None
p0 = None

while True:
    frame_data = frame_socket.recv_pyobj()
    frame_bytes = np.frombuffer(frame_data["frame"], dtype=np.uint8)
    frame = cv2.imdecode(frame_bytes, cv2.IMREAD_GRAYSCALE)

    if frame is None or frame.size == 0:
        continue

    if old_frame is not None:
        frame = cv2.resize(frame, (old_frame.shape[1], old_frame.shape[0]))

    # Re-detect features only if necessary
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

            # Only sending dot details for significant movements
            dot_details = {'type': 'dot', 'center': (int(center_point[0]), int(center_point[1])), 'radius': 5, 'color': (255, 0, 0), 'thickness': -1}
            send_transformation('left', 'dot', dot_details)
            send_transformation('right', 'dot', dot_details)

        # Update the points for the next frame
        p0 = good_new.reshape(-1, 1, 2)

    old_frame = frame.copy()

