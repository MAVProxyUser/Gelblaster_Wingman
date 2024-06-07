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

# Optical flow parameters
lk_params = dict(winSize=(31, 31), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=15, blockSize=7)
MOVEMENT_THRESHOLD = 30

old_gray = None
p0 = None

while True:
    frame_data = frame_socket.recv_pyobj()
    frame_bytes = np.frombuffer(frame_data["frame"], dtype=np.uint8)
    frame = cv2.imdecode(frame_bytes, cv2.IMREAD_GRAYSCALE)

    if frame is None or frame.size == 0:
        continue

    if old_gray is not None and (frame.shape[0] != old_gray.shape[0] or frame.shape[1] != old_gray.shape[1]):
        frame = cv2.resize(frame, (old_gray.shape[1], old_gray.shape[0]))

    if old_gray is None:
        old_gray = frame
        p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
        continue

    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame, p0, None, **lk_params)

    if p1 is not None and st.any():
        good_new = p1[st==1]
        good_old = p0[st==1]
        movement = np.linalg.norm(good_new - good_old, axis=1)
        significant_movement = movement > MOVEMENT_THRESHOLD

        if significant_movement.any():
            significant_points = good_new[significant_movement]
            min_x, min_y = np.min(significant_points, axis=0)
            max_x, max_y = np.max(significant_points, axis=0)
            center_point = significant_points.mean(axis=0)

            bbox_details = {'type': 'bbox', 'start_point': (int(min_x), int(min_y)), 'end_point': (int(max_x), int(max_y)), 'color': (255, 0, 0), 'thickness': 2}
            dot_details = {'type': 'dot', 'center': (int(center_point[0]), int(center_point[1])), 'radius': 5, 'color': (255, 0, 0), 'thickness': -1}
            send_transformation('left', 'bound', bbox_details)
            send_transformation('right', 'bound', bbox_details)
            send_transformation('left', 'dot', dot_details)
            send_transformation('right', 'dot', dot_details)

    old_gray = frame.copy()
    p0 = good_new.reshape(-1, 1, 2) if p1 is not None else None


