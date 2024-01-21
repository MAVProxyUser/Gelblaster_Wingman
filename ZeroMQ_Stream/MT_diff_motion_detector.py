import zmq
import cv2
import numpy as np
import threading
import queue
import time

# Setup ZeroMQ for frame receiving
context = zmq.Context()
frame_socket = context.socket(zmq.SUB)
frame_socket.connect("tcp://localhost:5555")
frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Setup ZeroMQ for command sending
command_socket = context.socket(zmq.REQ)
command_socket.connect("tcp://localhost:5557")

# Initialize previous frame
prev_frame = None

# Create a queue to store frames
frame_queue = queue.Queue()

# Function to detect motion
def detect_motion(frame):
    global prev_frame

    # If this is the first frame
    if prev_frame is None:
        prev_frame = frame
        return None

    # Ensure prev_frame and frame have the same shape
    if prev_frame.shape != frame.shape:
        prev_frame = cv2.resize(prev_frame, (frame.shape[1], frame.shape[0]))

    # Compute absolute difference between current frame and previous frame
    diff = cv2.absdiff(prev_frame, frame)

    # Threshold the diff image so that we get the foreground
    threshold_value = 120 # Increased threshold value
    ret, threshold = cv2.threshold(diff, threshold_value, 255, cv2.THRESH_BINARY)

    # Find contours in the threshold image
    contours, _ = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If no contours were found, return None
    if not contours:
        return None

    # Filter out small contours based on area
    min_contour_area = 4000  # Increase this value
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

    # If no contours left after filtering, return None
    if not contours:
        return None

    # Find the largest contour
    largest_contour = max(contours, key=cv2.contourArea)

    # Get the bounding box of the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Update prev_frame
    prev_frame = frame

    return {'type': 'bbox', 'start_point': (x, y), 'end_point': (x+w, y+h), 'color': (255, 255, 255), 'thickness': 2}

# Function to send transformations to the server
def send_transformation(camera, action, details):
    if action == 'motion':
        # Send the bounding box to the server
        command = {
            'camera': camera,
            'action': 'bbox',
            'details': details,
            'apply_once': True
        }
        print(f"Sending command: {command}")  # Debugging output
        command_socket.send_pyobj(command)
        response = command_socket.recv_pyobj()
        print(f"Received response: {response}")  # Debugging output

# Function to handle frame
def handle_frame():
    while True:
        frame_data = frame_socket.recv_pyobj()
        frame_bytes = np.frombuffer(frame_data["frame"], dtype=np.uint8)
        frame = cv2.imdecode(frame_bytes, cv2.IMREAD_GRAYSCALE)

        if frame is None or frame.size == 0:
            continue

        #flip the frame horizontally and vertically
        frame = cv2.flip(frame, -1)

        # Attach a timestamp to the frame and add it to the queue
        timestamp = time.time()
        frame_queue.put((frame, timestamp))

# Function to process frames
def process_frames():
    while True:
        # Get the next frame from the queue
        frame, timestamp = frame_queue.get()

        motion_details = detect_motion(frame)
        if motion_details is not None:
            print(f"Motion detected, sending bbox: {motion_details['start_point']}, {motion_details['end_point']}")
            send_transformation('left', 'motion', motion_details)
        else:
            print("No motion detected.")

# Start the threads for handling and processing frames
frame_thread = threading.Thread(target=handle_frame)
frame_thread.start()

process_thread = threading.Thread(target=process_frames)
process_thread.start()