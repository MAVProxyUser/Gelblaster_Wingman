import depthai as dai
import time
import zmq
import cv2
import argparse
import numpy as np

# Argument parser
parser = argparse.ArgumentParser()
parser.add_argument('--fps', type=int, default=60, help="FPS to set for the server's camera feed")
args = parser.parse_args()

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

# Additional sockets for transformation commands and time sync
transform_socket = context.socket(zmq.REP)
transform_socket.bind("tcp://*:5557")
responder = context.socket(zmq.REP)
responder.bind("tcp://*:5556")

# Create pipeline and define cameras
pipeline = dai.Pipeline()
monoLeft = pipeline.createMonoCamera()
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setFps(args.fps)

monoRight = pipeline.createMonoCamera()
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setFps(args.fps)

colorCam = pipeline.createColorCamera()
colorCam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setFps(args.fps)
colorCam.setPreviewSize(640, 480)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xoutLeft = pipeline.createXLinkOut()
xoutLeft.setStreamName("video_left")
monoLeft.out.link(xoutLeft.input)

xoutRight = pipeline.createXLinkOut()
xoutRight.setStreamName("video_right")
monoRight.out.link(xoutRight.input)

xoutColor = pipeline.createXLinkOut()
xoutColor.setStreamName("video_color")
colorCam.preview.link(xoutColor.input)

# Function to apply transformations
def apply_transformations(frame, transformations, alpha=1.0):
    for transform in transformations:
        color = tuple([int(c * alpha) for c in transform['color']])
        if transform['type'] == 'text':
            adjusted_position = adjust_coordinates(frame, transform['position'])
            cv2.putText(frame, transform['text'], adjusted_position, cv2.FONT_HERSHEY_SIMPLEX,
                        transform['font_scale'], color, transform['thickness'])
        elif transform['type'] == 'bbox':
            adjusted_start_point = adjust_coordinates(frame, transform['start_point'])
            adjusted_end_point = adjust_coordinates(frame, transform['end_point'])
            cv2.rectangle(frame, adjusted_start_point, adjusted_end_point, color, transform['thickness'])
        elif transform['type'] == 'dot':
            adjusted_center = adjust_coordinates(frame, transform['center'])
            cv2.circle(frame, adjusted_center, transform['radius'], color, transform['thickness'])
    return frame

def adjust_coordinates(frame, pos):
    h, w = frame.shape[:2]
    x, y = pos
    return (w - x, h - y)

# Store current transformations for each camera
current_transformations = {
    'left': [],
    'right': [],
    'color': []
}

# Store lingering transformations
lingering_transformations = []

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    qLeft = device.getOutputQueue(name="video_left", maxSize=1, blocking=False)
    qRight = device.getOutputQueue(name="video_right", maxSize=1, blocking=False)
    qColor = device.getOutputQueue(name="video_color", maxSize=1, blocking=False)

    frame_count = 0  # Initialize frame_count

    while True:
        # Get frames from the queues
        imgFrameLeft = qLeft.get()
        frameLeft = imgFrameLeft.getCvFrame()
        imgFrameRight = qRight.get()
        frameRight = imgFrameRight.getCvFrame()
        imgFrameColor = qColor.get()
        frameColor = imgFrameColor.getCvFrame()

        # Convert color frame to black and white
        frameColorBW = cv2.cvtColor(frameColor, cv2.COLOR_BGR2GRAY)

        # Rotate frames once upon capture
        frameLeft = cv2.flip(frameLeft, -1)
        frameRight = cv2.flip(frameRight, -1)
        frameColorBW = cv2.flip(frameColorBW, -1)

        frames = {'left': frameLeft, 'right': frameRight, 'color': frameColorBW}

        # Check for transformation commands
        try:
            transform_command = transform_socket.recv_pyobj(flags=zmq.NOBLOCK)
            if transform_command:
                print(f"Received command: {transform_command}")
                camera = transform_command['camera']
                action = transform_command['action']
                details = transform_command['details']
                apply_once = transform_command.get('apply_once', False)
                linger_duration = transform_command.get('linger_duration', 0)

                if action in ['text', 'bbox', 'dot']:
                    if apply_once:
                        frames[camera] = apply_transformations(frames[camera], [details])
                        if linger_duration > 0:
                            lingering_transformations.append({
                                'camera': camera, 'details': details,
                                'end_time': time.time() + linger_duration
                            })
                    else:
                        current_transformations[camera].append(details)
                elif action == 'clear':
                    current_transformations[camera] = []

                transform_socket.send_pyobj({"status": "success"})
        except zmq.Again:
            pass

        # Apply current transformations
        for cam, transforms in current_transformations.items():
            frames[cam] = apply_transformations(frames[cam], transforms)

        # Handle lingering transformations
        current_time = time.time()
        lingering_transformations = [t for t in lingering_transformations if t['end_time'] > current_time]
        for transform in lingering_transformations:
            alpha = max(0, (transform['end_time'] - current_time) / transform['details'].get('linger_duration', 1))
            frames[transform['camera']] = apply_transformations(frames[transform['camera']], [transform['details']], alpha)

        # Encode and send frames
        for cam, frame in frames.items():
            ret, compressed_frame = cv2.imencode('.jpg', frame)
            socket.send_pyobj({
                "type": f"frame_data_{cam}",
                "frame": compressed_frame.tobytes(),
                "frame_number": frame_count,
                "send_timestamp": time.time(),
                "server_fps": args.fps
            })

        # Listen for a time sync request from the client
        try:
            message = responder.recv_pyobj(flags=zmq.NOBLOCK)
            if message and message["type"] == "client_time_sync":
                responder.send_pyobj({
                    "type": "server_time_sync",
                    "client_timestamp": message["client_timestamp"],
                    "server_timestamp": time.time()
                })
        except zmq.Again:
            pass

        frame_count += 1  # Increment frame_count
