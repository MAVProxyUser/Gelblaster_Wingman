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

# Setup the socket for receiving time sync requests from the client
responder = context.socket(zmq.REP)
responder.bind("tcp://*:5556")

# Create pipeline
pipeline = dai.Pipeline()

# Define mono camera and output
monoLeft = pipeline.createMonoCamera()
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setFps(args.fps)

xout = pipeline.createXLinkOut()
xout.setStreamName("video")
monoLeft.out.link(xout.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    # Initial time sync
    initial_time_message = {
        "type": "initial_time_sync",
        "server_time": time.time()
    }
    socket.send_pyobj(initial_time_message)

    # Initialize background frame for motion detection
    background_frame = None

    # Define constants for motion detection
    MIN_CONTOUR_AREA = 500  # Adjust this value based on your requirements
    BACKGROUND_REFRESH_INTERVAL = 10  # Number of seconds to refresh the background

    # Initialize variables for background refresh
    last_background_update_time = time.time()

    frame_count = 0
    largest_contour = None  # Variable to store the largest contour
    while True:
        imgFrame = q.get()
        frame = imgFrame.getCvFrame()
        frame_count += 1
        current_time = time.time()

        # Convert frame to grayscale
        gray_frame = cv2.GaussianBlur(frame, (21, 21), 0)

        # Refresh background frame after a certain interval
        if background_frame is None or (current_time - last_background_update_time) > BACKGROUND_REFRESH_INTERVAL:
            background_frame = gray_frame
            last_background_update_time = current_time
            print("Background updated.")

        # Calculate frame difference
        frame_delta = cv2.absdiff(background_frame, gray_frame)
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour based on area
        largest_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= MIN_CONTOUR_AREA and area > largest_area:
                largest_area = area
                largest_contour = contour
            else:
                print("No significant contours found.")

        # Draw bounding box for the largest contour
        if largest_contour is not None:
            (x, y, w, h) = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            print(f"Bounding Box: {(x, y, x + w, y + h)}")

        # Periodic time sync every 100 frames
        if frame_count % 100 == 0:
            sync_message = {
                "type": "time_sync",
                "server_time": time.time()
            }
            socket.send_pyobj(sync_message)

        ret, compressed_frame = cv2.imencode('.jpg', frame)

        # Send frame data with motion tracking information
        socket.send_pyobj({
            "type": "frame_data",
            "frame": compressed_frame.tobytes(),
            "frame_number": frame_count,
            "send_timestamp": time.time(),
            "server_fps": args.fps
        })

        # Listen for a time sync request from the client
        try:
            message = responder.recv_pyobj(flags=zmq.NOBLOCK)  # Non-blocking receive
            if message and message["type"] == "client_time_sync":
                # Respond with the current server time and the client's timestamp
                responder.send_pyobj({
                    "type": "server_time_sync",
                    "client_timestamp": message["client_timestamp"],
                    "server_timestamp": time.time()
                })
        except zmq.Again:
            pass  # No message received, continue


