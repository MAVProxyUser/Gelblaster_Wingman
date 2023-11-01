import depthai as dai
import time
import zmq
import cv2

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

# Create pipeline
pipeline = dai.Pipeline()

# Define mono camera and output
monoLeft = pipeline.createMonoCamera()
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setFps(30)

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

    frame_count = 0
    while True:
        imgFrame = q.get()
        frame_count += 1

        # Periodic time sync every 100 frames
        if frame_count % 100 == 0:
            sync_message = {
                "type": "time_sync",
                "server_time": time.time()
            }
            socket.send_pyobj(sync_message)

        ret, compressed_frame = cv2.imencode('.jpg', imgFrame.getCvFrame())
        
        socket.send_pyobj({
            "type": "frame_data",
            "frame": compressed_frame,
            "frame_number": frame_count,
            "send_timestamp": time.time()
        })

