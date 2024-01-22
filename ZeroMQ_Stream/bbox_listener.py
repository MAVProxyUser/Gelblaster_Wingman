import zmq
import cv2
import numpy as np

# Setup ZeroMQ for frame receiving
context = zmq.Context()

# Socket for receiving frames
frame_socket = context.socket(zmq.SUB)
frame_socket.connect("tcp://localhost:5555")
frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Socket for receiving transformation commands
command_socket = context.socket(zmq.SUB)
command_socket.connect("tcp://localhost:5557")
command_socket.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    # Receive a frame
    frame_data = frame_socket.recv_pyobj()

    # The frame data is compressed and sent as bytes, so we need to decompress it
    nparr = np.frombuffer(frame_data['frame'], np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    # Receive a transformation command
    command = command_socket.recv_pyobj()
    print(f"Received command: {command}")  # Added this line

    # If the command is a bounding box, print the coordinates
    if command['action'] == 'bbox':
        print("Received bounding box command")  # Added this line
        print(f"Received bounding box: {command['details']}")