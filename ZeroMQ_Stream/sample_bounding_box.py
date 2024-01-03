import zmq
import time

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5557")

# Define the bounding box
bbox = {
    'type': 'bbox',  # Specify the type of transformation
    'start_point': (0, 0),
    'end_point': (100, 100),
    'color': (255, 255, 255),  # White color
    'thickness': 2
}

# Define the movement pattern
pattern = [(i, i) for i in range(0, 500, 10)]  # Diagonal pattern

# Loop over the pattern
for point in pattern:
    # Update the bounding box position
    bbox['start_point'] = point
    bbox['end_point'] = (point[0] + 100, point[1] + 100)

    # Send the bounding box to the server
    socket.send_pyobj({
        'camera': 'left',  # Apply to the left camera
        'action': 'bbox',  # Correct action type
        'details': bbox,
        'apply_once': True  # Apply for one frame only
    })

    # Receive the response from the server
    message = socket.recv_pyobj()
    print(message)

    # Wait for a short period of time before moving the bounding box
    time.sleep(0.1)