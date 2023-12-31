import zmq
import argparse

# Argument parser for command line arguments
parser = argparse.ArgumentParser(description='Send transformation commands to the video server.')
parser.add_argument('--server', type=str, default='localhost', help='IP address of the server.')
args = parser.parse_args()

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://{args.server}:5557")

def send_transformation(camera, action, details, apply_once=False, linger_duration=0):
    command = {
        'camera': camera,
        'action': action,
        'details': details,
        'apply_once': apply_once,
        'linger_duration': linger_duration
    }
    socket.send_pyobj(command)
    response = socket.recv_pyobj()
    print(f"Server response: {response}")

# Example usage:

# Add a text transformation on the left camera
send_transformation(camera='left', action='text', details={
    'type': 'text', 
    'text': 'Hello, Left Camera!', 
    'position': (50, 50), 
    'font_scale': 1, 
    'color': (255, 255, 255), 
    'thickness': 2
}, apply_once=True, linger_duration=3)

# Add a bounding box on the right camera
send_transformation(camera='right', action='bound', details={
    'type': 'bbox', 
    'start_point': (100, 100), 
    'end_point': (200, 200), 
    'color': (0, 255, 0), 
    'thickness': 3
}, apply_once=False)

# Add a dot on the color camera with lingering effect
send_transformation(camera='color', action='dot', details={
    'type': 'dot', 
    'center': (320, 240), 
    'radius': 10, 
    'color': (0, 0, 255), 
    'thickness': -1
}, apply_once=True, linger_duration=2)

# Clear all transformations on the left camera
send_transformation(camera='left', action='clear', details={})

# Clear all transformations on the right camera
send_transformation(camera='right', action='clear', details={})

# Clear all transformations on the color camera
send_transformation(camera='color', action='clear', details={})



