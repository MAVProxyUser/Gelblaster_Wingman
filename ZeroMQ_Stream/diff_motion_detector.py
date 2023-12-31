import numpy as np
import cv2
import zmq

# Initialize ZMQ context and sockets for frame receiving
context = zmq.Context()
frame_socket = context.socket(zmq.SUB)
frame_socket.connect("tcp://localhost:5555")  # Adjust as per your setup
frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

command_socket = context.socket(zmq.REQ)
command_socket.connect("tcp://localhost:5557")  # Adjust as per your setup

def send_transformation(camera, action, details):
    command = {
        'camera': camera,
        'action': action,
        'details': details,
        'apply_once': True,
        'linger_duration': 1
    }
    command_socket.send_pyobj(command)
    return command_socket.recv_pyobj()

def clear_transformations(camera):
    send_transformation(camera, 'clear', {})

# Clear transformations for all cameras upon connecting
clear_transformations('left')
clear_transformations('right')
clear_transformations('color')

old_frames = {
    'left': None,
    'right': None,
    'color': None
}

while True:
    try:
        # Receive frame message from ZMQ stream
        frame_message = frame_socket.recv_pyobj()
        frame_type = frame_message.get("type")
        frame_bytes = frame_message.get("frame")
        camera = frame_type.split('_')[-1]  # Assuming the type format is "frame_data_camera"

        # Attempt to decode the frame assuming it is JPEG compressed
        nparr = np.frombuffer(frame_bytes, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)  # Decode image as grayscale

        if frame is not None and old_frames[camera] is not None:
            # Compute the absolute difference between the current frame and the previous frame
            diff_frame = cv2.absdiff(old_frames[camera], frame)
            _, thresh = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)  # Threshold to get binary result

            # Find the point of maximum differential
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(thresh)

            # Create a bounding box around the point of maximum change
            x_center, y_center = max_loc
            x_start = max(x_center - 25, 0)
            y_start = max(y_center - 25, 0)
            x_end = min(x_center + 25, frame.shape[1])
            y_end = min(y_center + 25, frame.shape[0])

            bbox_details = {
                'type': 'bbox', 
                'start_point': (int(x_start), int(y_start)), 
                'end_point': (int(x_end), int(y_end)), 
                'color': (255, 0, 0), 
                'thickness': 2
            }

            # Send bounding box details back to the server for display
            send_transformation('left', 'bound', bbox_details)
            send_transformation('right', 'bound', bbox_details)

            # Update the previous frame
            old_frames[camera] = frame

        # Handling keyboard interruption
        if cv2.waitKey(1) == ord('q'):
            break
    except Exception as e:
        print("Encountered error:", e)
        break

# Cleanup
command_socket.close()
frame_socket.close()
context.term()

