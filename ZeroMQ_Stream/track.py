import zmq
import numpy as np
from simple_pid import PID

# Setup ZeroMQ for frame receiving
context = zmq.Context()
frame_socket = context.socket(zmq.SUB)
frame_socket.connect("tcp://localhost:5557")  # Change the port to 5557
frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Initialize PID controllers for pan and tilt
pan_pid = PID(0.5, 0.0, 0.0)  # These constants are arbitrary and need tuning
tilt_pid = PID(0.1, 0.0, 0.0)  # These constants are arbitrary and need tuning

def track():
    while True:
        try:
            # Receive the bounding box and frame shape data from the ZMQ stream
            data = frame_socket.recv_pyobj()
            print(f"Received data: {data}")  # Debugging output

            bbox = data['details']['start_point'] + data['details']['end_point']
            frame_shape = (480, 640)  # Assuming a default frame size, replace with actual if available

            # Extract the center point of the bounding box
            x, y, w, h = bbox
            center_point = np.array([x + w / 2, y + h / 2])

            # Update the setpoints of the PID controllers to the center of the frame
            pan_pid.setpoint = frame_shape[1] / 2
            tilt_pid.setpoint = frame_shape[0] / 2

            # Calculate the error values
            pan_error = center_point[0] - pan_pid.setpoint
            tilt_error = center_point[1] - tilt_pid.setpoint

            # Use PID controllers to calculate adjustments
            pan_adjustment = pan_pid(pan_error)
            tilt_adjustment = tilt_pid(tilt_error)

            print(f"Pan adjustment: {pan_adjustment}, Tilt adjustment: {tilt_adjustment}")
        except Exception as e:
            print(f"An error occurred while tracking: {e}")

if __name__ == "__main__":
    track()
