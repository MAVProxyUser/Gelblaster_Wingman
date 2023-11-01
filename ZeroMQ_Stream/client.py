import zmq
import cv2
import time

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.1.140:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Initialize time_offset with a default value
time_offset = 0
time_sync_received = False
previous_frame_number = 0
client_frame_count = 0
client_start_time = time.time()

while True:
    data = socket.recv_pyobj()

    if data["type"] == "initial_time_sync" and not time_sync_received:
        server_time = data["server_time"]
        client_time = time.time()
        time_offset = client_time - server_time
        time_sync_received = True
        continue

    if data["type"] == "time_sync":
        server_time = data["server_time"]
        client_time = time.time()
        time_offset = client_time - server_time
        continue

    if data["type"] == "frame_data":
        frame = cv2.imdecode(data["frame"], cv2.IMREAD_COLOR)

        # Adjust for time offset
        adjusted_send_timestamp = data["send_timestamp"] + time_offset

        # Calculate transmission latency (time from sending to receiving)
        transmission_latency = (time.time() - adjusted_send_timestamp) * 1000

        # Independent FPS calculation for client
        client_frame_count += 1
        client_elapsed_time = time.time() - client_start_time
        client_fps = client_frame_count / client_elapsed_time

        # Display latencies and FPS on the frame
        cv2.putText(frame, f"Transmission Latency: {transmission_latency:.2f} ms, Client FPS: {client_fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Received Frame", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

