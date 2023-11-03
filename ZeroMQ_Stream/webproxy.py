import zmq
import base64
import time
import argparse
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

# Parsing command-line arguments for server host
parser = argparse.ArgumentParser(description='Web proxy for ZeroMQ stream.')
parser.add_argument('--server', type=str, default='localhost', help='Host for the ZeroMQ server.')
args = parser.parse_args()

app = Flask(__name__)
socketio = SocketIO(app)

# ZeroMQ setup
context = zmq.Context()
zmq_socket = context.socket(zmq.SUB)
zmq_socket.connect(f"tcp://{args.server}:5555") 
zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")

@socketio.on('connect')
def client_connected():
    print("Client connected!")

# HTML template directly in Python code
TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>ZeroMQ to WebSocket</title>
</head>
<body>
    <h2>Received Image:</h2>
    <img id="receivedImage" width="640" height="480" alt="Streamed Image">
    <div id="stats">
        <p><strong>Transmission Latency:</strong> <span id="transmissionLatency">N/A</span> ms</p>
        <p><strong>Display FPS:</strong> <span id="clientFPS">N/A</span></p>
        <p><strong>Server FPS:</strong> <span id="serverFPS">N/A</span></p>
    </div>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        let frameCount = 0;
        let startTime = new Date().getTime();
        let timeOffset = 0; // Offset between client and server time

        const socket = io.connect('http://' + window.location.hostname + ':8080');

        // Function to send a time sync request
        function sendTimeSyncRequest() {
            const clientTimestamp = new Date().getTime() / 1000;  // in seconds
            socket.emit('client_time_sync', { client_time: clientTimestamp });
        }

        // Handle server time sync response
        socket.on('server_time_sync', function(data) {
            const serverTimestamp = data.server_time;
            const clientTimestamp = data.client_time;
            const roundTripTime = (new Date().getTime() / 1000) - clientTimestamp;
            const serverTime = serverTimestamp + roundTripTime / 2;  // Estimate server time at the moment of message reception
            const clientTime = new Date().getTime() / 1000;  // Current client time
            timeOffset = serverTime - clientTime;  // Calculate the offset between client and server
        });

        // Periodically send time sync requests
        setInterval(sendTimeSyncRequest, 5000);  // Every 5 seconds

        socket.on('new_image', function(data) {
            const imgElement = document.getElementById('receivedImage');
            const transmissionLatencyElement = document.getElementById('transmissionLatency');
            const clientFPSElement = document.getElementById('clientFPS');
            const serverFPSElement = document.getElementById('serverFPS');

            imgElement.src = "data:image/jpeg;base64," + data.image_data;
            transmissionLatencyElement.textContent = (data.transmission_latency + timeOffset * 1000).toFixed(2);  // Adjust latency with timeOffset

            frameCount++;
            let currentTime = new Date().getTime();
            let elapsedTime = (currentTime - startTime) / 1000;  // in seconds
            let fps = frameCount / elapsedTime;

            clientFPSElement.textContent = fps.toFixed(2);
            serverFPSElement.textContent = data.server_fps;
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(TEMPLATE)

def zmq_to_ws():
    while True:
        data = zmq_socket.recv_pyobj()
        if data["type"] == "frame_data":
            current_time = time.time()
            transmission_latency = (current_time - data["send_timestamp"]) * 1000  # in milliseconds
            
            image_base64 = base64.b64encode(data["frame"]).decode('utf-8')
            socketio.emit('new_image', {
                'image_data': image_base64,
                'transmission_latency': transmission_latency,
                'server_fps': data["server_fps"]
            })

@socketio.on('client_time_sync')
def handle_client_time_sync(data):
    client_timestamp = data['client_time']
    server_timestamp = time.time()
    emit('server_time_sync', {
        'client_time': client_timestamp,
        'server_time': server_timestamp
    })

if __name__ == '__main__':
    socketio.start_background_task(target=zmq_to_ws)
    socketio.run(app, host="0.0.0.0", port=8080)


