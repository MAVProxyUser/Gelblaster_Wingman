import zmq
import base64
import time
from flask import Flask, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)

# ZeroMQ setup
context = zmq.Context()
zmq_socket = context.socket(zmq.SUB)
zmq_socket.connect("tcp://localhost:5555")
zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")

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
        const socket = io.connect(window.location.origin);
        socket.on('new_image', function(data) {
            const imgElement = document.getElementById('receivedImage');
            const transmissionLatencyElement = document.getElementById('transmissionLatency');
            const clientFPSElement = document.getElementById('clientFPS');
            const serverFPSElement = document.getElementById('serverFPS');

            imgElement.src = "data:image/jpeg;base64," + data.image_data;
            transmissionLatencyElement.textContent = data.transmission_latency.toFixed(2);
            
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

if __name__ == '__main__':
    socketio.start_background_task(target=zmq_to_ws)
    socketio.run(app, host="0.0.0.0", port=8080)


