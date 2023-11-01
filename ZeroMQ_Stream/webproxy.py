import zmq
import base64
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
        <p><strong>Client FPS:</strong> <span id="clientFPS">N/A</span></p>
        <p><strong>Server FPS:</strong> <span id="serverFPS">N/A</span></p>
    </div>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        const socket = io.connect(window.location.origin);
        socket.on('new_image', function(data) {
            const imgElement = document.getElementById('receivedImage');
            const transmissionLatencyElement = document.getElementById('transmissionLatency');
            const clientFPSElement = document.getElementById('clientFPS');
            const serverFPSElement = document.getElementById('serverFPS');

            imgElement.src = "data:image/jpeg;base64," + data.image_data;
            transmissionLatencyElement.textContent = data.transmission_latency;
            clientFPSElement.textContent = data.client_fps;
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
            image_base64 = base64.b64encode(data["frame"]).decode('utf-8')
            socketio.emit('new_image', {
                'image_data': image_base64,
                'transmission_latency': data["send_timestamp"],
                'client_fps': "N/A",  # You can modify this as needed
                'server_fps': data["server_fps"]
            })

if __name__ == '__main__':
    socketio.start_background_task(target=zmq_to_ws)
    socketio.run(app, host="0.0.0.0", port=8080)

