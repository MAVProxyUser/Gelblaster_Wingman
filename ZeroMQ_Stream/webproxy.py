import zmq
import base64
from flask import Flask, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# ZeroMQ setup
context = zmq.Context()
zmq_socket = context.socket(zmq.SUB)
zmq_socket.connect("tcp://192.168.1.140:5555")
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

    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        const socket = io.connect(window.location.origin);
        socket.on('new_image', function(data) {
            const imgElement = document.getElementById('receivedImage');
            imgElement.src = "data:image/jpeg;base64," + data.image_data;
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
        frame_data = zmq_socket.recv_pyobj()
        if frame_data["type"] == "frame_data":
            compressed_frame = frame_data["frame"]
            image_base64 = base64.b64encode(compressed_frame).decode('utf-8')
            socketio.emit('new_image', {'image_data': image_base64})

if __name__ == '__main__':
    socketio.start_background_task(target=zmq_to_ws)
    socketio.run(app, host="0.0.0.0", port=8080)



