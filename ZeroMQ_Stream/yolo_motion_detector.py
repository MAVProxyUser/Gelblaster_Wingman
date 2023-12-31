import cv2
import numpy as np
import zmq

class MotionTracker:
    def __init__(self, yolo_weights, yolo_cfg, frame_socket, command_socket, detection_socket):
        self.frame_socket = frame_socket
        self.command_socket = command_socket
        self.detection_socket = detection_socket  # Publisher socket for detections
        # Load the YOLO model
        self.net = cv2.dnn.readNet(yolo_weights, yolo_cfg)
        self.layer_names = self.net.getLayerNames()

        layer_output_indices = self.net.getUnconnectedOutLayers()

        # Check the structure of getUnconnectedOutLayers() output and adapt accordingly
        if len(layer_output_indices) > 0 and isinstance(layer_output_indices[0], (np.ndarray, list)):
            # If the elements of the array are arrays themselves (e.g., [[1], [2], [3]])
            self.layer_names = [self.layer_names[i[0] - 1] for i in layer_output_indices]
        else:
            # If the elements of the array are just numbers (e.g., [1, 2, 3])
            self.layer_names = [self.layer_names[i - 1] for i in layer_output_indices]

    def draw_bounding_box(self, frame, class_id, confidence, x, y, x_plus_w, y_plus_h):
        label = str(class_id)
        color = (255, 0, 0)  # Blue color in BGR
        cv2.rectangle(frame, (x, y), (x_plus_w, y_plus_h), color, 2)
        cv2.putText(frame, label, (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def run(self):
        while True:
            try:
                frame_data = self.frame_socket.recv()
                if not frame_data:
                    print("No frame data received!")
                    continue  # Skip this iteration if no frame data is received

                frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), -1)
                
                if frame is None or frame.size == 0:
                    print("Frame could not be decoded or is empty!")
                    continue  # Skip this iteration if the frame is not correctly decoded
                # Check if the frame is decoded properly
                height, width = frame.shape[:2]
                print(f"Received frame resolution: {width}x{height}")

                # Continue with the existing blob and detection logic blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), swapRB=True, crop=False)
                self.net.setInput(blob)
                outs = self.net.forward(self.layer_names)

                detections = []
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.5:  # Confidence threshold
                            # Object detected
                            center_x = int(detection[0] * frame.shape[1])
                            center_y = int(detection[1] * frame.shape[0])
                            w = int(detection[2] * frame.shape[1])
                            h = int(detection[3] * frame.shape[0])

                            # Rectangle coordinates
                            x = center_x - w / 2
                            y = center_y - h / 2

                            # Draw bounding box for the "person" class
                            if class_id == 0:  # Assuming 0 is the ID for "person" in the model
                                self.draw_bounding_box(frame, class_id, confidence, round(x), round(y), round(x+w), round(y+h))
                                detections.append({
                                    'class_id': int(class_id),
                                    'confidence': float(confidence),
                                    'bbox': [round(x), round(y), round(x+w), round(y+h)]
                                })

                # Send detections to the detection_socket
                if detections:
                    self.detection_socket.send_json({
                        'detections': detections
                    })

                # Display result frame
                cv2.imshow("Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit condition
                    break

            except cv2.error as e:
                print(f"OpenCV error: {e}")

if __name__ == "__main__":
    # ZMQ setup
    context = zmq.Context()
    frame_socket = context.socket(zmq.SUB)
    frame_socket.connect("tcp://localhost:5555")
    frame_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    command_socket = context.socket(zmq.REQ)
    command_socket.connect("tcp://localhost:5557")

    # Additional ZMQ setup for detection results
    detection_socket = context.socket(zmq.PUB)
    detection_socket.bind("tcp://*:5558")  # Adjust as needed for your setup

    # Paths to the YOLOv4 model files
    yolo_weights = 'models/yolov4.weights'
    yolo_cfg = 'models/yolov4.cfg'

    motion_tracker = MotionTracker(yolo_weights, yolo_cfg, frame_socket, command_socket, detection_socket)
    motion_tracker.run()

