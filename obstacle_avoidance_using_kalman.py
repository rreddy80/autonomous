import cv2
import numpy as np

# Load YOLOv3 pre-trained weights and configuration
net = cv2.dnn.readNetFromDarknet('yolov3.cfg', 'yolov3.weights')
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Create Kalman filter
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = 0.1 * np.eye(4, dtype=np.float32)
kalman.measurementNoiseCov = 0.1 * np.eye(2, dtype=np.float32)

# Initialize video capture from drone's camera
cap = cv2.VideoCapture(0)  # Replace 0 with the appropriate camera index if multiple cameras are connected

while True:
    # Read frame from video capture
    ret, frame = cap.read()

    if not ret:
        break

    # Perform object detection using YOLOv3
    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Process the detected objects
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    # Perform obstacle avoidance using Kalman filter on detected object positions
    for i in range(len(boxes)):
        x, y, w, h = boxes[i]
        object_position = np.array([[x + w / 2], [y + h / 2]], dtype=np.float32)
        kalman.correct(object_position)
        predicted_position = kalman.predict()

        # Draw object position on the frame
        cv2.circle(frame, (int(predicted_position[0]), int(predicted_position[1])), 5, (0, 255, 0), -1)

        # Display object class and confidence
        label = f'{class_ids[i]}: {confidences[i]:.2f}'
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the frame with obstacle positions
    cv2.imshow("Obstacle Avoidance", frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
