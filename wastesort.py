import os
import time
import cv2
import numpy as np
import tensorflow as tf
import serial
from roboflow import Roboflow
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# 1. Initialize Serial Communication with Arduino
# Make sure to replace 'COM_PORT' with the actual port (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
ser = serial.Serial('COM_PORT', 9600)  # Open serial port at 9600 baud rate

# 2. Download the Dataset from Roboflow
rf = Roboflow(api_key="aNil8uJXRq")  # Using the provided API key
project = rf.workspace("sakib-9bfzb").project("waste-detection-2.0-hwohv")
version = project.version(2)
dataset = version.download("tfrecord")
print("Dataset downloaded to:", dataset.location)

# 3. Set up paths for training data
TRAIN_TFRECORD = os.path.join(dataset.location, "train.record")
TEST_TFRECORD = os.path.join(dataset.location, "test.record")
LABEL_MAP_PATH = os.path.join(dataset.location, "label_map.pbtxt")

# 4. Load a pre-trained model (SSD MobileNet v2)
PRETRAINED_MODEL_URL = 'http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_coco_2018_03_29.tar.gz'
PRETRAINED_MODEL_DIR = 'models/ssd_mobilenet'
os.makedirs(PRETRAINED_MODEL_DIR, exist_ok=True)

!wget -O pretrained_model.tar.gz {PRETRAINED_MODEL_URL}
!tar -zxvf pretrained_model.tar.gz -C {PRETRAINED_MODEL_DIR}

# 5. Load the trained model for inference
model = tf.saved_model.load("models/exported_model/saved_model")

# 6. Load the label map for object detection
def load_label_map(label_map_path):
    label_map = label_map_util.load_labelmap(label_map_path)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=2, use_display_name=True)
    return label_map_util.create_category_index(categories)

category_index = load_label_map(LABEL_MAP_PATH)

# 7. Initialize the video capture (IP Camera URL)
ip_camera_url = "http://your-ip-camera-url/video"  # Replace with your actual IP camera URL
cap = cv2.VideoCapture(ip_camera_url)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Couldn't open the IP camera.")
    exit()

# Function to run object detection
def run_inference_for_single_image(model, image_np):
    image_tensor = model.signatures['serving_default'].inputs[0]
    output_dict = model(image_tensor)
    return output_dict

# Real-time detection loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    # Perform object detection on the frame
    output_dict = run_inference_for_single_image(model, frame)
    
    # Process the output: Visualize the results
    vis_util.visualize_boxes_and_labels_on_image_array(
        frame,
        output_dict['detection_boxes'],
        output_dict['detection_classes'],
        output_dict['detection_scores'],
        category_index,
        instance_masks=output_dict.get('detection_masks', None),
        use_normalized_coordinates=True,
        line_thickness=8)

    # Display the result in the OpenCV window
    cv2.imshow("Waste Classification", frame)
    
    # Check for biodegradable or non-biodegradable class (class IDs can vary depending on your dataset)
    biodegradable_class_id = 1  # You need to adjust this based on your dataset
    non_biodegradable_class_id = 2  # Adjust accordingly
    
    # Check if the waste is biodegradable or non-biodegradable based on detection
    if output_dict['detection_classes'][0] == biodegradable_class_id:
        print("Biodegradable waste detected. Moving left.")
        ser.write(b'left')  # Send 'left' to Arduino (90 degrees left)
    elif output_dict['detection_classes'][0] == non_biodegradable_class_id:
        print("Non-biodegradable waste detected. Moving right.")
        ser.write(b'right')  # Send 'right' to Arduino (90 degrees right)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Close serial connection
ser.close()
