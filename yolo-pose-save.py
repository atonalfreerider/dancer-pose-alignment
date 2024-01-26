# a script that loads a video from the content folder from a mounted drive, and runs a yolo pose detection on every frame. it saves all the poses to a list and then saves that list to an indented json
# Import the necessary libraries.
import cv2
import json
import numpy as np
import os
from ultralytics import YOLO

# Load a model
model = YOLO(os.path.join('drive/MyDrive/Zouk/3D-Pose', 'yolov8x-pose.pt'))  # load an official model

# Define the path to the video.
video_path = os.path.join('drive/MyDrive/Zouk/3D-Pose/Larissa-Kadu-Recap', '03-Recap-Redshirt.mp4')

# Load the video.
cap = cv2.VideoCapture(video_path)

# Get the number of frames in the video.
num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

def results_to_dict(results):
    frame_detections = []

    for result in results:
        # Extract boxes and keypoints
        boxes = result.boxes.xyxy
        keypoints = result.keypoints

        # Iterate through each detection in the frame
        for box, kp in zip(boxes, keypoints):
            x1, y1, x2, y2 = box[:4]
            width, height = x2 - x1, y2 - y1

            # Extract keypoints data
            keypoints_data = []
            conf = kp.conf.cpu().numpy()[0]

            # Iterate through each keypoint in the detection
            for idx, keypoint in enumerate(kp.xy.cpu().numpy()[0]):
                keypoints_data.append({
                    "Index": idx,
                    "Point": {
                        "X": int(keypoint[0]),
                        "Y": int(keypoint[1]),
                        "IsEmpty": False
                    },
                    "Confidence": float(conf[idx])
                })

            # Assuming the first class in names is the relevant one
            class_id = 0
            class_name = result.names.get(class_id, 'unknown')

            detection_dict = {
                "Keypoints": keypoints_data,
                "Class": {
                    "Id": class_id,
                    "Name": class_name
                },
                "Bounds": {
                    "X": int(x1),
                    "Y": int(y1),
                    "Width": int(width),
                    "Height": int(height),
                    "Location": {
                        "X": int(x1),
                        "Y": int(y1),
                        "IsEmpty": False
                    },
                    "Size": {
                        "Width": int(width),
                        "Height": int(height),
                        "IsEmpty": False
                    },
                    "IsEmpty": False,
                    "Top": int(y1),
                    "Right": int(x2),
                    "Bottom": int(y2),
                    "Left": int(x1)
                }
                # Omitted 'Confidence' for bounding box due to previous IndexError
            }

            frame_detections.append(detection_dict)

    return frame_detections

# Create a list to store the poses.
poses = []

# Iterate over each frame in the video.
for i in range(num_frames):
    print(f'Processing frame {i}/{num_frames}')

    # Read the frame.
    ret, frame = cap.read()

    # Run YOLO pose detection on the frame.
    # Predict with the model
    results = model(frame)  # predict on an image

    # Convert the results to a dictionary.
    results_dict = results_to_dict(results)

    # Add the results to the list of poses.
    poses.append(results_dict)

# Save the poses to a JSON file.
with open('poses.json', 'w') as f:
    json.dump(poses, f, indent = 4)