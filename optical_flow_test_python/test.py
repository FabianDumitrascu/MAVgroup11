#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dense Optical Flow Visualization with IMU Data
Color encodes the **speed (magnitude)** of motion instead of direction.
The x, y, z position data is displayed based on the closest timestamp.
"""

import cv2
import os
import numpy as np
import pandas as pd
import re
from datetime import datetime

# Path to images folder and IMU data file
folder_path = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/frontcam_flight'  # Adjust path
imu_file_path = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/IMU_data.csv'

# Load IMU data
imu_data = pd.read_csv(imu_file_path)
imu_data['timestamp'] = pd.to_datetime(imu_data['time'])  # Convert timestamps to datetime objects

# Get sorted list of image files
image_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.jpg') or f.endswith('.png')])

print(f"IMU Data Loaded: {len(imu_data)} measurements found.")
print(f"Total number of image files: {len(image_files)}")

# Video writer setup (optional)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_video = cv2.VideoWriter('output_video.avi', fourcc, 30.0, (640, 480))

# Camera calibration matrix (intrinsics)
mtx = np.array([[323.58405676, 0, 262.96788762],
                [0, 323.93936434, 214.11123418],
                [0, 0, 1]])

# Distortion coefficients
dist = np.array([[-3.38129283e-01,  1.34930511e-01, -1.58307236e-04,  3.84031314e-04, -2.78102296e-02]])

# Previous frame placeholder
prev_gray = None

# Function to extract timestamp from image filename (assuming format is like 'image_123456789012345.jpg')
def extract_timestamp_from_filename(filename):
    pattern = r'(\d{8})'  # Matches the 15-digit microsecond timestamp
    match = re.search(pattern, filename)
    if match:
        timestamp_microseconds = int(match.group(1))  # Extract timestamp in microseconds
        return timestamp_microseconds
    return None

# Function to find the closest timestamp in IMU data
def find_closest_data(timestamp_microseconds, imu_data):
    # Ensure that the 'time' column in IMU data is in microseconds (integer type)
    imu_data['time_microseconds'] = (imu_data['time'] * 1_000_000).astype(int)  # Convert IMU time to microseconds

    # Calculate the time difference (absolute value) between image timestamp (in microseconds) and IMU data timestamps (also in microseconds)
    imu_data['time_diff'] = (imu_data['time_microseconds'] - timestamp_microseconds).abs()

    # Find the row with the smallest time difference
    closest_row = imu_data.loc[imu_data['time_diff'].idxmin()]

    # Return the x, y, z positions
    return closest_row['pos_x'], closest_row['pos_y'], closest_row['pos_z']


# Loop through each image
for image_file in image_files:
    # Load and preprocess image
    image = cv2.imread(os.path.join(folder_path, image_file))
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image = cv2.undistort(image, mtx, dist)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Extract timestamp from the image filename
    image_timestamp = extract_timestamp_from_filename(image_file)
    if image_timestamp is None:
        print(f"Skipping image {image_file} due to missing timestamp.")
        continue

    # Find closest IMU data point for the timestamp
    x, y, z = find_closest_data(image_timestamp, imu_data)

    if prev_gray is None:
        prev_gray = gray
        continue  # Skip first frame since there's no previous to compare

    # Compute dense optical flow (Farneback)
    flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None,
                                        0.5, 3, 5, 3, 5, 1.2, 0)

    # Calculate flow magnitude (speed)
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])

    # Normalize magnitude to range 0-255 (for visualization)
    mag_norm = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Map magnitude (speed) directly to color (colormap)
    speed_color = cv2.applyColorMap(mag_norm, cv2.COLORMAP_JET)

    # Overlay speed map onto original image
    result_image = cv2.addWeighted(image, 0.6, speed_color, 0.4, 0)

    # Overlay x, y, z position on the image
    font = cv2.FONT_HERSHEY_SIMPLEX
    position_text = f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}"
    cv2.putText(result_image, position_text,
                (10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # Show and save result
    cv2.imshow('Speed-based Optical Flow with IMU Data', result_image)
    output_video.write(result_image)

    # Wait and allow quitting
    if cv2.waitKey(34) & 0xFF == ord('q'):
        break

    # Update previous frame
    prev_gray = gray

# Cleanup
output_video.release()
cv2.destroyAllWindows()
