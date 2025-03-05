#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  3 12:42:22 2025

@author: ntermote
"""

import cv2
import os
import numpy as np
from matplotlib import pyplot as plt

# Optical flow function definition


# Set the folder path where your images are stored
folder_path = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/frontcam_flight'  # Replace this with the actual path

# Get all image files in the folder and sort them (if they are numbered sequentially)
image_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.jpg') or f.endswith('.png')])

# Create a VideoWriter object if you want to save the output video (optional)
# Define the video codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Using XVID codec
output_video = cv2.VideoWriter('output_video.avi', fourcc, 30.0, (640, 480))  # 30 fps, 640x480 resolution

# Calibration matrix camera
mtx = np.array([[323.58405676, 0,262.96788762],
                [0, 323.93936434, 214.11123418],
                [0, 0, 1]])

# Other calibration array (forgot the name)
dist = np.array([[-3.38129283e-01,  1.34930511e-01, -1.58307236e-04,  3.84031314e-04, -2.78102296e-02]])

# Variables for optical flow
prev_gray = None
prev_points = None

# Loop through each image in the folder
for image_file in image_files:
    # Load the image
    image = cv2.imread(os.path.join(folder_path, image_file))
    
    # Optionally, resize the image (if needed) to match the video size
    #image = cv2.resize(image, (640, 480))  # Resize to 640x480 (adjust as needed)
    
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    image = cv2.undistort(image, mtx, dist)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    if prev_gray is None:
        print("check1")
        # Initialize tracking points in the first frame using Shi-Tomasi corner detector
        prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **dict(maxCorners=100, qualityLevel=0.1, minDistance=3, blockSize=7))
        print("check2")
        print(prev_points)
        print(prev_gray)
        prev_gray = gray
        
    # Calculate optical flow (sparse) between the previous and current frame
    if prev_points is not None:
        next_points, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_points, None)
        print("check3")
        if next_points is None or len(next_points) == 0:
            print("No valid points found for optical flow.")
            # Reinitialize the tracking points by finding new features in the current frame
            prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **dict(maxCorners=100, qualityLevel=0.1, minDistance=3, blockSize=7))
            # Recalculate optical flow with the newly initialized points
            next_points, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_points, None)       
        
        # Select good points
        good_new = next_points[status == 1]
        good_old = prev_points[status == 1]
        
        # Draw the flow vectors (arrows)
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            # Draw an arrow: (start_point, end_point)
            image = cv2.arrowedLine(image, (int(c), int(d)), (int(a), int(b)), (0, 255, 0), 2)
     
    else:
        print("No good features found, skipping frame.")
        # You can either skip the frame or use the previous points to draw an arrow.
        # If you want to continue without optical flow, just display the image
        pass

   
    print("check4")
    # Display the image in a window (this is like displaying the frames in a video)
    cv2.imshow('Video', image)
    
    # Write the frame to the video file (optional)
    output_video.write(image)
    
    # Wait for 30ms (adjust the delay for fps control)
    key = cv2.waitKey(100)  # 30ms delay for 30fps
    
    # If the 'q' key is pressed, exit the loop
    if key == ord('q'):
        break

    # Update the previous frame and points for the next iteration
    prev_gray = gray
    prev_points = good_new.reshape(-1, 1, 2)

# Release the video writer and close all OpenCV windows
output_video.release()
cv2.destroyAllWindows()