#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  4 09:29:26 2025

@author: ntermote
"""

import cv2
import numpy as np
import glob
import os

# Checkerboard size (number of internal corners)
checkerboard_size = (9, 6)

# Size of each square in millimeters (or cm, but be consistent if you want real-world scale)
square_size = 35.15

# Prepare object points (same for all images)
obj_p = np.zeros((np.prod(checkerboard_size), 3), dtype=np.float32)
obj_p[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2)
obj_p *= square_size

# Storage for object points and image points
obj_points = []
img_points = []

# Path to calibration images
calibration_folder = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/checkerboard_calibration'
image_paths = glob.glob(os.path.join(calibration_folder, '*.jpg'))

if not image_paths:
    raise FileNotFoundError(f"No images found in {calibration_folder}")

print(f"Found {len(image_paths)} images for calibration")

# Process each image
for idx, path in enumerate(image_paths):
    img = cv2.imread(path)

    if img is None:
        print(f"⚠️ Warning: Failed to load image: {path}")
        continue

    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if not ret:
        print(f"❌ No corners detected in {path}")
        continue

    print(f"✅ Corners detected in {path}")

    # Save points only if corners found
    img_points.append(corners)
    obj_points.append(obj_p)

    # Optional: Draw and show for debugging
    cv2.drawChessboardCorners(img, checkerboard_size, corners, ret)
    cv2.imshow('Detected Corners', img)
    cv2.waitKey(50)

cv2.destroyAllWindows()

# Final check to avoid crashing
if len(obj_points) == 0 or len(img_points) == 0:
    raise RuntimeError("No valid images with detectable checkerboards were found. Check your images and settings.")

# Use the last processed image's size for calibration
image_shape = gray.shape[::-1]

print(f"\nStarting calibration with {len(obj_points)} valid images...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_shape, None, None)

print("\nCalibration Complete")
print("Camera Matrix:\n", mtx)
print("Distortion Coefficients:\n", dist)

# Test undistortion on one image (for visual confirmation)
test_image_path = image_paths[0]
test_img = cv2.imread(test_image_path)
test_img = cv2.rotate(test_img, cv2.ROTATE_90_COUNTERCLOCKWISE)

undistorted_img = cv2.undistort(test_img, mtx, dist)

cv2.imshow('Original', test_img)
cv2.imshow('Undistorted', undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Optionally, save calibration to file (can load later to avoid recalibrating)
np.savez('camera_calibration.npz', mtx=mtx, dist=dist)
print("\nCalibration saved to 'camera_calibration.npz'")
