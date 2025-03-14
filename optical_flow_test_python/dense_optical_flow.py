import cv2
import numpy as np
import pandas as pd
import os
import re

# Load IMU data
imu_file_path = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/IMU_data.csv'
imu_data = pd.read_csv(imu_file_path)
imu_data['time_microseconds'] = (imu_data['time'] * 1_000_000).astype(int)  # Convert IMU times to microseconds

# Path to images folder
folder_path = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/frontcam_flight'
image_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.jpg') or f.endswith('.png')])

# Camera intrinsic parameters (as given in your case)
focal_length_x = 1000  # Example value, replace with actual focal length
focal_length_y = 1000  # Example value, replace with actual focal length
cx = 640  # Example value, replace with actual cx
cy = 480  # Example value, replace with actual cy

mtx = np.array([[focal_length_x, 0, cx],
              [0, focal_length_y, cy],
              [0, 0, 1]])

# Distortion coefficients (if you have them)
dist = np.array([-0.2, 0.1, 0, 0, 0])

# Initialize previous frame placeholder
prev_gray = None
prev_timestamp = None

# Helper: Compute rotational flow per pixel
def compute_rotational_flow(shape, gyro_x, gyro_y, gyro_z, dt, cx, cy):
    h, w = shape
    fx, fy = focal_length_x, focal_length_y

    # Create mesh grid for each pixel
    y, x = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    
    # Normalize pixel coordinates to camera frame
    x = (x - cx) / fx
    y = (y - cy) / fy

    omega = np.array([gyro_x, gyro_y, gyro_z])

    # Compute the rotational flow in x and y directions
    flow_x = dt * (x * omega[2] - y * omega[0] + (1 + x**2) * omega[1])
    flow_y = dt * (-x * omega[0] - y * omega[2] + (1 + y**2) * omega[0])

    return np.dstack((flow_x, flow_y)).astype(np.float32)

# Function to extract timestamp from image filename
def extract_timestamp(image_file):
    pattern = r'(\d{8})'  # Assuming filename contains timestamp in microseconds
    match = re.search(pattern, image_file)
    if match:
        timestamp_microseconds = int(match.group(1))  # Extract timestamp in microseconds
        return timestamp_microseconds
    return None

# Function to find closest IMU data to a given timestamp in microseconds
def find_closest_data(timestamp_microseconds, imu_data):
    imu_data['time_diff'] = (imu_data['time_microseconds'] - timestamp_microseconds).abs()
    closest_row = imu_data.loc[imu_data['time_diff'].idxmin()]
    return closest_row['pos_x'], closest_row['pos_y'], closest_row['pos_z'], closest_row['rate_p'], closest_row['rate_q'], closest_row['rate_r']

# Compute depth from translational flow using known velocity and focal length
def compute_depth(flow, vel_x, vel_y, vel_z):
    # Compute the magnitude of the translational flow (2D flow magnitude)
    flow_magnitude = np.linalg.norm(flow, axis=2)

    # Use the formula to compute depth: depth = (focal_length * velocity) / flow_magnitude
    velocity_magnitude = np.linalg.norm([vel_x, vel_y, vel_z])  # Total velocity magnitude
    depth = (focal_length_x * velocity_magnitude) / (flow_magnitude + 1e-5)  # Add a small value to avoid division by zero

    # Return depth for each pixel in the image
    return depth

# Inside the main loop where the flow is computed:
for image_file in image_files:
    # Load and preprocess image
    image = cv2.imread(os.path.join(folder_path, image_file))
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image = cv2.undistort(image, mtx, dist)
    
    # Convert to grayscale and ensure it's single-channel
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Ensure both images are single-channel grayscale images
    if gray.ndim != 2:
        print(f"Skipping image {image_file} due to incorrect number of channels.")
        continue

    # Extract timestamp from the image filename
    image_timestamp = extract_timestamp(image_file)
    if image_timestamp is None:
        print(f"Skipping image {image_file} due to missing timestamp.")
        continue

    # Get closest IMU data for this timestamp
    vel_x, vel_y, vel_z, gyro_x, gyro_y, gyro_z = find_closest_data(image_timestamp, imu_data)
    
    # Compute time difference between current and previous image
    if prev_timestamp is not None:
        dt = (image_timestamp - prev_timestamp) * 1e-6  # dt in seconds
    else:
        dt = 0  # No previous timestamp, assume 0 for the first image

    # Ensure that prev_gray and gray are the same size, but only do this if prev_gray exists
    if prev_gray is not None and gray.shape != prev_gray.shape:
        # Resize gray to match prev_gray's shape
        gray = cv2.resize(gray, (prev_gray.shape[1], prev_gray.shape[0]))

    # Check that both gray and prev_gray are single-channel before optical flow computation
    if prev_gray is not None and gray.ndim == 2 and prev_gray.ndim == 2:
        # Compute dense optical flow (Farneback)
        flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None,
                                            0.5, 3, 5, 3, 5, 1.2, 0)

        # Remove rotational flow using gyro data
        rot_flow = compute_rotational_flow(gray.shape, gyro_x, gyro_y, gyro_z, dt, cx, cy)
        trans_flow = flow - rot_flow

        # Compute depth from translational flow (using known velocity)
        depth = compute_depth(trans_flow, vel_x, vel_y, vel_z)

       # Visualize and overlay results
        speed_color = cv2.applyColorMap(cv2.normalize(cv2.cartToPolar(trans_flow[..., 0], trans_flow[..., 1])[0], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8), cv2.COLORMAP_INFERNO)
        result_image = cv2.addWeighted(image, 0.6, speed_color, 0.4, 0)
        
        # Get the depth of the center pixel (or any other pixel of interest)
        center_depth = depth[depth.shape[0] // 2, depth.shape[1] // 2]
        
        # Ensure vel_x, vel_y, vel_z, and center_depth are scalar values
        vel_x = vel_x.item() if isinstance(vel_x, np.ndarray) else vel_x
        vel_y = vel_y.item() if isinstance(vel_y, np.ndarray) else vel_y
        vel_z = vel_z.item() if isinstance(vel_z, np.ndarray) else vel_z
        center_depth = center_depth.item() if isinstance(center_depth, np.ndarray) else center_depth
        
        # Now we can safely format these scalar values
        position_text = f"X: {vel_x:.2f}, Y: {vel_y:.2f}, Z: {vel_z:.2f}, Depth: {center_depth:.2f}"
        cv2.putText(result_image, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        # Show result
        cv2.imshow('Speed-based Optical Flow with IMU Data', speed_color)
        
        # Wait and allow quitting
        if cv2.waitKey(34) & 0xFF == ord('q'):
            break



    else:
        print(f"Skipping image {image_file} due to incorrect number of channels or lack of previous frame.")

    # Update previous frame and timestamp after processing
    prev_gray = gray
    prev_timestamp = image_timestamp

cv2.destroyAllWindows()
