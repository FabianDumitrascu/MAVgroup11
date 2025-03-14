import cv2
import os
import numpy as np

# Set the folder path where your images are stored
folder_path = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/frontcam_flight'  # Replace with actual path

# Get all image files in the folder and sort them
image_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.jpg') or f.endswith('.png')])

# Define the video codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Using XVID codec
output_video = cv2.VideoWriter('output_video.avi', fourcc, 30.0, (640, 480))  # 30 fps, 640x480 resolution

# Calibration matrix camera
mtx = np.array([[323.58405676, 0, 262.96788762],
                [0, 323.93936434, 214.11123418],
                [0, 0, 1]])

dist = np.array([[-3.38129283e-01, 1.34930511e-01, -1.58307236e-04, 3.84031314e-04, -2.78102296e-02]])

# Loop through each image in the folder
for image_file in image_files:
    # Load the image
    image = cv2.imread(os.path.join(folder_path, image_file))
    
    # Rotate and undistort the image
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image = cv2.undistort(image, mtx, dist)
    
    # Image is already in YUV format, no need for conversion
    yuv = image
    
    # Define green color range in YUV space
    lower_green = np.array([60, 75, 75], dtype=np.uint8)
    upper_green = np.array([70, 91, 91], dtype=np.uint8)  # Adjust based on actual green range
    
    # Create mask for green color
    mask = cv2.inRange(yuv, lower_green, upper_green)
    
    # Highlight green pixels in the original image (change to red for visualization)
    image[mask > 0] = [0, 0, 255]  # Set detected green pixels to red
    
    # Display the processed image
    cv2.imshow('Highlighted Green', image)
    
    # Write frame to the output video
    output_video.write(image)
    
    # Wait for a short period (adjust delay for fps control)
    key = cv2.waitKey(100)
    if key == ord('q'):
        break

# Release resources
output_video.release()
cv2.destroyAllWindows()
