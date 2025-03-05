import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

# Set up the camera intrinsic parameters
# Replace these values with your actual camera calibration data
focal_length = 1000  # Example value, in pixels
cx, cy = 640, 480   # Example values for the optical center (principal point)
K = np.array([[focal_length, 0, cx],
              [0, focal_length, cy],
              [0, 0, 1]])  # Camera intrinsic matrix

# Load images (adjust path to your set of images)
image_folder = '/home/ntermote/Documents/TUD_Robotics/AE4317_Autonomous_flight_MAVs/frontcam_flight'
image_files = sorted([f for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png'))])

# Feature detection using ORB (you can use SIFT or SURF if available)
orb = cv2.ORB_create()
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# To store 3D points and camera poses
points_3D = []
camera_poses = []

# Initialize variables for previous image keypoints and descriptors
prev_keypoints = None
prev_descriptors = None
prev_image = None
prev_pose = np.eye(4)  # Initial pose (identity matrix)

# Function for triangulation of 3D points from two views
def triangulate_points(K, R, t, pts1, pts2):
    # Ensure that pts1 and pts2 are valid (same number of points)
    if len(pts1) != len(pts2):
        raise ValueError("Number of points in pts1 and pts2 must match")
    
    # Ensure that pts1 and pts2 are in homogeneous coordinates
    pts1_hom = cv2.convertPointsToHomogeneous(pts1)
    pts2_hom = cv2.convertPointsToHomogeneous(pts2)

    # Create projection matrices for both views
    P1 = np.dot(K, np.hstack((np.eye(3), np.zeros((3, 1)))))  # Camera 1 (identity matrix for the first camera)
    P2 = np.dot(K, np.hstack((R, t)))  # Camera 2 (with rotation and translation)

    # Triangulate points
    points_4D = cv2.triangulatePoints(P1, P2, pts1_hom, pts2_hom)

    # Convert from homogeneous coordinates to 3D
    points_3D = points_4D[:3, :] / points_4D[3, :]
    return points_3D.T  # Return points as Nx3 array

# Main loop to process images
for i, image_file in enumerate(image_files):
    image = cv2.imread(os.path.join(image_folder, image_file))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect keypoints and descriptors
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    if prev_image is None:
        prev_image = image
        prev_keypoints = keypoints
        prev_descriptors = descriptors
        continue

    # Match descriptors between current and previous image
    matches = bf.match(prev_descriptors, descriptors)

    # Extract matched points from both images
    pts1 = np.float32([prev_keypoints[m.queryIdx].pt for m in matches])
    pts2 = np.float32([keypoints[m.trainIdx].pt for m in matches])

    if len(pts1) < 8:  # Minimum number of points required for Essential Matrix computation
        print("Not enough matches for essential matrix computation.")
        continue

    # Estimate the essential matrix
    E, mask = cv2.findEssentialMat(pts1, pts2, K)

    # Recover the relative camera pose (R and t)
    _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K)

    # Apply the mask to only use good points
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]

    # Now we can triangulate the points from both views
    try:
        points_3D_current = triangulate_points(K, R, t, pts1, pts2)
        # Append the 3D points and camera pose
        points_3D.append(points_3D_current)
        camera_poses.append(np.hstack((R, t)))
    except ValueError as e:
        print(f"Error in triangulation: {e}")
        continue

    # Update the previous image data for the next iteration
    prev_image = image
    prev_keypoints = keypoints
    prev_descriptors = descriptors
    prev_pose = np.hstack((R, t))  # Updating previous pose

    # Visualize the matches (optional)
    img_matches = cv2.drawMatches(prev_image, prev_keypoints, image, keypoints, matches, None)
    cv2.imshow("Matches", img_matches)
    cv2.waitKey(1)

cv2.destroyAllWindows()

# At this point, we have all the 3D points and the camera poses
# You can save these or visualize the results

# Stack all 3D points
all_points_3D = np.vstack(points_3D)

# Convert 3D points to plot them
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(all_points_3D[:, 0], all_points_3D[:, 1], all_points_3D[:, 2], s=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
