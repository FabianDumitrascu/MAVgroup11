#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

// Compute IoU (Intersection over Union)
float computeIoU(Rect box1, Rect box2) {
    int xA = max(box1.x, box2.x);
    int yA = max(box1.y, box2.y);
    int xB = min(box1.x + box1.width, box2.x + box2.width);
    int yB = min(box1.y + box1.height, box2.y + box2.height);

    int intersectionArea = max(0, xB - xA) * max(0, yB - yA);
    int box1Area = box1.width * box1.height;
    int box2Area = box2.width * box2.height;

    return (float)intersectionArea / (box1Area + box2Area - intersectionArea);
}

// Kalman Filter Object Tracker
struct ObjectTracker {
    KalmanFilter kf;
    Rect predictedBox;
    bool detected = false;
    int missedFrames = 0;
    static constexpr int maxMissedFrames = 5;

    ObjectTracker() {}

    ObjectTracker(Rect initialBox) {
        kf = KalmanFilter(6, 4, 0);

        kf.transitionMatrix = (Mat_<float>(6, 6) << 
            1, 0, 0, 0, 1, 0,
            0, 1, 0, 0, 0, 1,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);

        kf.measurementMatrix = (Mat_<float>(4, 6) << 
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0);

        setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
        setIdentity(kf.measurementNoiseCov, Scalar::all(1e-2));
        setIdentity(kf.errorCovPost, Scalar::all(0.1));

        kf.statePost.at<float>(0) = initialBox.x;
        kf.statePost.at<float>(1) = initialBox.y;
        kf.statePost.at<float>(2) = initialBox.width;
        kf.statePost.at<float>(3) = initialBox.height;
        kf.statePost.at<float>(4) = 0;
        kf.statePost.at<float>(5) = 0;

        predictedBox = initialBox;
    }

    void predict() {
        Mat prediction = kf.predict();
        predictedBox.x = prediction.at<float>(0);
        predictedBox.y = prediction.at<float>(1);
        predictedBox.width = prediction.at<float>(2);
        predictedBox.height = prediction.at<float>(3);
    }

    void update(Rect detectedBox) {
        Mat measurement = (Mat_<float>(4, 1) << detectedBox.x, detectedBox.y, detectedBox.width, detectedBox.height);
        kf.correct(measurement);
        detected = true;
        missedFrames = 0;
    }
};

// Detect green floor
Mat detectFloorMask(Mat& frame) {
    Mat hsv, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // Stricter green threshold
    Scalar lower_green(35, 70, 70); 
    Scalar upper_green(85, 255, 255);

    inRange(hsv, lower_green, upper_green, mask);
    return mask;
}

// Check if object touches the green floor
bool touchesFloor(const Rect& box, const Mat& floorMask) {
    for (int y = box.y; y < box.y + box.height; y++) {
        for (int x = box.x; x < box.x + box.width; x++) {
            if (floorMask.at<uchar>(y, x) > 0) {
                return true;
            }
        }
    }
    return false;
}

// Check if floor is barely visible (meaning object is close)
bool floorIsSmall(const Mat& floorMask) {
    int nonZeroPixels = countNonZero(floorMask);
    return nonZeroPixels < 5000;  // Adjust this threshold based on testing
}

int main() {
    VideoCapture cap("rtp://127.0.0.1:5000");
    if (!cap.isOpened()) {
        cerr << "Error: Could not open RTP stream!" << endl;
        return -1;
    }

    Mat frame;
    vector<ObjectTracker> trackers;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "Error: Empty frame!" << endl;
            break;
        }

        // Detect floor
        Mat floorMask = detectFloorMask(frame);
        cv::rotate(floorMask, floorMask, cv::ROTATE_90_COUNTERCLOCKWISE);
        imshow("Floor Mask", floorMask);

        bool closeObjectDetected = floorIsSmall(floorMask);

        // Convert to grayscale
        Mat grayImage;
        cvtColor(frame, grayImage, COLOR_BGR2GRAY);

        // Apply Gaussian blur
        Mat blurredImage;
        GaussianBlur(grayImage, blurredImage, Size(5, 5), 1.4);

        // Apply Canny edge detection
        Mat edges;
        Canny(blurredImage, edges, 90, 150);

        // Find contours
        vector<vector<Point>> contours;
        findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Filter objects by size and width
        double min_area_threshold = 500.0;
        double max_area_threshold = 5000.0;
        vector<Rect> detectedBoxes;

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            Rect box = boundingRect(contour);

            if (area > min_area_threshold && area < max_area_threshold) {
                // Ensure objects aren't too wide (aspect ratio limit)
                //if (box.width < box.height * 0.5) {  
                    detectedBoxes.push_back(box);
                //}
            }
        }

        // Predict positions of existing trackers
        for (auto& tracker : trackers) {
            tracker.predict();
            tracker.detected = false;
            tracker.missedFrames++;
        }

        // Match detections to trackers
        vector<bool> matched(detectedBoxes.size(), false);
        for (auto& tracker : trackers) {
            float bestIoU = 0.0;
            int bestMatchIdx = -1;

            for (size_t i = 0; i < detectedBoxes.size(); i++) {
                float iou = computeIoU(tracker.predictedBox, detectedBoxes[i]);
                if (iou > bestIoU && iou > 0.3) { 
                    bestIoU = iou;
                    bestMatchIdx = i;
                }
            }

            if (bestMatchIdx != -1) {
                tracker.update(detectedBoxes[bestMatchIdx]);
                matched[bestMatchIdx] = true;
            }
        }

        // Add new trackers for unmatched detections
        for (size_t i = 0; i < detectedBoxes.size(); i++) {
            if (!matched[i] && (touchesFloor(detectedBoxes[i], floorMask) || closeObjectDetected)) {
                trackers.emplace_back(detectedBoxes[i]);
            }
        }

        // Remove trackers that have been missing for too long
        trackers.erase(remove_if(trackers.begin(), trackers.end(),
            [](ObjectTracker& t) { return t.missedFrames > ObjectTracker::maxMissedFrames; }), 
            trackers.end());

        // Draw bounding boxes
        for (auto& tracker : trackers) {
            Scalar color = tracker.detected ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            rectangle(frame, tracker.predictedBox, color, 2);
        }

	cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);
	cv::rotate(edges, edges, cv::ROTATE_90_COUNTERCLOCKWISE);


        imshow("Original Stream", frame);
        imshow("Edges", edges);


        if (waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

