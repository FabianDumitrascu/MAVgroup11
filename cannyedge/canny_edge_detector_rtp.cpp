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

// Apply Non-Maximum Suppression (NMS)
vector<Rect> applyNMS(vector<Rect>& boxes, float iouThreshold = 0.5) {
    if (boxes.empty()) return {};

    // Sort boxes by area (or confidence if available)
    sort(boxes.begin(), boxes.end(), [](const Rect& a, const Rect& b) {
        return (a.width * a.height) > (b.width * b.height); // Larger area first
    });

    vector<bool> suppressed(boxes.size(), false);
    vector<Rect> finalBoxes;

    for (size_t i = 0; i < boxes.size(); i++) {
        if (suppressed[i]) continue;

        finalBoxes.push_back(boxes[i]);

        for (size_t j = i + 1; j < boxes.size(); j++) {
            if (computeIoU(boxes[i], boxes[j]) > iouThreshold) {
                suppressed[j] = true; // Suppress overlapping box
            }
        }
    }

    return finalBoxes;
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

        setIdentity(kf.processNoiseCov, Scalar::all(5e-3));
        setIdentity(kf.measurementNoiseCov, Scalar::all(2e-2));
        setIdentity(kf.errorCovPost, Scalar::all(0.0));

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
	// Crop the frame to remove a quarter from the top and bottom
	int width = frame.cols;
	int height = frame.rows;

	int newHeight = height / 2;   // Keep only the central 50% (remove 25% from top and bottom)
	int startY = height / 4;      // Start cropping from 1/4th of the original height

	frame = frame(Rect(0, startY, width, newHeight)).clone();

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
        GaussianBlur(grayImage, blurredImage, Size(11, 11), 1.9);

        // Apply Canny edge detection
        Mat edges;
        Canny(blurredImage, edges, 50, 255);
        
        vector<Vec4i> lines;
	HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

	// Draw detected lines on the image
	for (size_t i = 0; i < lines.size(); i++) {
	    Vec4i l = lines[i];
	    line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 4);
	}


     	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

	// Apply dilation first to connect edges
	dilate(edges, edges, kernel, Point(-1, -1), 2);

	// Apply erosion to remove small noise and refine shapes
	erode(edges, edges, kernel, Point(-1, -1), 1);

        int borderSize = 2;  // Adjust based on need
	copyMakeBorder(edges, edges, 0,0, borderSize, borderSize, BORDER_CONSTANT, Scalar(255)); 

	vector<vector<Point>> contours;
	findContours(edges, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        // Filter objects by size and width
        double min_area_threshold = 2500.0;
        double max_area_threshold = 20000.0;
        vector<Rect> detectedBoxes;

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            Rect box = boundingRect(contour);
            //double ratio = box.width/box.height;
            double aspectRatio = (double)box.width / box.height;
            if (aspectRatio < 0.1 || aspectRatio > 5.0) {  // Ignore very thin or wide objects
	    continue;
	    }

            if (area > min_area_threshold && area < max_area_threshold) {
            	//if (ratio < 1) {
                    detectedBoxes.push_back(box);
                    //}
            }
        }
        
	// Apply Non-Maximum Suppression (NMS) to remove redundant boxes
	detectedBoxes = applyNMS(detectedBoxes, 0.5);

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

            if (bestMatchIdx != -1 && 
		    (detectedBoxes[bestMatchIdx].width * detectedBoxes[bestMatchIdx].height) >= min_area_threshold &&
		    (detectedBoxes[bestMatchIdx].width * detectedBoxes[bestMatchIdx].height) <= max_area_threshold) {
		    
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
        int area = tracker.predictedBox.width * tracker.predictedBox.height;
	std::cout << tracker.predictedBox.width * tracker.predictedBox.height << "\n";
	    	if (area >= min_area_threshold && area <= max_area_threshold) {
		    Scalar color = tracker.detected ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
		    rectangle(frame, tracker.predictedBox, color, 2);

		}
	}

	cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);
	cv::rotate(edges, edges, cv::ROTATE_90_COUNTERCLOCKWISE);
	cv::rotate(blurredImage, blurredImage, cv::ROTATE_90_COUNTERCLOCKWISE);

        imshow("Edges", edges);
        imshow("blurred",blurredImage);
        imshow("Original Stream", frame);


        if (waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
