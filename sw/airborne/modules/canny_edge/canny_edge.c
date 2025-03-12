#include "opencv2/opencv.hpp"
#include "stdio.h"
#include "std.h"
#include "subsystems/datalink/telemetry.h"

using namespace cv;

// Function to process the image using Canny edge detection
void canny_edge_detect(uint8_t *image, int width, int height) {
    Mat frame(height, width, CV_8UC3, image); // Convert UAV image buffer to OpenCV format

    // Convert to grayscale
    Mat grayImage;
    cvtColor(frame, grayImage, COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    Mat blurredImage;
    GaussianBlur(grayImage, blurredImage, Size(5, 5), 1.4);

    // Apply Canny edge detection
    Mat edges;
    Canny(blurredImage, edges, 100, 150);

    // Send processed data via telemetry (Example: send the number of detected edges)
    int edge_count = countNonZero(edges);
    DOWNLINK_SEND_EDGE_DATA(DefaultChannel, &edge_count);

    printf("[Canny Edge] Processed frame - Detected Edges: %d\n", edge_count);
}

// Initialization function
void canny_edge_init(void) {
    printf("[Canny Edge] Module Initialized\n");
}

