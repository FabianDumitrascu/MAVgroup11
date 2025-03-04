#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>

// Helper function: returns true if filename ends with ".jpg" (case-insensitive)
bool endsWithJpg(const std::string &filename) {
    if (filename.size() < 4) return false;
    std::string ext = filename.substr(filename.size() - 4);
    for (auto &c : ext)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return (ext == ".jpg");
}

int main(int argc, char* argv[])
{
    // Set the default folder path; can be overridden by command-line argument.
    std::string folderPath = "../data/sim_flight_poles_dataset";
    if (argc > 1) {
        folderPath = argv[1];
    }

    // Collect all .jpg files from the directory.
    std::vector<std::string> imageFiles;
    try {
        for (const auto &entry : std::filesystem::directory_iterator(folderPath)) {
            if (entry.is_regular_file()) {
                std::string path = entry.path().string();
                if (endsWithJpg(path)) {
                    imageFiles.push_back(path);
                }
            }
        }
    }
    catch (const std::filesystem::filesystem_error &e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return 1;
    }
    if (imageFiles.empty()) {
        std::cerr << "No .jpg files found in: " << folderPath << std::endl;
        return 0;
    }

    // Sort filenames alphabetically.
    std::sort(imageFiles.begin(), imageFiles.end());

    // Define an upscaling factor.
    double scaleFactor = 2.0;  // Change this value to upscale more or less.

    // Process images in pairs.
    for (size_t i = 0; i < imageFiles.size() - 1; i += 2) {
        // Load the images in color.
        cv::Mat img1 = cv::imread(imageFiles[i], cv::IMREAD_COLOR);
        cv::Mat img2 = cv::imread(imageFiles[i + 1], cv::IMREAD_COLOR);
        if (img1.empty() || img2.empty()) {
            std::cerr << "Error: Could not load image: " 
                      << imageFiles[i] << " or " << imageFiles[i + 1] << std::endl;
            continue;
        }

        // Rotate images 90° counterclockwise.
        cv::Mat rotatedImg1, rotatedImg2;
        cv::rotate(img1, rotatedImg1, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(img2, rotatedImg2, cv::ROTATE_90_COUNTERCLOCKWISE);

        // Upscale the rotated images.
        cv::Mat upscaledImg1, upscaledImg2;
        cv::resize(rotatedImg1, upscaledImg1, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LINEAR);
        cv::resize(rotatedImg2, upscaledImg2, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LINEAR);

        // Convert upscaled images to grayscale (required for optical flow).
        cv::Mat gray1, gray2;
        cv::cvtColor(upscaledImg1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(upscaledImg2, gray2, cv::COLOR_BGR2GRAY);

        // Compute optical flow between the two grayscale images using Farneback's algorithm.
        cv::Mat flow;  // This will hold the 2-channel flow vectors (dx, dy)
        cv::calcOpticalFlowFarneback(
            gray1, gray2, flow,
            0.5,   // pyramid scale (<1)
            3,     // number of pyramid layers
            15,    // window size
            3,     // number of iterations per pyramid level
            5,     // size of the pixel neighborhood used to find polynomial expansion (poly_n)
            1.2,   // standard deviation for Gaussian smoothing (poly_sigma)
            0      // flags
        );

        // Create an optical flow visualization.
        cv::Mat flowVisualization;
        cv::cvtColor(gray1, flowVisualization, cv::COLOR_GRAY2BGR);
        const int step = 16;  // Sampling step for drawing flow vectors.
        for (int y = 0; y < flowVisualization.rows; y += step) {
            for (int x = 0; x < flowVisualization.cols; x += step) {
                const cv::Point2f &fxy = flow.at<cv::Point2f>(y, x);
                cv::line(flowVisualization, 
                         cv::Point(x, y), 
                         cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
                         cv::Scalar(0, 255, 0), 1);
                cv::circle(flowVisualization, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
            }
        }

        // Segment motion by thresholding the optical flow magnitude.
        cv::Mat flowChannels[2];
        cv::split(flow, flowChannels);
        cv::Mat magnitude, angle;
        cv::cartToPolar(flowChannels[0], flowChannels[1], magnitude, angle, true);  // angle in degrees

        // Threshold the magnitude to segment areas with significant motion.
        double threshValue = 2.0;  // Tune this threshold based on your data.
        cv::Mat segmented;
        cv::threshold(magnitude, segmented, threshValue, 255, cv::THRESH_BINARY);
        segmented.convertTo(segmented, CV_8U);

        // Create display windows.
        const std::string win1 = "Upscaled Image 1";
        const std::string win2 = "Upscaled Image 2";
        const std::string winFlow = "Optical Flow";
        const std::string winSegmented = "Segmented Motion";
        cv::namedWindow(win1, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(win2, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(winFlow, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(winSegmented, cv::WINDOW_AUTOSIZE);

        // Position the windows (adjust coordinates as needed).
        cv::moveWindow(win1, 50, 50);
        cv::moveWindow(win2, upscaledImg1.cols + 100, 50);
        cv::moveWindow(winFlow, 50, upscaledImg1.rows + 100);
        cv::moveWindow(winSegmented, upscaledImg1.cols + 100, upscaledImg1.rows + 100);

        // Display the images.
        cv::imshow(win1, upscaledImg1);
        cv::imshow(win2, upscaledImg2);
        cv::imshow(winFlow, flowVisualization);
        cv::imshow(winSegmented, segmented);

        std::cout << "Displaying:\n  " << imageFiles[i]
                  << "\n  " << imageFiles[i + 1]
                  << "\nPress any key to move on to the next pair..." << std::endl;
        cv::waitKey(0);  // Wait for a key press.

        // Destroy all windows before processing the next pair.
        cv::destroyAllWindows();
    }

    return 0;
}
