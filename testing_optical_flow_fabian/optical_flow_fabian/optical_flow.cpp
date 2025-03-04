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
    // Sort the filenames alphabetically.
    std::sort(imageFiles.begin(), imageFiles.end());

    // Define a scaling factor for upscaling the images.
    double scaleFactor = 2.0; // Change this to get a bigger or smaller display

    // Process images in pairs.
    for (size_t i = 0; i < imageFiles.size() - 1; i += 2) {
        // Load images in color.
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

        // Upscale (resize) the rotated images.
        cv::Mat upscaledImg1, upscaledImg2;
        cv::resize(rotatedImg1, upscaledImg1, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LINEAR);
        cv::resize(rotatedImg2, upscaledImg2, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LINEAR);

        // For optical flow, convert the upscaled images to grayscale.
        cv::Mat gray1, gray2;
        cv::cvtColor(upscaledImg1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(upscaledImg2, gray2, cv::COLOR_BGR2GRAY);

        // Compute optical flow between the two grayscale images using Farneback's algorithm.
        cv::Mat flow;  // This will hold the 2-channel flow vectors (dx, dy)
        cv::calcOpticalFlowFarneback(
            gray1, gray2, flow,
            0.2,              // pyramid scale (<1)
            3,                // number of pyramid layers
            30,               // window size
            3,                // number of iterations at each pyramid level
            5,                // size of the pixel neighborhood used to find polynomial expansion
            1.2,              // standard deviation of the Gaussian used to smooth derivatives
            0                 // flags
        );

        // Create a visualization of the optical flow.
        cv::Mat flowVisualization;
        cv::cvtColor(gray1, flowVisualization, cv::COLOR_GRAY2BGR);
        const int step = 16;  // Sampling step for drawing flow vectors.
        for (int y = 0; y < flowVisualization.rows; y += step) {
            for (int x = 0; x < flowVisualization.cols; x += step) {
                const cv::Point2f &fxy = flow.at<cv::Point2f>(y, x);
                // Draw a line representing the flow vector.
                cv::line(flowVisualization, 
                         cv::Point(x, y), 
                         cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
                         cv::Scalar(0, 255, 0), 1);
                // Draw a small circle at the origin of the flow vector.
                cv::circle(flowVisualization, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
            }
        }

        // Create three windows: one for each upscaled image and one for the optical flow.
        const std::string win1 = "Image 1";
        const std::string win2 = "Image 2";
        const std::string winFlow = "Optical Flow";
        cv::namedWindow(win1, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(win2, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(winFlow, cv::WINDOW_AUTOSIZE);

        // Move the windows to avoid overlap (adjust coordinates as needed for your screen).
        cv::moveWindow(win1, 50, 50);
        cv::moveWindow(win2, upscaledImg1.cols + 100, 50);
        cv::moveWindow(winFlow, 50, upscaledImg1.rows + 100);

        // Display the images.
        cv::imshow(win1, upscaledImg1);
        cv::imshow(win2, upscaledImg2);
        cv::imshow(winFlow, flowVisualization);

        std::cout << "Displaying:\n  " << imageFiles[i]
                  << "\n  " << imageFiles[i + 1]
                  << "\nPress any key to continue...\n";
        cv::waitKey(0);

        // Clean up the windows before processing the next pair.
        cv::destroyWindow(win1);
        cv::destroyWindow(win2);
        cv::destroyWindow(winFlow);
    }

    return 0;
}
