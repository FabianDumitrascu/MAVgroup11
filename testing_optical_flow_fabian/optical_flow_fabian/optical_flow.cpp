#include <iostream>
#include <vector>
#include <string>
#include <algorithm>        // for std::sort
#include <filesystem>       // C++17 filesystem
#include <opencv2/opencv.hpp>

// Helper function to check if a path has a .jpg or .JPG extension
bool endsWithJpg(const std::string &filename) {
    if (filename.size() < 4) return false;
    std::string ext = filename.substr(filename.size() - 4);
    // Make extension lowercase
    for (auto &c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return (ext == ".jpg");
}

int main(int argc, char* argv[])
{
    // By default, we look in "../data/sim_flight_poles_dataset"
    // You can override this by passing a different path as the first argument
    std::string folderPath = "../data/sim_flight_poles_dataset";
    if (argc > 1) {
        folderPath = argv[1];
    }

    // Vector to hold paths to all .jpg files
    std::vector<std::string> imageFiles;

    // Use C++17 filesystem to iterate through the directory
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

    // Check if we found any .jpg files
    if (imageFiles.empty()) {
        std::cerr << "No .jpg files found in: " << folderPath << std::endl;
        return 0;
    }

    // Sort filenames alphabetically
    std::sort(imageFiles.begin(), imageFiles.end());

    // Display images in pairs
    // i.e. (0,1), (2,3), (4,5), ...
    for (size_t i = 0; i < imageFiles.size() - 1; i += 2) {
        // Load images in color
        cv::Mat img1 = cv::imread(imageFiles[i], cv::IMREAD_COLOR);
        cv::Mat img2 = cv::imread(imageFiles[i + 1], cv::IMREAD_COLOR);

        // Check if loading succeeded
        if (img1.empty() || img2.empty()) {
            std::cerr << "Error: Could not load image: " 
                      << imageFiles[i] << " or " << imageFiles[i + 1] << std::endl;
            continue;
        }

        // Rotate images 90 degrees counterclockwise (to the left)
        cv::Mat rotatedImg1, rotatedImg2;
        cv::rotate(img1, rotatedImg1, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(img2, rotatedImg2, cv::ROTATE_90_COUNTERCLOCKWISE);

        // Create windows for the images
        const std::string winname1 = "Image1";
        const std::string winname2 = "Image2";

        cv::namedWindow(winname1, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(winname2, cv::WINDOW_AUTOSIZE);

        // Move windows so that they are positioned next to each other.
        // Adjust the x and y coordinates as needed for your screen.
        cv::moveWindow(winname1, 100, 600); // e.g., position (100, 100)
        cv::moveWindow(winname2, 600, 100); // e.g., position (600, 100)

        // Display the rotated images
        cv::imshow(winname1, rotatedImg1);
        cv::imshow(winname2, rotatedImg2);

        std::cout << "Displaying:\n  " << imageFiles[i]
                  << "\n  " << imageFiles[i + 1]
                  << "\nPress any key to continue...\n";
        cv::waitKey(0);

        // Destroy windows to clean up
        cv::destroyWindow(winname1);
        cv::destroyWindow(winname2);
    }

    return 0;
}
