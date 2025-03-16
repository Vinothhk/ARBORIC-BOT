#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    // Create/Open a VideoCapture object to access the camera
    cv::VideoCapture cap(2); // 0 is the default webcam

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the camera!" << std::endl;
        return -1;
    }

    // Ensure the "images" folder exists
    std::string folder = "../images";
    if (!fs::exists(folder)) {
        fs::create_directory(folder);
    }

    cv::Mat frame;
    int imageCount = 1;

    std::cout << "Press 's' to save an image, 'q' to quit." << std::endl;

    while (true) {
        cap >> frame; // Capture a new frame

        if (frame.empty()) {
            std::cerr << "Error: Captured empty frame!" << std::endl;
            break;
        }

        cv::imshow("Live Camera Feed", frame);

        char key = cv::waitKey(1);
        if (key == 's') {  // Save the image when 's' is pressed
            std::string filename = folder + "/image_" + std::to_string(imageCount) + ".jpg";
            cv::imwrite(filename, frame);
            std::cout << "Image saved: " << filename << std::endl;
            imageCount++;
        } else if (key == 'q') {  // Quit when 'q' is pressed
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
