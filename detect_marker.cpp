#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

bool loadCameraCalibration(const std::string& filepath, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file " << filepath << " for reading!" << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    
    fs.release();
    std::cout << "Calibration data loaded successfully from: " << filepath << std::endl;
    return true;
}

// using namespace cv;
int main() {

    std::string load_path = "./../CamCalibration/camera_parameters.yaml"
    // Load camera calibration parameters
    cv::Mat cameraMatrix, distCoeffs;
    if (loadCameraCalibration(load_path, cameraMatrix, distCoeffs)) {
        std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
        std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
    } else {
        std::cerr << "Failed to load calibration data!" << std::endl;
    }

    // cv::FileStorage fs("calibration.yml", cv::FileStorage::READ);
    // if (!fs.isOpened()) {
    //     std::cerr << "Failed to open calibration file!" << std::endl;
    //     return -1;
    // }
    // fs["camera_matrix"] >> cameraMatrix;
    // fs["distortion_coefficients"] >> distCoeffs;
    // fs.release();

    // Create ArUco dictionary and parameters
    cv::aruco::ArucoDetector detector;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    // Open video capture
    cv::VideoCapture cap(2); // Open the default camera
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream or file!" << std::endl;
        return -1;
    }

    // Define marker size (in meters or any unit)
    float markerLength = 0.05; // 5 cm

    while (true) {
        cv::Mat frame;
        cap >> frame; // Capture a new frame
        if (frame.empty()) break;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<cv::Vec3d> rvecs, tvecs;

        // Detect ArUco markers
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // If markers are detected
        if (!markerIds.empty()) {
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // Estimate pose of each marker
            cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (size_t i = 0; i < markerIds.size(); ++i) {
                // Draw marker axes
                cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5);

                // Display marker ID and position
                std::cout << "Marker ID: " << markerIds[i]
                          << " Translation: " << tvecs[i]
                          << " Rotation: " << rvecs[i] << std::endl;
            }
        }

        // Display the frame
        cv::imshow("ArUco Detection", frame);

        // Break on 'q' key press
        if (cv::waitKey(10) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
