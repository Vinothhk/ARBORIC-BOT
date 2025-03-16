#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <iostream>
#include "opencv2/calib3d.hpp"
#include <opencv2/aruco.hpp>
// #include "aruco_samples_utility.hpp"
using namespace cv;

// int main() {
//     // User specifies the image path
//     string imagePath;
//     // cout << "/home/vinoth/cpp_codes/build/marker23.png";
//     // cin >> imagePath;
//     imagePath = "/home/vinoth/cpp_codes/build/marker23.png";
//     // Read and validate the image
//     Mat image = imread(imagePath);
//     if (image.empty()) {
//         cerr << "Error: Could not open the image at " << imagePath << endl;
//         return -1;
//     }

//     // Initialize ArUco detector parameters and dictionary
//     cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
//     cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//     cv::aruco::ArucoDetector detector(dictionary, detectorParams);

//     // Detect markers
//     vector<int> ids;
//     vector<vector<Point2f>> corners;
//     detector.detectMarkers(image, corners, ids);
//     Mat outputImage = image.clone();
//     cout<<"ID size: "<<ids.size()<<endl;
//     if (ids.size()) {
//         cout<<"Section4"<<endl;
//         cv::aruco::drawDetectedMarkers(outputImage, corners, ids);
//         cout << "Detected marker IDs: ";
//         for (int id : ids) {
//             cout << id << " ";
//         }
//         cout << endl;
//     } else {
//         cout << "No markers detected in the image." << endl;
//     }

//     // Display the output image
//     imshow("Detected Markers", outputImage);
//     waitKey(0); // Wait for a key press before closing the window

//     return 0;
// }

using namespace std;
// using namespace cv;

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


int main() {
    // Open the default camera (camera index 0)
    VideoCapture cap(2);
    if (!cap.isOpened()) {
        cerr << "Error: Could not access the camera." << endl;
        return -1;
    }
    
    // Initialize ArUco detector parameters and dictionary
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    float markerLength = 0.025;
    Mat frame,copyframe;

    // Mat cameraMatrix = (Mat1d(3, 3) <<628.158, 0., 324.099,0., 628.156, 260.908,0., 0., 1.);
    // Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); //(Mat1d(3, 3) <<0.0995485, -0.206384,0.00754589, 0.00336531, 0);
    // cout<<"Cam matrix: "<<cameraMatrix;

    std::string load_path = "./../CamCalibration/camera_parameters.yaml";
    // Load camera calibration parameters
    cv::Mat cameraMatrix, distCoeffs;

    if (loadCameraCalibration(load_path, cameraMatrix, distCoeffs)) {
        std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
        std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
    } else {
        std::cerr << "Failed to load calibration data!" << std::endl;
    }

    while (cap.grab()) {
        // Capture a frame from the camera
        // cap >> frame;
        cap.retrieve(frame);
        if (frame.empty()) {
            cerr << "Error: Captured an empty frame." << endl;
            break;
        }

        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
        objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
        // Detect markers
        vector<int> ids;
        vector<vector<Point2f>> corners,rejected;
        detector.detectMarkers(frame, corners, ids,rejected);

        // Draw detected markers
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            cout << "Detected marker IDs: ";
            for (int id : ids) {
                cout << id << " ";
            }
            cout << endl;
        } else {
            // cout << "No markers detected in the current frame." << endl;
        }

        size_t nMarkers = corners.size();
        vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if(!ids.empty()) {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++) {
                cv::solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }

        for(unsigned int i = 0; i < ids.size(); i++){
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);

        }
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {
            // Display marker ID and position
            std::cout << "Marker ID: " << ids[i]
                      << " Translation: " << tvecs[i]
                      << " Rotation: " << rvecs[i] << std::endl;
        }

        // Display the frame
        imshow("Detected Markers", frame);

        frame.copyTo(copyframe);
        // Exit the loop if the user presses the 'q' key
        if (waitKey(1) == 'q') {
            break;
        }
    }

    // Release the camera and close all OpenCV windows
    cap.release();
    destroyAllWindows();

    return 0;
}

