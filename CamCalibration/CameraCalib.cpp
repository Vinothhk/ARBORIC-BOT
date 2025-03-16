#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
  
// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9}; 

void saveCameraCalibration(const std::string& filepath, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
  cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
  
  if (!fs.isOpened()) {
      std::cerr << "Error: Could not open file " << filepath << " for writing!" << std::endl;
      return;
  }

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs.release();
  std::cout << "Calibration data saved to: " << filepath << std::endl;
}


int main()
{
  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;
  
  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpoints;
  
  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }
  
  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;
  // Path of the folder containing checkerboard images
  std::string path = "./../images/*.jpg";
  std::cout << "Path : " << path << std::endl;
  cv::glob(path, images);
  int success_images = 0;
  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checker board corners 
  std::vector<cv::Point2f> corner_pts;
  bool success;
  int total_imgs = images.size();

// Looping over all the images in the directory
for(int i{0}; i<images.size(); i++)
{
  frame = cv::imread(images[i]);
  cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

  // Finding checker board corners
  // If desired number of corners are found in the image then success = true  
  success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
    
  /* 
    * If desired number of corner are detected,
    * we refine the pixel coordinates and display 
    * them on the images of checker board
  */
  if(success)
  {
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);
      
    // refining pixel coordinates for given 2d points.
    cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
      
    // Displaying the detected corner points on the checker board
    cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
      
    objpoints.push_back(objp);
    imgpoints.push_back(corner_pts);
    std::cout << "Success" << std::endl;
    success_images += 1;
  }
  else{
    std::cout << "Failed" << std::endl;
  }

  cv::imshow("Image",frame);
  cv::waitKey(0);
}

// std::cout<<"Chessboard corners are successfully detected"<<std::endl;
std::cout<<success_images<<" images are Successfully Calibrated out of "<<total_imgs<<std::endl;
cv::destroyAllWindows();

cv::Mat cameraMatrix,distCoeffs,R,T;

/*
  * Performing camera calibration by 
  * passing the value of known 3D points (objpoints)
  * and corresponding pixel coordinates of the 
  * detected corners (imgpoints)
*/
cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
std::cout << "distCoeffs : " << distCoeffs << std::endl;
std::cout << "Rotation vector : " << R << std::endl;
std::cout << "Translation vector : " << T << std::endl;

std::string save_path = "./../CamCalibration/camera_parameters.yaml";
saveCameraCalibration(save_path, cameraMatrix, distCoeffs);

return 0;
}