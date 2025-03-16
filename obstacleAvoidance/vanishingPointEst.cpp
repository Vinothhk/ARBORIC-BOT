#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

int main() {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ObstacleDetection");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);

    const std::string modelPath = "/home/vinoth/ComputerVisionCPP/Workspace/obstacleAvoidance/midas_v21_small_256.onnx";
    Ort::Session midasSession(env, modelPath.c_str(), sessionOptions);
    
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr inputName = midasSession.GetInputNameAllocated(0, allocator);
    Ort::AllocatedStringPtr outputName = midasSession.GetOutputNameAllocated(0, allocator);
    
    std::vector<const char*> inputNames = {inputName.get()};
    std::vector<const char*> outputNames = {outputName.get()};

    std::array<int64_t, 4> inputShape = {1, 3, 256, 256};

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera." << std::endl;
        return -1;
    }

    cv::Mat frame, resizedFrame, inputBlob;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Resize frame for model
        cv::resize(frame, resizedFrame, cv::Size(256, 256));
        resizedFrame.convertTo(resizedFrame, CV_32F, 1.0f / 255.0f);
        cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGR2RGB);

        // Convert to float tensor
        std::vector<float> inputTensorValues(1 * 3 * 256 * 256);
        int index = 0;
        for (int c = 0; c < 3; c++) {
            for (int i = 0; i < 256; i++) {
                for (int j = 0; j < 256; j++) {
                    inputTensorValues[index++] = resizedFrame.at<cv::Vec3f>(i, j)[c];
                }
            }
        }

        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo, inputTensorValues.data(), inputTensorValues.size(),
            inputShape.data(), inputShape.size()
        );

        std::vector<Ort::Value> outputTensors = midasSession.Run(
            Ort::RunOptions{nullptr}, inputNames.data(), &inputTensor, 1, outputNames.data(), 1
        );

        float* outputData = outputTensors[0].GetTensorMutableData<float>();
        cv::Mat depthMap(256, 256, CV_32F, outputData);

        // Normalize depth map
        cv::Mat depthDisplay;
        cv::normalize(depthMap, depthDisplay, 0, 255, cv::NORM_MINMAX);
        depthDisplay.convertTo(depthDisplay, CV_8U);

        // Apply Gaussian Blur to smooth noise
        cv::GaussianBlur(depthDisplay, depthDisplay, cv::Size(5, 5), 0);

        // Compute adaptive threshold based on mean depth
        double minVal, maxVal;
        cv::minMaxLoc(depthDisplay, &minVal, &maxVal);
        double adaptiveThreshold = minVal + (maxVal - minVal) * 0.3;  // Adjusting factor

        cv::Mat binaryMask;
        cv::threshold(depthDisplay, binaryMask, adaptiveThreshold, 255, cv::THRESH_BINARY);

        // Apply Morphological Operations to clean up noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binaryMask, binaryMask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(binaryMask, binaryMask, cv::MORPH_OPEN, kernel);

        // Divide into Left, Center, Right
        int width = binaryMask.cols;
        int height = binaryMask.rows;
        int regionWidth = width / 3;

        cv::Mat leftRegion = binaryMask(cv::Rect(0, 0, regionWidth, height));
        cv::Mat centerRegion = binaryMask(cv::Rect(regionWidth, 0, regionWidth, height));
        cv::Mat rightRegion = binaryMask(cv::Rect(2 * regionWidth, 0, regionWidth, height));

        // Weighted pixel sum for better accuracy
        double leftWeight = cv::sum(leftRegion)[0] * 1.2;  // Slightly higher weight
        double centerWeight = cv::sum(centerRegion)[0];
        double rightWeight = cv::sum(rightRegion)[0] * 1.2; // Slightly higher weight

        double obstacleThreshold = 5000000.0;
        std :: cout << "lv: "<< leftWeight << std::endl << "CV: "<< centerWeight << std::endl <<"rw: " << rightWeight << std::endl;

        std::string obstaclePosition;
        if (leftWeight < obstacleThreshold && centerWeight < obstacleThreshold && rightWeight < obstacleThreshold) {
            obstaclePosition = "Obstacle: None";
        } else if (centerWeight > leftWeight && centerWeight > rightWeight) {
            obstaclePosition = "Obstacle in CENTER!";
        } else if (leftWeight > rightWeight) {
            obstaclePosition = "Obstacle on LEFT!";
        } else {
            obstaclePosition = "Obstacle on RIGHT!";
        }

        // Display result
        cv::putText(frame, obstaclePosition, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

        cv::imshow("Original", frame);
        cv::imshow("Depth Estimation", depthDisplay);
        cv::imshow("Obstacle Detection", binaryMask);

        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}


// #include <iostream>
// #include <vector>
// #include <opencv2/opencv.hpp>
// #include <onnxruntime_cxx_api.h>

// int main() {
//     // Initialize ONNX Runtime environment
//     Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "VanishingPointEstimation");

//     // Create session options
//     Ort::SessionOptions sessionOptions;
//     sessionOptions.SetIntraOpNumThreads(1);

//     // Load the model
//     const std::string modelPath = "/home/vinoth/ComputerVisionCPP/Workspace/obstacleAvoidance/midas_v21_small_256.onnx";
//     Ort::Session midasSession(env, modelPath.c_str(), sessionOptions);
    
//     // Create an allocator
//     Ort::AllocatorWithDefaultOptions allocator;

//     // Get input and output names
//     Ort::AllocatedStringPtr inputName = midasSession.GetInputNameAllocated(0, allocator);
//     Ort::AllocatedStringPtr outputName = midasSession.GetOutputNameAllocated(0, allocator);
    
//     std::vector<const char*> inputNames = {inputName.get()};
//     std::vector<const char*> outputNames = {outputName.get()};

//     // Define input tensor shape (batch_size, channels, height, width)
//     std::array<int64_t, 4> inputShape = {1, 3, 256, 256}; // Adjust as per your model

//     // Open webcam
//     cv::VideoCapture cap(0);
//     if (!cap.isOpened()) {
//         std::cerr << "Error: Unable to open camera." << std::endl;
//         return -1;
//     }

//     cv::Mat frame, resizedFrame, inputBlob;
//     while (true) {
//         cap >> frame;
//         if (frame.empty()) break;

//         // Resize frame to match model input size
//         cv::resize(frame, resizedFrame, cv::Size(256, 256));
//         resizedFrame.convertTo(resizedFrame, CV_32F, 1.0f / 255.0f); // Normalize

//         // Convert BGR to RGB
//         cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGR2RGB);

//         // Convert image to float tensor
//         std::vector<float> inputTensorValues(1 * 3 * 256 * 256);
//         int index = 0;
//         for (int c = 0; c < 3; c++) {
//             for (int i = 0; i < 256; i++) {
//                 for (int j = 0; j < 256; j++) {
//                     inputTensorValues[index++] = resizedFrame.at<cv::Vec3f>(i, j)[c];
//                 }
//             }
//         }

//         // Create memory info
//         Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);

//         // Create input tensor
//         Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
//             memoryInfo, 
//             inputTensorValues.data(), inputTensorValues.size(), 
//             inputShape.data(), inputShape.size()
//         );

//         // Run inference
//         std::vector<Ort::Value> outputTensors = midasSession.Run(
//             Ort::RunOptions{nullptr}, 
//             inputNames.data(), &inputTensor, 1, 
//             outputNames.data(), 1
//         );

//         // Process output tensor
//         float* outputData = outputTensors[0].GetTensorMutableData<float>();
//         cv::Mat depthMap(256, 256, CV_32F, outputData);

//         // Normalize depth map for display
//         cv::Mat depthDisplay;
//         cv::normalize(depthMap, depthDisplay, 0, 255, cv::NORM_MINMAX);
//         depthDisplay.convertTo(depthDisplay, CV_8U);

//         // Threshold depth map to identify obstacles (white = closer objects)
//         cv::Mat binaryMask;
//         double thresholdValue = 50; // Adjust this based on the scene
//         cv::threshold(depthDisplay, binaryMask, thresholdValue, 255, cv::THRESH_BINARY);

//         // Divide the frame into left, center, and right regions
//         int width = binaryMask.cols;
//         int height = binaryMask.rows;
//         int regionWidth = width / 3;

//         cv::Mat leftRegion = binaryMask(cv::Rect(0, 0, regionWidth, height));
//         cv::Mat centerRegion = binaryMask(cv::Rect(regionWidth, 0, regionWidth, height));
//         cv::Mat rightRegion = binaryMask(cv::Rect(2 * regionWidth, 0, regionWidth, height));

//         // Count white pixels in each region
//         int leftCount = cv::countNonZero(leftRegion);
//         int centerCount = cv::countNonZero(centerRegion);
//         int rightCount = cv::countNonZero(rightRegion);

//         // Determine the obstacle location
//         std::string obstaclePosition;
//         if (centerCount > leftCount && centerCount > rightCount) {
//             obstaclePosition = "Obstacle in CENTER!";
//         } else if (leftCount > rightCount) {
//             obstaclePosition = "Obstacle on LEFT!";
//         } else {
//             obstaclePosition = "Obstacle on RIGHT!";
//         }

//         // Display obstacle detection result
//         cv::putText(frame, obstaclePosition, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

//         // Show original and processed images
//         cv::imshow("Original", frame);
//         cv::imshow("Depth Estimation", depthDisplay);
//         cv::imshow("Obstacle Detection", binaryMask);

//         // Exit loop if 'q' is pressed
//         if (cv::waitKey(1) == 'q') break;
//     }

//     cap.release();
//     cv::destroyAllWindows();
//     return 0;
// }
