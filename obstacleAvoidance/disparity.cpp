#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Open two camera feeds (left & right)
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open camera streams!" << endl;
        return -1;
    }

    // StereoBM (Block Matching) for depth estimation
    Ptr<StereoBM> stereo = StereoBM::create(16, 15);

    while (true) {
        Mat frameL, frameR, grayL, grayR, disparity;
        cap >> frameL;
        cap >> frameR;
        if (frameL.empty() || frameR.empty()) break;

        // Convert to grayscale
        cvtColor(frameL, grayL, COLOR_BGR2GRAY);
        cvtColor(frameR, grayR, COLOR_BGR2GRAY);

        // Compute disparity map
        stereo->compute(grayL, grayR, disparity);

        // Normalize disparity for visualization
        normalize(disparity, disparity, 0, 255, NORM_MINMAX, CV_8U);

        // Show results
        imshow("Left Camera", frameL);
        imshow("Right Camera", frameR);
        imshow("Disparity Map (Depth)", disparity);

        if (waitKey(1) == 27) break;  // Press 'Esc' to exit
    }

    cap.release();
    // capR.release();
    destroyAllWindows();
    return 0;
}
