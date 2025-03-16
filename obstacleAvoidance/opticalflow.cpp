#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cout << "Error: Camera not found!" << endl;
        return -1;
    }

    Mat prevGray, gray, flow, frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        if (!prevGray.empty()) {
            // Compute Optical Flow (Farneback)
            calcOpticalFlowFarneback(prevGray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

            // Draw flow vectors
            for (int y = 0; y < frame.rows; y += 10) {
                for (int x = 0; x < frame.cols; x += 10) {
                    Point2f flowAtXY = flow.at<Point2f>(y, x);
                    line(frame, Point(x, y), Point(cvRound(x + flowAtXY.x), cvRound(y + flowAtXY.y)), Scalar(0, 255, 0), 1);
                    circle(frame, Point(x, y), 1, Scalar(0, 0, 255), -1);
                }
            }
        }

        // Update previous frame
        prevGray = gray.clone();

        // Show result
        imshow("Optical Flow", frame);
        if (waitKey(1) == 27) break;  // Press 'ESC' to exit
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
