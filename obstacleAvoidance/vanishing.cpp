#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Function to compute the intersection of two lines
Point2f computeIntersection(Vec4i line1, Vec4i line2) {
    Point2f p1(line1[0], line1[1]), p2(line1[2], line1[3]);
    Point2f p3(line2[0], line2[1]), p4(line2[2], line2[3]);

    // Compute line equations: Ax + By = C
    float A1 = p2.y - p1.y, B1 = p1.x - p2.x, C1 = A1 * p1.x + B1 * p1.y;
    float A2 = p4.y - p3.y, B2 = p3.x - p4.x, C2 = A2 * p3.x + B2 * p3.y;

    float det = A1 * B2 - A2 * B1;
    if (fabs(det) < 1e-10) return Point2f(-1, -1); // Parallel lines

    return Point2f((B2 * C1 - B1 * C2) / det, (A1 * C2 - A2 * C1) / det);
}

int main() {
    // Open webcam
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open webcam!" << endl;
        return -1;
    }

    while (true) {
        Mat frame, gray, edges;
        cap >> frame;
        if (frame.empty()) break;
        // Convert to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Apply Canny Edge Detection
        Canny(gray, edges, 50, 200);

        // Detect lines using Hough Transform
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 200);

        vector<Point2f> intersections;
        for (size_t i = 0; i < lines.size(); i++) {
            for (size_t j = i + 1; j < lines.size(); j++) {
                Point2f pt = computeIntersection(lines[i], lines[j]);
                if (pt.x >= 0 && pt.y >= 0 && pt.x < frame.cols && pt.y < frame.rows) {
                    intersections.push_back(pt);
                }
            }
        }

        // Compute average intersection point (Vanishing Point)
        if (!intersections.empty()) {
            Point2f vp(0, 0);
            for (auto& pt : intersections) vp += pt;
            vp *= (1.0 / intersections.size());

            // Draw vanishing point
            circle(frame, vp, 10, Scalar(0, 0, 255), -1);
        }

        // Show results
        imshow("Edges", edges);
        imshow("Vanishing Point Detection", frame);

        if (waitKey(1) == 27) break;  // Press 'Esc' to exit
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
