// #include <QApplication>
// #include <QLabel>
// #include <QPushButton>
// #include <QVBoxLayout>
// #include <QWidget>
// #include <QTimer>
// #include <opencv2/opencv.hpp>
// #include <QImage>
// #include <QPixmap>

// class ObstacleDetectionGUI : public QWidget {
//     Q_OBJECT  // Necessary for Qt's meta-object system

// public:
//     explicit ObstacleDetectionGUI(QWidget *parent = nullptr);
//     ~ObstacleDetectionGUI();

// public slots:
//     void startCamera();
//     void stopCamera();
//     void updateFrame();

// private:
//     QLabel *videoLabel;
//     QPushButton *startButton, *stopButton;
//     QTimer *timer;
//     cv::VideoCapture cap;
//     bool isRunning = false;
// };

// #include "gui.moc"  // Force MOC processing (important for single-file setup)

// // Constructor
// ObstacleDetectionGUI::ObstacleDetectionGUI(QWidget *parent) : QWidget(parent) {
//     setWindowTitle("Obstacle Detection GUI");

//     videoLabel = new QLabel(this);
//     startButton = new QPushButton("Start", this);
//     stopButton = new QPushButton("Stop", this);

//     QVBoxLayout *layout = new QVBoxLayout();
//     layout->addWidget(videoLabel);
//     layout->addWidget(startButton);
//     layout->addWidget(stopButton);
//     setLayout(layout);

//     timer = new QTimer(this);
//     connect(startButton, &QPushButton::clicked, this, &ObstacleDetectionGUI::startCamera);
//     connect(stopButton, &QPushButton::clicked, this, &ObstacleDetectionGUI::stopCamera);
//     connect(timer, &QTimer::timeout, this, &ObstacleDetectionGUI::updateFrame);
// }

// // Destructor
// ObstacleDetectionGUI::~ObstacleDetectionGUI() {
//     stopCamera();
// }

// // Start Camera
// void ObstacleDetectionGUI::startCamera() {
//     if (!isRunning) {
//         cap.open(0);
//         if (!cap.isOpened()) {
//             videoLabel->setText("Error: Camera not found!");
//             return;
//         }
//         isRunning = true;
//         timer->start(30);
//     }
// }

// // Stop Camera
// void ObstacleDetectionGUI::stopCamera() {
//     if (isRunning) {
//         timer->stop();
//         cap.release();
//         videoLabel->clear();
//         isRunning = false;
//     }
// }

// // Capture and Process Frame
// void ObstacleDetectionGUI::updateFrame() {
//     cv::Mat frame, edges;
//     cap >> frame;
//     if (frame.empty()) return;

//     cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
//     cv::GaussianBlur(edges, edges, cv::Size(5, 5), 1.5);
//     cv::Canny(edges, edges, 50, 150);

//     QImage qimg(edges.data, edges.cols, edges.rows, edges.step, QImage::Format_Grayscale8);
//     videoLabel->setPixmap(QPixmap::fromImage(qimg).scaled(videoLabel->size(), Qt::KeepAspectRatio));
// }

// // Main Function
// int main(int argc, char *argv[]) {
//     QApplication app(argc, argv);
//     ObstacleDetectionGUI window;
//     window.resize(640, 480);
//     window.show();
//     return app.exec();
// }


#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QPixmap>

class ObstacleDetectionGUI : public QWidget {
    Q_OBJECT

public:
    explicit ObstacleDetectionGUI(QWidget *parent = nullptr);
    ~ObstacleDetectionGUI();

public slots:
    void startCamera();
    void stopCamera();
    void updateFrame();

private:
    QLabel *rgbLabel;
    QLabel *edgeLabel;
    QPushButton *startButton, *stopButton;
    QTimer *timer;
    cv::VideoCapture cap;
    bool isRunning = false;
};

#include "gui.moc"

// Constructor
ObstacleDetectionGUI::ObstacleDetectionGUI(QWidget *parent) : QWidget(parent) {
    setWindowTitle("Obstacle Detection GUI");

    rgbLabel = new QLabel(this);
    edgeLabel = new QLabel(this);
    startButton = new QPushButton("Start", this);
    stopButton = new QPushButton("Stop", this);

    QHBoxLayout *videoLayout = new QHBoxLayout();
    videoLayout->addWidget(rgbLabel);
    videoLayout->addWidget(edgeLabel);

    QVBoxLayout *layout = new QVBoxLayout();
    layout->addLayout(videoLayout);
    layout->addWidget(startButton);
    layout->addWidget(stopButton);
    setLayout(layout);

    timer = new QTimer(this);
    connect(startButton, &QPushButton::clicked, this, &ObstacleDetectionGUI::startCamera);
    connect(stopButton, &QPushButton::clicked, this, &ObstacleDetectionGUI::stopCamera);
    connect(timer, &QTimer::timeout, this, &ObstacleDetectionGUI::updateFrame);
}

// Destructor
ObstacleDetectionGUI::~ObstacleDetectionGUI() {
    stopCamera();
}

// Start Camera
void ObstacleDetectionGUI::startCamera() {
    if (!isRunning) {
        cap.open(0);
        if (!cap.isOpened()) {
            rgbLabel->setText("Error: Camera not found!");
            return;
        }
        isRunning = true;
        timer->start(30);
    }
}

// Stop Camera
void ObstacleDetectionGUI::stopCamera() {
    if (isRunning) {
        timer->stop();
        cap.release();
        rgbLabel->clear();
        edgeLabel->clear();
        isRunning = false;
    }
}

// Capture and Process Frame
void ObstacleDetectionGUI::updateFrame() {
    cv::Mat frame, gray, edges;
    cap >> frame;
    if (frame.empty()) return;

    // Convert to grayscale and detect edges
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
    cv::Canny(gray, edges, 50, 150);

    // Convert RGB frame to QImage
    QImage rgbImg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
    rgbLabel->setPixmap(QPixmap::fromImage(rgbImg).scaled(rgbLabel->size(), Qt::KeepAspectRatio));

    // Convert Edge-detected frame to QImage
    QImage edgeImg(edges.data, edges.cols, edges.rows, edges.step, QImage::Format_Grayscale8);
    edgeLabel->setPixmap(QPixmap::fromImage(edgeImg).scaled(edgeLabel->size(), Qt::KeepAspectRatio));
}

// Main Function
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    ObstacleDetectionGUI window;
    window.resize(800, 400);
    window.show();
    return app.exec();
}

