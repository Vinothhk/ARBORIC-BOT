#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#include <cstdlib>
#include <thread>
#include <mutex>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <queue>

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

struct MarkerNode {
    int marker;
    string relation;
};

struct SharedData {
    std::mutex mtx;
    bool isGUIrequest = false;
    bool markerVisible = false;
    int detectedMarkerID = -1;
    float x,y,z,roll,pitch,yaw;
    bool obstacleDetected = false;
    int serial_port;
    int targetMarkerID = -1;
    map<pair<int, int>, string> MarkerMap;
};


class WhileTrueNode : public BT::ControlNode
{
public:
    WhileTrueNode(const std::string& name) : BT::ControlNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        while (true)
        {
            for (auto& child : children_nodes_)
            {
                BT::NodeStatus status = child->executeTick();
                
                if (status == BT::NodeStatus::SUCCESS)  
                {
                    return BT::NodeStatus::SUCCESS; // Stop when marker is found
                }
            }
            this_thread::sleep_for(std::chrono::seconds(1));  // Retry delay
        }
        return BT::NodeStatus::FAILURE;
    }
};

class WaitForRequest : public BT::SyncActionNode {
    public:
        explicit WaitForRequest(const std::string &name,const BT::NodeConfig &config, std::shared_ptr<SharedData> sharedData) : BT::SyncActionNode(name, config), sharedData(sharedData) {}
        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<int>("status")};
        }

        BT::NodeStatus tick() override {
            // std::cout << "Waiting for request from GUI...\n";
            // std :: cout <<  "Press key 's': " << std::endl;
            // std :: cin >> o;
            while (!sharedData->isGUIrequest) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // return BT::NodeStatus::RUNNING;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return BT::NodeStatus::SUCCESS;
            }
    private:
        std::shared_ptr<SharedData> sharedData;
};

class MarkerFound : public BT::SyncActionNode {
    public:
        explicit MarkerFound(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData)
            : BT::SyncActionNode(name, config), sharedData(sharedData) {}
    
        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<int>("marker_id")};
        }
    
        BT::NodeStatus tick() override {
            bool found = sharedData->markerVisible;
            int marker_id = sharedData->detectedMarkerID;
            BT::TreeNode::setOutput("marker_id",marker_id);

            if(!found){
                std::cout << "Marker not found..\n";
                return BT::NodeStatus::FAILURE;
            }
            else{
                std::cout << "Marker found !\n";
                BT::TreeNode::setOutput("marker_id",marker_id);
                return BT::NodeStatus::SUCCESS;
            }
        }
    private:
        std::shared_ptr<SharedData> sharedData;
    };    


class SpinSearch : public BT::SyncActionNode {
    public:
        explicit SpinSearch(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData) 
            : BT::SyncActionNode(name, config), sharedData(sharedData) {}
        
        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<int>("marker_id")};
        }
        
        BT::NodeStatus tick() override {
            std::cout << "Spinning to search for marker...\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<SharedData> sharedData;
};
    
class MoveSlightly : public BT::SyncActionNode {
    public:
        explicit MoveSlightly(const std::string& name,const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData) 
            : BT::SyncActionNode(name, config), sharedData(sharedData) {}
        
        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<int>("marker_id")};
        }
        
        BT::NodeStatus tick() override {
            std::cout << "Moving slightly to reposition for marker detection...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<SharedData> sharedData;
    };
        
class StoreMarkerRelation : public BT::SyncActionNode {
public:
    explicit StoreMarkerRelation(const std::string& name,const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData) 
        : BT::SyncActionNode(name, config), sharedData(sharedData) {}
    
    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<int>("marker_id")};
    }

    BT::NodeStatus tick() override {
        std::cout << "Storing marker relation...\n";
        return BT::NodeStatus::SUCCESS;
    }

        
    string getOppositeDirection(const string& direction) {
        if (direction == "Right") return "Left";
        if (direction == "Left") return "Right";
        return direction; // Default case (if other directions exist)
    }

    // Function to find the relation between two markers
    void findRelation(int start, int end, map<pair<int, int>, string>& myMap) {
        // Step 1: Convert map into bidirectional adjacency list with opposite directions
        unordered_map<int, vector<pair<int, string>>> adj;
        for (const auto& it : myMap) {
            int from = it.first.first;
            int to = it.first.second;
            string direction = it.second;

            // Store both directions (with reversed meaning)
            adj[from].push_back({to, direction});
            adj[to].push_back({from, getOppositeDirection(direction)}); // Reverse connection
        }

        // Step 2: BFS to find the shortest path
        queue<MarkerNode> q;
        unordered_map<int, bool> visited;

        q.push({start, ""});
        visited[start] = true;

        while (!q.empty()) {
            MarkerNode current = q.front();
            q.pop();

            if (current.marker == end) {
                cout << "Relation from " << start << " to " << end << ": " << current.relation << endl;
                return;
            }

            for (const auto& neighbor : adj[current.marker]) {
                int nextNode = neighbor.first;
                string newDirection = neighbor.second;

                // Merge consecutive identical directions
                string newRelation;
                if (current.relation.empty() || current.relation == newDirection) {
                    newRelation = newDirection;
                } else {
                    newRelation = current.relation + " → " + newDirection;
                }

                if (!visited[nextNode]) {
                    visited[nextNode] = true;
                    q.push({nextNode, newRelation});
                }
            }
        }

        cout << "No relation found between " << start << " and " << end << endl;
    }

private:
    std::shared_ptr<SharedData> sharedData;
    // unordered_map<int, vector<pair<int, string>>> adj;
};

class ApproachMarker : public BT::SyncActionNode {
public:
    ApproachMarker(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData) 
        : BT::SyncActionNode(name, config),sharedData(sharedData) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("marker_id") };
    }

    BT::NodeStatus tick() override {
        int marker_id;
        if (!getInput("marker_id", marker_id)) {
            std::cerr << "Missing marker_id input!\n";
            return BT::NodeStatus::FAILURE;
        }
        auto serial_port = sharedData->serial_port;
        std::cout << "Approaching marker " << marker_id << "...\n";

        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::string command = std::to_string(marker_id) + "\n";
        for (int i = 0; i < 10; i++) {
            cout<<"Sending command.. "<<endl;
            write(serial_port, command.c_str(), command.size());
            this_thread::sleep_for(1000ms);
        }

        return BT::NodeStatus::SUCCESS;
    }
private:
    std::shared_ptr<SharedData> sharedData;
};
        
class AvoidObstacles : public BT::SyncActionNode {
public:
    AvoidObstacles(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Avoiding obstacles...\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class FollowPath : public BT::SyncActionNode {
public:
    FollowPath(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Following path...\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class DualFeedGUI : public QWidget {
    Q_OBJECT

public:
    explicit DualFeedGUI(std::shared_ptr<SharedData> &sharedData, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, QWidget *parent = nullptr);
    ~DualFeedGUI();

    void startCameras();
    void stopCameras();
    void updateFrames();
    void handleMarker1();
    void handleMarker2();
    void handleMarker3();
    std::shared_ptr<SharedData> sharedData;

private:
    QLabel *rgbLabel, *edgeLabel;
    QPushButton *startButton, *stopButton, *markerButton1, *markerButton2, *markerButton3;
    QTimer *timer;
    cv::VideoCapture cap1;
    bool isRunning = false;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    void processAruco(cv::Mat &frame);
    void processObstacleDetection(cv::Mat &frame);
};

#include "obstacle_avoidance.moc"
#include <opencv4/opencv2/objdetect/aruco_detector.hpp>

DualFeedGUI::DualFeedGUI(std::shared_ptr<SharedData> &sharedData,const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, QWidget *parent) {

    // std::cout << "Constructor Camera Matrix:\n" << cameraMatrix << std::endl;
    // std::cout << "Constructor Distortion Coefficients:\n" << distCoeffs << std::endl;

    this->cameraMatrix = cameraMatrix;
    this->distCoeffs = distCoeffs;
    this->sharedData = sharedData;

    setWindowTitle("ArUco Pose & Obstacle Detection");

    rgbLabel = new QLabel(this);
    edgeLabel = new QLabel(this);
    startButton = new QPushButton("Start Camera", this);
    stopButton = new QPushButton("Stop Camera", this);
    markerButton1 = new QPushButton("Marker 1", this);
    markerButton2 = new QPushButton("Marker 2", this);
    markerButton3 = new QPushButton("Marker 3", this);

    QHBoxLayout *videoLayout = new QHBoxLayout();
    videoLayout->addWidget(rgbLabel);
    videoLayout->addWidget(edgeLabel);

    // Layout for control buttons
    QHBoxLayout *controlLayout = new QHBoxLayout();
    controlLayout->addWidget(startButton);
    controlLayout->addWidget(stopButton);

    // Layout for marker buttons
    QHBoxLayout *markerLayout = new QHBoxLayout();
    markerLayout->addWidget(markerButton1);
    markerLayout->addWidget(markerButton2);
    markerLayout->addWidget(markerButton3);

    // Main layout
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(videoLayout);
    mainLayout->addLayout(controlLayout);
    mainLayout->addLayout(markerLayout);

    // Set the main layout
    setLayout(mainLayout);

    timer = new QTimer(this);
    connect(startButton, &QPushButton::clicked, this, &DualFeedGUI::startCameras);
    connect(stopButton, &QPushButton::clicked, this, &DualFeedGUI::stopCameras);
    connect(timer, &QTimer::timeout, this, &DualFeedGUI::updateFrames);

    connect(markerButton1, &QPushButton::clicked, this, &DualFeedGUI::handleMarker1);
    connect(markerButton2, &QPushButton::clicked, this, &DualFeedGUI::handleMarker2);
    connect(markerButton3, &QPushButton::clicked, this, &DualFeedGUI::handleMarker3);

}

DualFeedGUI::~DualFeedGUI() {
    stopCameras();
}

void DualFeedGUI::handleMarker1() {
    // qDebug() << "Marker 1 button clicked";
    qDebug() << "Requested to Move to Marker 1";
    sharedData->isGUIrequest = true;
    sharedData->targetMarkerID = 1;
}

void DualFeedGUI::handleMarker2() {
    // qDebug() << "Marker 2 button clicked";
    qDebug() << "Requested to Move to Marker 2";
    sharedData->isGUIrequest = true;
    sharedData->targetMarkerID = 2;
}

void DualFeedGUI::handleMarker3() {
    // qDebug() << "Marker 3 button clicked";
    qDebug() << "Requested to Move to Marker 3";
    sharedData->isGUIrequest = true;
    sharedData->targetMarkerID = 3;
}

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

void DualFeedGUI::startCameras() {
    if (!isRunning) {
        cap1.open(0); // Camera 1 for ArUco
        // cap1.open(1); // Camera 2 for Obstacle Detection
        
        if (!cap1.isOpened()) {
            rgbLabel->setText("Error: Camera(s) not found!");
            return;
        }
        
        isRunning = true;
        timer->start(30);
    }
}

void DualFeedGUI::stopCameras() {
    if (isRunning) {
        timer->stop();
        cap1.release();
        rgbLabel->clear();
        edgeLabel->clear();
        isRunning = false;
    }
}

void DualFeedGUI::updateFrames() {
    cv::Mat frame1, frame2;
    cap1 >> frame1;
    cap1 >> frame2;
    
    if (frame1.empty() || frame2.empty()) return;
    
    processAruco(frame1);
    processObstacleDetection(frame2);
    
    QImage img1(frame1.data, frame1.cols, frame1.rows, frame1.step, QImage::Format_RGB888);
    rgbLabel->setPixmap(QPixmap::fromImage(img1.rgbSwapped()).scaled(rgbLabel->size(), Qt::KeepAspectRatio));
    
    QImage img2(frame2.data, frame2.cols, frame2.rows, frame2.step, QImage::Format_Grayscale8);
    edgeLabel->setPixmap(QPixmap::fromImage(img2).scaled(edgeLabel->size(), Qt::KeepAspectRatio));
}

void DualFeedGUI::processAruco(cv::Mat &frame) {
    // std::cout << "PA Camera Matrix:\n" << cameraMatrix << std::endl;
    // std::cout << "PA Distortion Coefficients:\n" << distCoeffs << std::endl;

    // std::lock_guard<std::mutex> lock(sharedData.mtx);
    // sharedData->detectedMarkerID = 10;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
    detector.detectMarkers(frame, markerCorners, markerIds, rejected);
    
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        sharedData->markerVisible = true;
    }
    else{
        // sharedData->detectedMarkerID = -1;
        sharedData->markerVisible = false;
    }
    for (int id : markerIds) {
        // std::cout << "Detected marker ID: " << id << std::endl;
        sharedData->detectedMarkerID = id;
    }

    size_t nMarkers = markerCorners.size();
    vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    float markerLength = 0.03;
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    if (!markerCorners.empty() && !cameraMatrix.empty() && !distCoeffs.empty()) {
    // Calculate pose for each marker
    try {
        if(!markerIds.empty()) {
        for (size_t i = 0; i < nMarkers; i++) {
            cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }
    }
    catch (const std::exception &e) {
        qDebug() << "Error in SolvePnP: " << e.what();
    }


    try{
        for(unsigned int i = 0; i < markerIds.size(); i++){
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength, 2);
        }
    }
    catch (const std::exception &e) {
        qDebug() << "Error in drawFrameAxes: " << e.what();
    }

    try{
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.03, cameraMatrix, distCoeffs, rvecs, tvecs);
    }
    catch (const std::exception &e) {
        qDebug() << "Error in estimatePoseSingleMarkers: " << e.what();
        }
    }

    for (size_t i = 0; i < markerIds.size(); ++i) {
        // Display marker ID and position
        // std::cout << "Marker ID: " << markerIds[i]
        //           << " Translation: " << tvecs[i]
        //           << " Rotation: " << rvecs[i] << std::endl;

        sharedData->x = tvecs[i][0];
        sharedData->y = tvecs[i][1];
        sharedData->z = tvecs[i][2];
        sharedData->roll = rvecs[i][0];
        sharedData->pitch = rvecs[i][1];
        sharedData->yaw = rvecs[i][2];
    }

}

void DualFeedGUI::processObstacleDetection(cv::Mat &frame) {
    cv::Mat edges;
    cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(edges, edges, cv::Size(5, 5), 1.5);
    cv::Canny(edges, edges, 50, 150);
    frame = edges;
}

void runBehaviorTree(std::shared_ptr<SharedData> sharedData) {
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<WhileTrueNode>("WhileTrue");
    factory.registerNodeType<WaitForRequest>("WaitForRequest",sharedData);
    factory.registerNodeType<MarkerFound>("MarkerFound",sharedData);
    factory.registerNodeType<SpinSearch>("SpinSearch",sharedData);
    factory.registerNodeType<MoveSlightly>("MoveSlightly",sharedData);
    factory.registerNodeType<StoreMarkerRelation>("StoreMarkerRelation",sharedData);
    factory.registerNodeType<ApproachMarker>("ApproachMarker",sharedData);
    factory.registerNodeType<AvoidObstacles>("AvoidObstacles");
    factory.registerNodeType<FollowPath>("FollowPath");   

    auto tree = factory.createTreeFromFile("./../tree/treeV1.xml");
    
    // Keep running behavior tree
    tree.tickWhileRunning();
}

int openSerialPort(const std::string& port) {
    int serial_port = open(port.c_str(), O_RDWR);
    if (serial_port < 0) {
        std::cerr << "Failed to open serial port: " << port << "\n";
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error in getting terminal attributes\n";
        return -1;
    }

    tty.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
    tcsetattr(serial_port, TCSANOW, &tty);
    return serial_port;
}


int main(int argc, char *argv[]) {

    QApplication app(argc, argv);
    auto sharedData = std::make_shared<SharedData>();

    map<pair<int, int>, string> myMap;
    myMap[{1, 2}] = "Right";
    myMap[{1, 3}] = "Left";
    myMap[{2, 6}] = "Right";
    myMap[{3, 4}] = "Left";
    myMap[{4, 5}] = "Left";
    myMap[{5, 6}] = "Left";


    // Load Camera Calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::string load_path = "./../CamCalibration/camera_parameters.yaml";
    if (loadCameraCalibration(load_path, cameraMatrix, distCoeffs)) {
        std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
        std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
    } else {
        std::cerr << "Failed to load calibration data!" << std::endl;
        return -1;
    }

    // int serial_port = openSerialPort("/dev/ttyACM0");
    int serial_port = 2;
    if (serial_port < 0) {
        return 1;  // Exit if serial port fails
    }
    sharedData->serial_port = serial_port;
    sharedData->MarkerMap = myMap;
    // Start Aruco Processing in a separate QThread
    // ArucoProcessing arucoThread;
    // arucoThread.start();

    // Run Behavior Tree in a separate thread
    std::thread btThread(runBehaviorTree, sharedData);
    // Start GUI
    DualFeedGUI window(sharedData, cameraMatrix, distCoeffs);
    window.resize(1280, 480);
    window.show();

    int result = app.exec();

    // Cleanup threads
    // arucoThread.quit();
    // arucoThread.wait();
    btThread.join();

    return result;
}