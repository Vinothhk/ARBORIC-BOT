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
#include <onnxruntime_cxx_api.h>

#define FRAME_CENTER_X 320  // Adjust based on camera resolution
#define FRAME_CENTER_Y 240
#define MARKER_SIZE_THRESHOLD 300000  // Adjust based on marker size wh

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

struct MarkerNode {
    int marker;
    string relation;
    vector<int> path; // To store the path of markers
    vector<string> directions; // To store the list of directions
};


struct SharedData {
    std::mutex mtx;
    std::vector<cv::Point2f> corners;
    string ObstacleDirection = "";
    string imm_Dir = "";
    int imm_ID;
    bool isGUIrequest = false;
    bool markerVisible = false;
    int detectedMarkerID = -1;
    float x,y,z,roll,pitch,yaw;
    bool obstacleDetected = false;
    int serial_port;
    int targetMarkerID = -1;
    bool GoalReached = false;
    map<pair<int, int>, string> MarkerMap;
};

class ParallelNode : public BT::ControlNode {
public:
    ParallelNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ControlNode(name, config), success_threshold_(1), failure_threshold_(1) {}

    void set_success_threshold(int threshold) { success_threshold_ = threshold; }
    void set_failure_threshold(int threshold) { failure_threshold_ = threshold; }

    BT::NodeStatus tick() override {
        std::atomic<int> success_count{0};
        std::atomic<int> failure_count{0};
        std::vector<std::thread> threads;
        std::mutex child_mutex;

        for (BT::TreeNode* child : children_nodes_) {
            threads.emplace_back([&, child]() {
                BT::NodeStatus child_status;
                {
                    std::lock_guard<std::mutex> lock(child_mutex); // Ensure safe execution
                    child_status = child->executeTick();
                }

                if (child_status == BT::NodeStatus::SUCCESS) {
                    success_count.fetch_add(1, std::memory_order_relaxed);
                } else if (child_status == BT::NodeStatus::FAILURE) {
                    failure_count.fetch_add(1, std::memory_order_relaxed);
                }
            });
        }

        for (auto& t : threads) t.join(); // Wait for all child nodes

        if (success_count >= success_threshold_) return BT::NodeStatus::SUCCESS;
        if (failure_count >= failure_threshold_) return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::RUNNING;
    }

private:
    int success_threshold_;
    int failure_threshold_;
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

            std::string command = "2 2\n";
            write(sharedData->serial_port, command.c_str(), command.length()); 
            auto start_time = std::chrono::steady_clock::now();
            while (!sharedData->markerVisible) {
                std::cout << "Searching for marker...\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // Check if 5 seconds have passed
                auto elapsed_time = std::chrono::steady_clock::now() - start_time;
                if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= 5) {
                    std::cout << "Marker not found within 5 seconds. Returning FAILURE.\n";
                    return BT::NodeStatus::FAILURE;
                }
            }

            // If marker is detected before timeout
            std::string stop_cmd = "0 0\n";
            write(sharedData->serial_port, stop_cmd.c_str(), stop_cmd.length());
            std::cout << "Marker detected! Returning SUCCESS.\n";
            return BT::NodeStatus::SUCCESS;
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

            std::string command = "1 1\n";
            write(sharedData->serial_port, command.c_str(), command.length()); 

            std::this_thread::sleep_for(std::chrono::seconds(2));

            std::string stop_cmd = "0 0\n";
            write(sharedData->serial_port, stop_cmd.c_str(), stop_cmd.length());
            
            if (sharedData->markerVisible) {
                return BT::NodeStatus::SUCCESS;
            }
            else{
                return BT::NodeStatus::FAILURE;
            }
        }
    private:
        std::shared_ptr<SharedData> sharedData;
    };
        
class StoreMarkerRelation : public BT::SyncActionNode{
public:
    explicit StoreMarkerRelation(const std::string& name,const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData) 
        : BT::SyncActionNode(name, config), sharedData(sharedData) {}
    
    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<int>("marker_id")};
    }

    BT::NodeStatus tick() override {
        std::cout << "Storing marker relation...\n";
        pair<string, int> result;
        std::string direction;
        if (sharedData->markerVisible){
            if( sharedData-> detectedMarkerID == sharedData->targetMarkerID){
                std::cout << "Marker is already in front of the robot!" << std::endl;
                sharedData->imm_Dir = "NONE";
                return BT::NodeStatus::SUCCESS;
            }
            else{
                result = findRelation(sharedData->detectedMarkerID, sharedData->targetMarkerID, sharedData->MarkerMap);
                sharedData->imm_Dir = result.first;
                direction = result.first;
                sharedData->imm_ID = result.second;
            }
        }
        if (direction == "NONE"){
            std::cout << "The Marker has to move straight. No Turning Needed" <<std::endl;
        }
        else if (direction == "Right" || direction == "Left" ){
            std::cout << "The Robot has to move: " << direction << std::endl;
        }
        else {
            std::cout << "Marker not visible!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

        
    string getReverseDirection(const string& direction) {
        if (direction == "Right") return "Left";
        if (direction == "Left") return "Right";
        return ""; // Default case for unknown directions
    }
    
    // Function to find the relation from start to end using BFS
    pair<string, int> findRelation(int start, int end, map<pair<int, int>, string>& myMap) {
        if (start == end){
            std::cout << "Start and end markers are the same!" << std::endl;
            return make_pair("NONE", start);
        }
        // Step 1: Convert map into bidirectional adjacency list
        unordered_map<int, vector<pair<int, string>>> adj;
        for (const auto& it : myMap) {
            int from = get<0>(it.first);
            int to = get<1>(it.first);
            string direction = it.second;
    
            // Store both directions
            adj[from].push_back({to, direction});
            if (myMap.find({to, from}) == myMap.end()) { // If reverse direction is not explicitly declared
                adj[to].push_back({from, getReverseDirection(direction)});
            }
        }
    
        // Step 2: BFS to find shortest path
        queue<MarkerNode> q;
        unordered_map<int, bool> visited;
        std::string imm_Dir;
        int imm_ID;
        q.push({start, "", {start}, {}}); // Initialize with start marker
        visited[start] = true;
    
        while (!q.empty()) {
            MarkerNode current = q.front();
            q.pop();
            
            if (current.marker == end) {
                cout << "Relation from " << start << " to " << end << ": " << current.relation << endl;
                cout << "Intermediate markers: ";
                for (int marker : current.path) {
                    cout << marker << " ";
                }
                cout << endl;
    
                cout << "Direction order: ";
                for (const string& dir : current.directions) {
                    cout << dir << " ";
                }
                cout << endl;
    
                // // Iterate over the vector with index
                // for (size_t i = 0; i < current.directions.size(); ++i) {
                //     std::cout << "Index: " << i << ", Value: " << current.directions[i] << endl;
                // }
                imm_Dir = current.directions[0];
                imm_ID = current.path[1];
                std::cout << "initial params: " << imm_ID << " " << imm_Dir << endl;
                return make_pair(imm_Dir, imm_ID);
            }
    
            for (const auto& neighbor : adj[current.marker]) {
                int nextNode = neighbor.first;
                string newDirection = neighbor.second;
    
                // Merge identical consecutive directions
                string newRelation;
                if (current.relation.empty() || current.relation == newDirection) {
                    newRelation = newDirection;
                } else {
                    newRelation = current.relation + " → " + newDirection;
                }
    
                if (!visited[nextNode]) {
                    visited[nextNode] = true;
    
                    // Create a new path and direction list for the next node
                    vector<int> newPath = current.path;
                    newPath.push_back(nextNode);
    
                    vector<string> newDirections = current.directions;
                    newDirections.push_back(newDirection);
    
                    q.push({nextNode, newRelation, newPath, newDirections});
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
// class ApproachMarker : public BT::SyncActionNode {
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
        std::cout << "Detected: " << sharedData->detectedMarkerID << std::endl;
        std::cout << "Target: " << sharedData->targetMarkerID << std::endl;
        
        if (sharedData->detectedMarkerID != sharedData->targetMarkerID){
            std::cout << "Target marker not found. so not approaching..";
            return BT::NodeStatus::FAILURE;
        }
        auto serial_port = sharedData->serial_port;

        string dir = sharedData->imm_Dir;
        std::cout << "Rotating to see the marker!" << std::endl;
        std::string cmd;
        if (dir ==  "NONE"){
            cout << "Moving Straight.." << endl;
            cmd = "1 1\n";
        }
        else if (dir ==  "Right"){
            cout << "Moving RIGHT.." << endl;
            cmd = "0 1\n";
        }
        else if (dir ==  "Left"){
            cout << "Moving LEFT.." << endl;
            cmd = "1 0\n";
        }
        write(serial_port, cmd.c_str(), cmd.length());


        while (!sharedData->obstacleDetected && sharedData->markerVisible){
            std::cout << "Approaching marker " << sharedData->targetMarkerID << "...\n";
            MoveToMarker();
            // std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        if(sharedData->ObstacleDirection == "RIGHT"){
            std::cout << "Obstacle on RIGHT. Robot is moving left .. "<<std::endl;
            std ::string cmd = "1 0\n";
            write(serial_port, cmd.c_str(), cmd.length());
            std::this_thread::sleep_for(std::chrono::seconds(2));
            cmd = "0 0\n";
            write(serial_port, cmd.c_str(), cmd.length());
        }
        else if(sharedData->ObstacleDirection == "CENTER") {
            std::cout << "Obstacle on CENTER. Robot is moving left .. "<<std::endl;
            std ::string cmd = "1 0\n";
            write(serial_port, cmd.c_str(), cmd.length());
            std::this_thread::sleep_for(std::chrono::seconds(2));
            cmd = "0 0\n";
            write(serial_port, cmd.c_str(), cmd.length());
        }
        else if(sharedData->ObstacleDirection == "LEFT") {
            std::cout << "Obstacle on LEFT. Robot is moving right .. "<<std::endl;
            std ::string cmd = "0 1\n";
            write(serial_port, cmd.c_str(), cmd.length());
            std::this_thread::sleep_for(std::chrono::seconds(2));
            cmd = "0 0\n";
            write(serial_port, cmd.c_str(), cmd.length());
        }
        
        // std::string command = std::to_string(marker_id) + "\n";
        // for (int i = 0; i < 10; i++) {
        //     cout<<"Sending command.. "<<endl;
        //     write(serial_port, command.c_str(), command.size());
        //     this_thread::sleep_for(1000ms);
        // }
        if (sharedData->markerVisible && sharedData->detectedMarkerID == sharedData->targetMarkerID) {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }

    void MoveToMarker(){
        std::cout << "Corners: [ ";
        corners = sharedData->corners;
        // std::cout << "[ ";
        // for (const auto& pt : corners) {
        //     std::cout << "(" << pt.x << ", " << pt.y << ") ";
        // }
        // std::cout << "] ";
        // std::cout << "]" << std::endl;
        Point2f marker_center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25;
        int marker_x = static_cast<int>(marker_center.x);
        int marker_y = static_cast<int>(marker_center.y);

        // Compute the marker size (approximate area)
        float marker_size = norm(corners[0] - corners[2]) * norm(corners[1] - corners[3]);
        // std::cout << "Marker Size: " << marker_size << endl;
        // Movement Logic
        std::string cmnd;
        std::string stop = "0 0\n";
        if (marker_size > MARKER_SIZE_THRESHOLD) {
            std::cout << "STOP" << std::endl;
            write(sharedData->serial_port, stop.c_str(), stop.length());
            sharedData->GoalReached = true;
            std::cout << "Marker is large enough. Stopping robot." << std::endl;
            std :: cout << "============================================" << std::endl;
        } else if (abs(marker_x - FRAME_CENTER_X) < 50) {  // Centered
            cmnd = "1 1\n";
            write(sharedData->serial_port, cmnd.c_str(), cmnd.length());
            std::cout << "FORWARD" << std::endl;
        } else if (marker_x < FRAME_CENTER_X) {  // Marker is left
            cmnd = "1 0\n";
            write(sharedData->serial_port, cmnd.c_str(), cmnd.length());
            std::cout << "LEFT" << std::endl;
        } else {  // Marker is right
            cmnd = "0 1\n";
            write(sharedData->serial_port, cmnd.c_str(), cmnd.length());
            std::cout << "RIGHT" << std::endl;
        }
    }
private:
    std::shared_ptr<SharedData> sharedData;
    std::vector<cv::Point2f> corners;
};
    
class AvoidObstacles : public BT::SyncActionNode {
// class AvoidObstacles : public BT::SyncActionNode {
public:
    AvoidObstacles(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Avoiding obstacles..." <<std::endl;
        int i=1;
        // for (i; i<10; i++){
        //     std::cout << "AO: "<<i<< std::endl;
        // }
        while(i>0){
            std::cout << "AO: "<<i<< std::endl;
            i++;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class FollowPath : public BT::SyncActionNode {
// class FollowPath : public BT::SyncActionNode {
public:
    FollowPath(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Following path..." <<std::endl;
        int i=1;
        // for (i; i<10; i++){
        //     std::cout << "FP: "<<i<< std::endl;
        // }
        while(i>0){
            std::cout << "FP: "<<i<< std::endl;
            i++;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class GoalReached : public BT::SyncActionNode {
public:
    GoalReached(const std::string& name,const BT::NodeConfiguration& config, std::shared_ptr<SharedData> sharedData) 
        : BT::SyncActionNode(name, config),sharedData(sharedData) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("marker_id") };
    }
    BT::NodeStatus tick() override {
        if (sharedData->GoalReached){
            std::cout << "Goal Reached :)) " <<std::endl;
            std :: cout << "============================================" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else if(sharedData->markerVisible && sharedData->obstacleDetected){
            std::cout << "Goal Reached" << std::endl;
            std :: cout << "============================================" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else {
            std::cout << "Goal Not Reached :( " <<std::endl;
        }
        return BT::NodeStatus::FAILURE;
    }
private:
    std::shared_ptr<SharedData> sharedData;
};

class DualFeedGUI : public QWidget {
    Q_OBJECT

public:
    explicit DualFeedGUI(std::shared_ptr<SharedData> &sharedData, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,Ort::Session* midasSession,std::array<int64_t, 4> &inputShape,std::vector<const char*> &inputNames,std::vector<const char*> &outputNames, QWidget *parent = nullptr);
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
    Ort::Session* midasSession;
    std::vector<const char*> inputNames,outputNames;
    std::array<int64_t, 4> inputShape;
    void processAruco(cv::Mat &frame);
    void processObstacleDetection(cv::Mat &frame,Ort::Session* midasSession,std::array<int64_t, 4> &inputShape,std::vector<const char*> &inputNames,std::vector<const char*> &outputNames);
};
#include "obstacle_avoidance.moc"
#include <opencv4/opencv2/objdetect/aruco_detector.hpp>

DualFeedGUI::DualFeedGUI(std::shared_ptr<SharedData> &sharedData,const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,Ort::Session* midasSession,std::array<int64_t, 4> &inputShape,std::vector<const char*> &inputNames,std::vector<const char*> &outputNames, QWidget *parent) :midasSession(midasSession) {

    // std::cout << "Constructor Camera Matrix:\n" << cameraMatrix << std::endl;
    // std::cout << "Constructor Distortion Coefficients:\n" << distCoeffs << std::endl;
    // this->midasSession = midasSession;
    this->inputShape = inputShape;
    this->inputNames =inputNames;
    this->outputNames = outputNames;
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
        // cap1.open(2); // Camera 2 for Obstacle Detection
        
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
    processObstacleDetection(frame2,midasSession,inputShape,inputNames,outputNames);
    
    QImage img1(frame1.data, frame1.cols, frame1.rows, frame1.step, QImage::Format_RGB888);
    rgbLabel->setPixmap(QPixmap::fromImage(img1.rgbSwapped()).scaled(rgbLabel->size(), Qt::KeepAspectRatio));
    
    // edgeLabel->setPixmap(QPixmap::fromImage(img2).scaled(edgeLabel->size(), Qt::KeepAspectRatio));
    // QImage img2(frame2.data, frame2.cols, frame2.rows, frame2.step, QImage::Format_RGB888);
    // edgeLabel->setPixmap(QPixmap::fromImage(img2.rgbSwapped()).scaled(edgeLabel->size(), Qt::KeepAspectRatio));

    QImage img2(frame2.data, frame2.cols, frame2.rows, frame2.step, QImage::Format_Grayscale8);
    edgeLabel->setPixmap(QPixmap::fromImage(img2).scaled(edgeLabel->size(), Qt::KeepAspectRatio));
}

void DualFeedGUI::processAruco(cv::Mat &frame) {
    // std::lock_guard<std::mutex> lock(sharedData.mtx);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
    detector.detectMarkers(frame, markerCorners, markerIds, rejected);
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        sharedData->corners = markerCorners.at(0);
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
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs, tvecs, markerLength, 2);
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
        // std::cout << "Marker ID: " << markerIds
        //           << " Translation: " << tvecs
        //           << " Rotation: " << rvecs << std::endl;
        sharedData->x = tvecs[0][0];
        sharedData->y = tvecs[0][1];
        sharedData->z = tvecs[0][2];
        sharedData->roll = rvecs[0][0];
        sharedData->pitch = rvecs[0][1];
        sharedData->yaw = rvecs[0][2];
    }

}

void DualFeedGUI::processObstacleDetection(cv::Mat &frameTwo,Ort::Session* midasSession, std::array<int64_t, 4> &inputShape,std::vector<const char*> &inputNames,std::vector<const char*> &outputNames) {
    cv::Mat resizedFrame;
    cv::Mat frame = frameTwo;
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

    std::vector<Ort::Value> outputTensors = midasSession->Run(
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

    double obstacleThreshold = 5000000.0 * 1.3;

    std::string obstaclePosition;
    if (leftWeight < obstacleThreshold && centerWeight < obstacleThreshold && rightWeight < obstacleThreshold) {
        obstaclePosition = "Obstacle: None";
        sharedData->ObstacleDirection = "None";
        sharedData->obstacleDetected = false;
    } else if (centerWeight > leftWeight && centerWeight > rightWeight) {
        obstaclePosition = "Obstacle: CENTER!";
        sharedData->ObstacleDirection = "CENTER";
        sharedData->obstacleDetected = true;
    } else if (leftWeight > rightWeight) {
        obstaclePosition = "Obstacle: LEFT!";
        sharedData->ObstacleDirection = "LEFT";
        sharedData->obstacleDetected = true;
    } else {
        obstaclePosition = "Obstacle: RIGHT!";
        sharedData->ObstacleDirection = "RIGHT";
        sharedData->obstacleDetected = true;
    }

    // Display result
    cv::resize(depthDisplay,depthDisplay,cv::Size(640,480));
    cv::resize(binaryMask,binaryMask,cv::Size(640,480));

    cv::putText(frame, obstaclePosition, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    cv::putText(depthDisplay, obstaclePosition, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    cv::putText(binaryMask, obstaclePosition, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);

    // cv::imshow("Original", frame);
    // cv::imshow("Depth Estimation", depthDisplay);
    // cv::imshow("Obstacle Detection", binaryMask);

    // frameTwo = frame;
    frameTwo = depthDisplay;
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
    factory.registerNodeType<GoalReached>("GoalReached",sharedData);
    auto tree = factory.createTreeFromFile("./../tree/treeV2.xml");
    
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
    //Temporoary 
    myMap[{0, 1}] = "Left";
    myMap[{0, 2}] = "Right";
    myMap[{1, 2}] = "Right";
    myMap[{1, 3}] = "Left";
    myMap[{2, 3}] = "Right";
    myMap[{3, 4}] = "Left";
    myMap[{4, 5}] = "Left";
    myMap[{5, 6}] = "Left";

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ObstacleDetection");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);

    const std::string modelPath = "/home/vinoth/arboric/obstacleAvoidance/midas_v21_small_256.onnx";
    Ort::Session midasSession(env, modelPath.c_str(), sessionOptions);
    
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr inputName = midasSession.GetInputNameAllocated(0, allocator);
    Ort::AllocatedStringPtr outputName = midasSession.GetOutputNameAllocated(0, allocator);
    
    std::vector<const char*> inputNames = {inputName.get()};
    std::vector<const char*> outputNames = {outputName.get()};

    std::array<int64_t, 4> inputShape = {1, 3, 256, 256};
    
    // Load Camera Calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::string load_path = "./../CamCalibration/camera_parameters.yaml";
    if (loadCameraCalibration(load_path, cameraMatrix, distCoeffs)) {
        // std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
        // std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
    } else {
        // std::cerr << "Failed to load calibration data!" << std::endl;
        return -1;
    }


    int serial_port = openSerialPort("/dev/ttyACM0");
    if (serial_port < 0) {
        return 1;  // Exit if serial port fails
    }
    sharedData->serial_port = serial_port;
    sharedData->MarkerMap = myMap;
    // Start Aruco Processing in a separate QThread
    // ArucoProcessing arucoThread;
    // arucoThread.start();
    std::cout << " ==========  ARBORIC is ON !! =========" << std::endl;
    // Run Behavior Tree in a separate thread
    std::thread btThread(runBehaviorTree, sharedData);
    // Start GUI
    DualFeedGUI window(sharedData, cameraMatrix, distCoeffs, &midasSession,inputShape,inputNames,outputNames );
    window.resize(1280, 480);
    window.show();
    
    int result = app.exec();

    // Cleanup threads
    // arucoThread.quit();
    // arucoThread.wait();
    btThread.join();

    return result;
}