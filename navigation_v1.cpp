#include "behaviortree_cpp/bt_factory.h"
#include <iostream>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>

using namespace std::chrono_literals;
using namespace std;

// Custom BT Nodes
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
    WaitForRequest(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<int>("status")};
    }
    BT::NodeStatus tick() override {
        std::cout << "Waiting for request from GUI...\n";
        std :: cout <<  "Press key 's': " << std::endl;
        std :: cin >> o;
        while (o == 's') {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return BT::NodeStatus::SUCCESS;
    }
}
private:
    char o;
};

class MarkerFound : public BT::ConditionNode {
public:
    MarkerFound(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<int>("marker_id")};
    }

    BT::NodeStatus tick() override {
        int marker_id = 5;
        BT::TreeNode::setOutput("marker_id",marker_id);
        bool found = (rand() % 2 == 0);  // Simulating marker detection (random true/false)
        std::cout << "Marker " << marker_id << (found ? " found!\n" : " not found.\n");
        return found ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class SpinSearch : public BT::SyncActionNode {
public:
    SpinSearch(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Spinning to search for marker...\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveSlightly : public BT::SyncActionNode {
public:
    MoveSlightly(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Moving slightly to reposition for marker detection...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return BT::NodeStatus::SUCCESS;
    }
};

class StoreMarkerRelation : public BT::SyncActionNode {
public:
    StoreMarkerRelation(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Storing marker relation...\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class ApproachMarker : public BT::SyncActionNode {
public:
    ApproachMarker(const std::string& name, const BT::NodeConfiguration& config, int serial_port) 
        : BT::SyncActionNode(name, config), serial_port_(serial_port) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("marker_id") };
    }

    BT::NodeStatus tick() override {
        int marker_id;
        if (!getInput("marker_id", marker_id)) {
            std::cerr << "Missing marker_id input!\n";
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "Approaching marker " << marker_id << "...\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::string command = std::to_string(marker_id) + "\n";
        for (int i = 0; i < 10; i++) {
            cout<<"Sending command.. "<<endl;
            write(serial_port_, command.c_str(), command.size());
            this_thread::sleep_for(1000ms);
        }

        return BT::NodeStatus::SUCCESS;
    }
private:
    int serial_port_;
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

int main() {
    int serial_port = openSerialPort("/dev/ttyACM0");
    if (serial_port < 0) {
        return 1;  // Exit if serial port fails
    }

    BT::BehaviorTreeFactory factory;

    // Register custom nodes 
    factory.registerNodeType<WhileTrueNode>("WhileTrue");
    factory.registerNodeType<WaitForRequest>("WaitForRequest");
    factory.registerNodeType<MarkerFound>("MarkerFound");
    factory.registerNodeType<SpinSearch>("SpinSearch");
    factory.registerNodeType<MoveSlightly>("MoveSlightly");
    factory.registerNodeType<StoreMarkerRelation>("StoreMarkerRelation");
    factory.registerNodeType<ApproachMarker>("ApproachMarker",serial_port);
    factory.registerNodeType<AvoidObstacles>("AvoidObstacles");
    factory.registerNodeType<FollowPath>("FollowPath");

    // Load the tree from XML
    auto tree = factory.createTreeFromFile("./../tree/treeV1.xml");

    // Execute the tree
    std::cout << "Starting Behavior Tree Execution...\n";
    tree.tickWhileRunning();
    // while (tree.tickRoot() == BT::NodeStatus::RUNNING) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
    // std::cout << "Behavior Tree Execution Finished.\n";

    return 0;
}
