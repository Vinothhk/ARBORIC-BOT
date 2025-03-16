// #include <iostream>
// #include <map>
// #include <string>
// #include <tuple>
// #include <unordered_map>
// #include <vector>
// #include <queue>

// using namespace std;

// struct Node {
//     int marker;
//     string relation;
// };

// // Function to find the relation from start to end using BFS
// void findRelation(int start, int end, map<tuple<int, int>, string>& myMap) {
//     // Step 1: Convert map into bidirectional adjacency list
//     unordered_map<int, vector<pair<int, string>>> adj;
//     for (const auto& it : myMap) {
//         int from = get<0>(it.first);
//         int to = get<1>(it.first);
//         string direction = it.second;
        
//         // Store both directions
//         adj[from].push_back({to, direction});
//         adj[to].push_back({from, direction}); // Reverse connection
//     }

//     // Step 2: BFS to find shortest path
//     queue<Node> q;
//     unordered_map<int, bool> visited;
    
//     q.push({start, ""});
//     visited[start] = true;

//     while (!q.empty()) {
//         Node current = q.front();
//         q.pop();

//         if (current.marker == end) {
//             cout << "Relation from " << start << " to " << end << ": " << current.relation << endl;
//             return;
//         }

//         for (const auto& neighbor : adj[current.marker]) {
//             int nextNode = neighbor.first;
//             string newDirection = neighbor.second;
            
//             // Merge identical consecutive directions
//             string newRelation;
//             if (current.relation.empty() || current.relation == newDirection) {
//                 newRelation = newDirection;
//             } else {
//                 newRelation = current.relation + " → " + newDirection;
//             }

//             if (!visited[nextNode]) {
//                 visited[nextNode] = true;
//                 q.push({nextNode, newRelation});
//             }
//         }
//     }

//     cout << "No relation found between " << start << " and " << end << endl;
// }

// int main() {
//     map<tuple<int, int>, string> myMap;

//     myMap[{1, 2}] = "Right";
//     myMap[{1, 3}] = "Left";
//     myMap[{2, 6}] = "Right";
//     myMap[{3, 4}] = "Left";
//     myMap[{4, 5}] = "Left";
//     myMap[{5, 6}] = "Left";


//     // int start = 6, end = 1;
//     int start,end;
//     cout << "Start: ";
//     cin >> start;
//     cout << "End: ";
//     cin >> end;
//     findRelation(start, end, myMap);

//     return 0;
// }


#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>
#include <queue>

using namespace std;

// Function to get the opposite direction
string getOppositeDirection(const string& direction) {
    if (direction == "Right") return "Left";
    if (direction == "Left") return "Right";
    return direction; // Default case (if other directions exist)
}

struct Node {
    int marker;
    string relation;
};

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
    queue<Node> q;
    unordered_map<int, bool> visited;

    q.push({start, ""});
    visited[start] = true;

    while (!q.empty()) {
        Node current = q.front();
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

int main() {
    // map<pair<int, int>, string> myMap;


    // myMap[{1, 2}] = "Right";
    // myMap[{1, 3}] = "Left";
    // myMap[{2, 6}] = "Right";
    // myMap[{3, 4}] = "Left";
    // myMap[{4, 5}] = "Left";
    // myMap[{5, 6}] = "Left";


    // // int start = 6, end = 1;
    // int start,end;
    // cout << "Start: ";
    // cin >> start;
    // cout << "End: ";
    // cin >> end;
    // findRelation(start, end, myMap);

    // return 0;
    map<pair<int, int>, string> myMap;

    // myMap[{1, 2}] = "Right";
    // myMap[{1, 3}] = "Left";
    // myMap[{2, 4}] = "Right";
    // myMap[{3, 4}] = "Left";
    // myMap[{4, 6}] = "Right";
    // myMap[{6, 1}] = "Left";

    myMap[{1, 2}] = "Right";
    myMap[{1, 3}] = "Left";
    myMap[{2, 6}] = "Right";
    myMap[{3, 4}] = "Left";
    myMap[{4, 5}] = "Left";
    myMap[{5, 6}] = "Left";

    // myMap[{5, 1}] = "Right"; // Added connection: 5 → 1 is Right

    int start, end;
    cout << "Enter start and end markers: ";
    cin >> start >> end;

    findRelation(start, end, myMap);

    return 0;
} 
