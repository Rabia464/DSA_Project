#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <list>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <map>

using namespace std;

// Evacuee Class
class Person {
public:
    int id;
    int age;
    int riskScore;  // Higher = more at risk
    pair<int, int> location;  // (x, y) coordinates
    int targetShelterId;
    bool evacuated;
    
    Person(int id, int age, int riskScore, pair<int, int> loc) 
        : id(id), age(age), riskScore(riskScore), location(loc), 
          targetShelterId(-1), evacuated(false) {}
    
    // For priority queue (higher risk = higher priority)
    bool operator<(const Person& other) const {
        return riskScore < other.riskScore;  // Lower risk score = lower priority
    }
};

// 2. Building Class
class Building {
public:
    int id;
    string name;
    pair<int, int> location;
    int capacity;
    int currentOccupancy;
    bool isBlocked;
    int disasterSeverity;  // 0-10 scale
    
    Building(int id, string name, pair<int, int> loc, int cap) 
        : id(id), name(name), location(loc), capacity(cap), 
          currentOccupancy(0), isBlocked(false), disasterSeverity(0) {}
};
// 3 shelter class
class Shelter {
public:
    int id;
    string name;
    pair<int, int> location;
    int capacity;
    queue<Person> waitingQueue;  // Queue for incoming people
    vector<Person> acceptedPeople;  // People inside shelter
    ofstream logFile;
    
    Shelter(int id, string name, pair<int, int> loc, int cap) 
        : id(id), name(name), location(loc), capacity(cap) {
        // Open log file for this shelter
        logFile.open("shelter_" + to_string(id) + "_log.txt", ios::app);
    }
    
    ~Shelter() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }
    
    bool hasSpace() {
        return acceptedPeople.size() < capacity;
    }
    bool hasSpace() {
        return acceptedPeople.size() < capacity;
    }
    
    void addPerson(Person& person) {
        if (hasSpace()) {
            acceptedPeople.push_back(person);
            person.evacuated = true;
            logFile << "Person " << person.id << " (Age: " << person.age 
                   << ", Risk: " << person.riskScore << " entered shelter at "
                   << getCurrentTime() << endl;
        } else {
            waitingQueue.push(person);
            logFile << "Person " << person.id << " added to waiting queue at "
                   << getCurrentTime() << endl;
        }
    }
    
    string getCurrentTime() {
        time_t now = time(0);
        char* dt = ctime(&now);
        string timeStr(dt);
        timeStr.pop_back();  // Remove newline
        return timeStr;
    }
};
class Road {
    public:
    pair<int,int> start;
    pair<int,int> end;
    bool isblocked;
    int distance;
    Road(pair<int,int> s, pair<int,int> e, int dist)
    : start(s),end(e), isblocked(false),distance(dist){}
};
class disasterNode{
    public:
    int buildingID;
    int severity;
    disasterNode* left;
     disasterNode* right;
      disasterNode(int bid, int sev) 
        : buildingID(bid), severity(sev), left(nullptr), right(nullptr) {}
};
// 6. BST for Disaster Severity
class DisasterTree {
private:
    disasterNode* root;
    
    disasterNode* insert(disasterNode* node, int buildingId, int severity) {
        if (node == nullptr) {
            return new disasterNode(buildingId, severity);
        }
        
        if (severity < node->severity) {
            node->left = insert(node->left, buildingId, severity);
        } else {
            node->right = insert(node->right, buildingId, severity);
        }
        
        return node;
    }
    
    void inOrderTraversal(disasterNode* node, vector<pair<int, int>>& result) {
        if (node != nullptr) {
            inOrderTraversal(node->left, result);
            result.push_back({node->buildingID, node->severity});
            inOrderTraversal(node->right, result);
        }
    }
    
public:
    DisasterTree() : root(nullptr) {}
    
    void insert(int buildingId, int severity) {
        root = insert(root, buildingId, severity);
    }
    
    vector<pair<int, int>> getAllDisasters() {
        vector<pair<int, int>> result;
        inOrderTraversal(root, result);
        return result;
    }
    
    int getSeverity(int buildingId) {
        // Search for building in tree
        vector<pair<int, int>> disasters = getAllDisasters();
        for (auto& d : disasters) {
            if (d.first == buildingId) {
                return d.second;
            }
        }
        return 0;
    }
};
// 7. Linked List Node for Movement History
class MovementNode {
public:
    int personId;
    pair<int, int> from;
    pair<int, int> to;
    string timestamp;
    MovementNode* next;
    
    MovementNode(int pid, pair<int, int> f, pair<int, int> t, string ts) 
        : personId(pid), from(f), to(t), timestamp(ts), next(nullptr) {}
};

// 8. Linked List for Movement History
class MovementHistory {
private:
    MovementNode* head;
    
public:
    MovementHistory() : head(nullptr) {}
    
    void addMovement(int personId, pair<int, int> from, pair<int, int> to) {
        time_t now = time(0);
        char* dt = ctime(&now);
        string timeStr(dt);
        timeStr.pop_back();
        
        MovementNode* newNode = new MovementNode(personId, from, to, timeStr);
        newNode->next = head;
        head = newNode;
    }
    
    vector<MovementNode*> getPersonHistory(int personId) {
        vector<MovementNode*> history;
        MovementNode* current = head;
        while (current != nullptr) {
            if (current->personId == personId) {
                history.push_back(current);
            }
            current = current->next;
        }
        return history;
    }
    
    void printHistory(int personId) {
        vector<MovementNode*> history = getPersonHistory(personId);
        cout << "Movement History for Person " << personId << ":\n";
        for (auto* node : history) {
            cout << "  From (" << node->from.first << "," << node->from.second 
                 << ") to (" << node->to.first << "," << node->to.second 
                 << ") at " << node->timestamp << endl;
        }
    }
};

int main(){
    return 0;
}

