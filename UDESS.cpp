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
class UndoStack{
    private:
        stack<string> undoStack;
    public:
        void pushAction(string action){
        undoStack.push(action);
    }
    string undo(){
        if (undoStack.empty()){
            return "NO actipns to undo";
        }
        string lastAction= undoStack.top();
        undoStack.pop();
        return lastAction;
    }
    bool isEmpty(){
        return undoStack.empty();
    }

};

class CitySystem {
private:
    vector<Building> buildings;
    vector<Shelter> shelters;
    vector<Road> roads;
    vector<Person> people;
    priority_queue<Person> evacuationQueue;  // Priority queue for evacuation
    DisasterTree disasterTree;
    MovementHistory movementHistory;
    UndoStack undoStack;
    ofstream mainLogFile;
     // BFS for shortest path (considers blocked roads)
    vector<pair<int, int>> bfsShortestPath(pair<int, int> start, pair<int, int> end) {
        // Build adjacency list from roads
        map<pair<int, int>, vector<pair<pair<int, int>, int>>> graph;
        
        for (const auto& road : roads) {
            if (!road.isblocked) {  // Only use unblocked roads
                graph[road.start].push_back({road.end, road.distance});
                graph[road.end].push_back({road.start, road.distance});
            }
        }
        
        // BFS to find shortest path
        queue<pair<int, int>> q;
        map<pair<int, int>, pair<int, int>> parent;  // For path reconstruction
        map<pair<int, int>, bool> visited;
        
        q.push(start);
        visited[start] = true;
        parent[start] = {-1, -1};  // Sentinel value
        
        bool found = false;
        
        while (!q.empty() && !found) {
            pair<int, int> current = q.front();
            q.pop();
            
            if (current == end) {
                found = true;
                break;
            }
            
            // Check direct neighbors from roads
            if (graph.find(current) != graph.end()) {
                for (const auto& neighbor : graph[current]) {
                    if (!visited[neighbor.first]) {
                        visited[neighbor.first] = true;
                        parent[neighbor.first] = current;
                        q.push(neighbor.first);
                    }
                }
            }
            
            // Also allow direct movement if no road network (fallback)
            // This handles cases where start/end aren't on road network
            vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
            for (const auto& dir : directions) {
                pair<int, int> next = {current.first + dir.first, current.second + dir.second};
                if (!visited[next] && graph.find(next) == graph.end()) {
                    // Allow direct movement if not on road network
                    visited[next] = true;
                    parent[next] = current;
                    q.push(next);
                }
            }
        }
        
        // Reconstruct path
        vector<pair<int, int>> path;
        if (found) {
            pair<int, int> current = end;
            while (current != make_pair(-1, -1)) {
                path.push_back(current);
                current = parent[current];
            }
            reverse(path.begin(), path.end());
        } else {
            // If no path found, return direct path (fallback)
            path.push_back(start);
            path.push_back(end);
        }
        
        return path;
    }
    
    // Calculate distance between two points
    int calculateDistance(pair<int, int> a, pair<int, int> b) {
        return abs(a.first - b.first) + abs(a.second - b.second);
    }
    
public:
    CitySystem() {
        mainLogFile.open("evacuation_report.txt", ios::app);
    }
    
    ~CitySystem() {
        if (mainLogFile.is_open()) {
            mainLogFile.close();
        }
    }
    
    // 1. Load city map from file
    bool loadCityMap(string filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Error: Could not open file " << filename << endl;
            return false;
        }
        
        string line;
        while (getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            stringstream ss(line);
            string type;
            ss >> type;
            
            if (type == "BUILDING") {
                int id, x, y, cap;
                string name;
                ss >> id >> name >> x >> y >> cap;
                buildings.push_back(Building(id, name, {x, y}, cap));
            }
            else if (type == "SHELTER") {
                int id, x, y, cap;
                string name;
                ss >> id >> name >> x >> y >> cap;
                shelters.push_back(Shelter(id, name, {x, y}, cap));
            }
            else if (type == "ROAD") {
                int x1, y1, x2, y2, dist;
                ss >> x1 >> y1 >> x2 >> y2 >> dist;
                roads.push_back(Road({x1, y1}, {x2, y2}, dist));
            }
        }
        
        file.close();
        undoStack.pushAction("Loaded city map from " + filename);
        cout << "City map loaded successfully!" << endl;
        return true;
    }
    
    // 2. Load disaster severity tree
    bool loadDisasterSeverity(string filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Error: Could not open file " << filename << endl;
            return false;
        }
        
        string line;
        while (getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            stringstream ss(line);
            int buildingId, severity;
            ss >> buildingId >> severity;
            disasterTree.insert(buildingId, severity);
            
            // Update building disaster severity
            for (auto& building : buildings) {
                if (building.id == buildingId) {
                    building.disasterSeverity = severity;
                    building.isBlocked = (severity >= 7);
                }
            }
        }
        
        file.close();
        undoStack.pushAction("Loaded disaster severity data");
        cout << "Disaster severity data loaded!" << endl;
        return true;
    }
    
    // 3. Add person to simulation
    void addPerson(int id, int age, int riskScore, pair<int, int> location) {
        Person person(id, age, riskScore, location);
        people.push_back(person);
        evacuationQueue.push(person);
        undoStack.pushAction("Added person " + to_string(id));
        cout << "Person " << id << " added to simulation" << endl;
    }
    
    // 4. Find nearest shelter for a person
    int findNearestShelter(pair<int, int> location) {
        if (shelters.empty()) return -1;
        
        int nearestId = shelters[0].id;
        int minDistance = calculateDistance(location, shelters[0].location);
        
        for (auto& shelter : shelters) {
            int dist = calculateDistance(location, shelter.location);
            if (dist < minDistance) {
                minDistance = dist;
                nearestId = shelter.id;
            }
        }
        
        return nearestId;
    }
    void runEvacuation() {
        mainLogFile << "\n========== EVACUATION SIMULATION STARTED ==========\n";
        mainLogFile << "Time: " << getCurrentTime() << "\n\n";
        
        priority_queue<Person> tempQueue = evacuationQueue;
        
        while (!tempQueue.empty()) {
            Person person = tempQueue.top();
            tempQueue.pop();
            
            if (person.evacuated) continue;
            
            // Find nearest shelter
            int shelterId = findNearestShelter(person.location);
            if (shelterId == -1) {
                cout << "No shelters available for person " << person.id << endl;
                continue;
            }
            
            // Find shelter
            Shelter* shelter = nullptr;
            for (auto& s : shelters) {
                if (s.id == shelterId) {
                    shelter = &s;
                    break;
                }
            }
            
            if (shelter) {
                // Calculate path using BFS
                pair<int, int> oldLocation = person.location;
                vector<pair<int, int>> path = bfsShortestPath(person.location, shelter->location);
                
                // Track each step in the path
                for (size_t i = 1; i < path.size(); i++) {
                    movementHistory.addMovement(person.id, path[i-1], path[i]);
                }
                
                person.location = shelter->location;
                person.targetShelterId = shelterId;
                
                // Add to shelter
                shelter->addPerson(person);
                
                // Update person in main list
                for (auto& p : people) {
                    if (p.id == person.id) {
                        p = person;
                        break;
                    }
                }
                
                mainLogFile << "Person " << person.id << " evacuated to Shelter " 
                           << shelterId << " (Risk: " << person.riskScore 
                           << ", Path length: " << path.size() << " steps)\n";
            }
        }
        
        mainLogFile << "\n========== EVACUATION SIMULATION COMPLETED ==========\n\n";
        cout << "Evacuation simulation completed!" << endl;
    }
    
    // 6. Sort dangerous buildings
    void sortDangerousBuildings() {
        sort(buildings.begin(), buildings.end(), 
             [](const Building& a, const Building& b) {
                 return a.disasterSeverity > b.disasterSeverity;
             });
        
        cout << "\nDangerous Buildings (sorted by severity):\n";
        for (const auto& building : buildings) {
            if (building.disasterSeverity > 0) {
                cout << "  " << building.name << " (ID: " << building.id 
                     << ") - Severity: " << building.disasterSeverity << endl;
            }
        }
    }
    
    // 7. Print statistics
    void printStatistics() {
        cout << "\n========== SIMULATION STATISTICS ==========\n";
        cout << "Total Buildings: " << buildings.size() << endl;
        cout << "Total Shelters: " << shelters.size() << endl;
        cout << "Total People: " << people.size() << endl;
        
        int evacuated = 0;
        for (const auto& person : people) {
            if (person.evacuated) evacuated++;
        }
        
        cout << "Evacuated: " << evacuated << endl;
        cout << "Remaining: " << (people.size() - evacuated) << endl;
        
        cout << "\nShelter Status:\n";
        for (const auto& shelter : shelters) {
            cout << "  Shelter " << shelter.id << " (" << shelter.name 
                 << "): " << shelter.acceptedPeople.size() << "/" 
                 << shelter.capacity << " capacity\n";
        }
    }
    
    // 8. Undo last action
    void undo() {
        string action = undoStack.undo();
        cout << "Undone: " << action << endl;
    }
    
    string getCurrentTime() {
        time_t now = time(0);
        char* dt = ctime(&now);
        string timeStr(dt);
        timeStr.pop_back();
        return timeStr;
    }
    
    // Getter methods for testing
    vector<Building>& getBuildings() { return buildings; }
    vector<Shelter>& getShelters() { return shelters; }
    vector<Person>& getPeople() { return people; }
    
    // Access movement history
    void printPersonHistory(int personId) {
        movementHistory.printHistory(personId);
    }
};
int main(){
    cout<<"hello";
    return 0;
}

