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
#include <climits>

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
    
    // Move constructor
    Shelter(Shelter&& other) noexcept
        : id(other.id), name(move(other.name)), location(other.location), 
          capacity(other.capacity), waitingQueue(move(other.waitingQueue)),
          acceptedPeople(move(other.acceptedPeople)) {
        logFile.open("shelter_" + to_string(id) + "_log.txt", ios::app);
    }
    
    // Move assignment operator
    Shelter& operator=(Shelter&& other) noexcept {
        if (this != &other) {
            if (logFile.is_open()) {
                logFile.close();
            }
            id = other.id;
            name = move(other.name);
            location = other.location;
            capacity = other.capacity;
            waitingQueue = move(other.waitingQueue);
            acceptedPeople = move(other.acceptedPeople);
            logFile.open("shelter_" + to_string(id) + "_log.txt", ios::app);
        }
        return *this;
    }
    
    // Delete copy constructor and copy assignment
    Shelter(const Shelter&) = delete;
    Shelter& operator=(const Shelter&) = delete;
    
    ~Shelter() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }
    
    bool hasSpace() {
        return acceptedPeople.size() < static_cast<size_t>(capacity);
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
            return "NO actions to undo";
        }
        string lastAction= undoStack.top();
        undoStack.pop();
        return lastAction;
    }
    bool isEmpty(){
        return undoStack.empty();
    }

};

// Graph class for city road network (Adjacency List representation)
class CityGraph {
private:
    // Adjacency list: map<vertex, vector<{neighbor, weight}>>
    map<pair<int, int>, vector<pair<pair<int, int>, int>>> adjacencyList;
    map<pair<pair<int, int>, pair<int, int>>, bool> blockedEdges;  // Track blocked roads
    
public:
    CityGraph() {}
    
    // Add an edge (road) to the graph
    void addEdge(pair<int, int> from, pair<int, int> to, int weight) {
        adjacencyList[from].push_back({to, weight});
        adjacencyList[to].push_back({from, weight});  // Undirected graph
    }
    
    // Remove/block an edge
    void blockEdge(pair<int, int> from, pair<int, int> to) {
        blockedEdges[{from, to}] = true;
        blockedEdges[{to, from}] = true;
    }
    
    // Unblock an edge
    void unblockEdge(pair<int, int> from, pair<int, int> to) {
        blockedEdges.erase({from, to});
        blockedEdges.erase({to, from});
    }
    
    // Check if edge is blocked
    bool isEdgeBlocked(pair<int, int> from, pair<int, int> to) {
        return blockedEdges.find({from, to}) != blockedEdges.end();
    }
    
    // Get neighbors of a vertex (only unblocked edges)
    vector<pair<pair<int, int>, int>> getNeighbors(pair<int, int> vertex) {
        vector<pair<pair<int, int>, int>> neighbors;
        if (adjacencyList.find(vertex) != adjacencyList.end()) {
            for (const auto& neighbor : adjacencyList[vertex]) {
                if (!isEdgeBlocked(vertex, neighbor.first)) {
                    neighbors.push_back(neighbor);
                }
            }
        }
        return neighbors;
    }
    
    // Check if vertex exists in graph
    bool hasVertex(pair<int, int> vertex) {
        return adjacencyList.find(vertex) != adjacencyList.end();
    }
    
    // BFS to find shortest path from start to end
    vector<pair<int, int>> shortestPath(pair<int, int> start, pair<int, int> end) {
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
            
            // Get neighbors from graph
            vector<pair<pair<int, int>, int>> neighbors = getNeighbors(current);
            for (const auto& neighbor : neighbors) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = current;
                    q.push(neighbor.first);
                }
            }
            
            // Fallback: Allow direct movement if vertex not in graph (for off-road movement)
            if (!hasVertex(current)) {
                vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
                for (const auto& dir : directions) {
                    pair<int, int> next = {current.first + dir.first, current.second + dir.second};
                    if (!visited[next]) {
                        visited[next] = true;
                        parent[next] = current;
                        q.push(next);
                    }
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
    
    // Get all vertices in the graph
    vector<pair<int, int>> getAllVertices() {
        vector<pair<int, int>> vertices;
        for (const auto& entry : adjacencyList) {
            vertices.push_back(entry.first);
        }
        return vertices;
    }
    
    // Get number of edges
    int getEdgeCount() {
        int count = 0;
        for (const auto& entry : adjacencyList) {
            count += entry.second.size();
        }
        return count / 2;  // Divide by 2 for undirected graph
    }
    
    // Print graph structure (for debugging)
    void printGraph() {
        cout << "\n========== CITY ROAD NETWORK GRAPH ==========\n";
        cout << "Total vertices: " << adjacencyList.size() << endl;
        cout << "Total edges: " << getEdgeCount() << endl;
        cout << "\nAdjacency List:\n";
        for (const auto& entry : adjacencyList) {
            cout << "  (" << entry.first.first << "," << entry.first.second << ") -> ";
            for (const auto& neighbor : entry.second) {
                if (!isEdgeBlocked(entry.first, neighbor.first)) {
                    cout << "(" << neighbor.first.first << "," << neighbor.first.second 
                         << ", w:" << neighbor.second << ") ";
                }
            }
            cout << endl;
        }
        cout << "==========================================\n";
    }
};

// City Visualizer Class - ASCII Art Visualization
class CityVisualizer {
private:
    static const int GRID_SIZE = 50;  // Grid size for visualization
    char grid[GRID_SIZE][GRID_SIZE];
    int minX, maxX, minY, maxY;
    
    void initializeGrid() {
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                grid[i][j] = ' ';  // Empty space
            }
        }
    }
    
    void normalizeCoordinates(int& x, int& y) {
        // Scale coordinates to fit in grid
        if (maxX - minX > 0) {
            x = ((x - minX) * (GRID_SIZE - 1)) / (maxX - minX);
        }
        if (maxY - minY > 0) {
            y = ((y - minY) * (GRID_SIZE - 1)) / (maxY - minY);
        }
        // Clamp to grid bounds
        x = std::max(0, std::min(GRID_SIZE - 1, x));
        y = std::max(0, std::min(GRID_SIZE - 1, y));
    }
    
public:
    CityVisualizer() {
        initializeGrid();
        minX = minY = 0;
        maxX = maxY = 100;
    }
    
    void calculateBounds(const vector<Building>& buildings, 
                        const vector<Shelter>& shelters,
                        const vector<Person>& people) {
        if (buildings.empty() && shelters.empty() && people.empty()) {
            minX = minY = 0;
            maxX = maxY = 100;
            return;
        }
        
        minX = minY = INT_MAX;
        maxX = maxY = INT_MIN;
        
        for (const auto& b : buildings) {
            minX = std::min(minX, b.location.first);
            maxX = std::max(maxX, b.location.first);
            minY = std::min(minY, b.location.second);
            maxY = std::max(maxY, b.location.second);
        }
        
        for (const auto& s : shelters) {
            minX = std::min(minX, s.location.first);
            maxX = std::max(maxX, s.location.first);
            minY = std::min(minY, s.location.second);
            maxY = std::max(maxY, s.location.second);
        }
        
        for (const auto& p : people) {
            minX = std::min(minX, p.location.first);
            maxX = std::max(maxX, p.location.first);
            minY = std::min(minY, p.location.second);
            maxY = std::max(maxY, p.location.second);
        }
        
        // Add padding
        int padding = 10;
        minX -= padding;
        maxX += padding;
        minY -= padding;
        maxY += padding;
    }
    
    void drawRoads(const vector<Road>& roads) {
        for (const auto& road : roads) {
            if (road.isblocked) continue;  // Skip blocked roads
            
            int x1 = road.start.first;
            int y1 = road.start.second;
            int x2 = road.end.first;
            int y2 = road.end.second;
            
            normalizeCoordinates(x1, y1);
            normalizeCoordinates(x2, y2);
            
            // Draw line using Bresenham-like algorithm
            int dx = abs(x2 - x1);
            int dy = abs(y2 - y1);
            int sx = (x1 < x2) ? 1 : -1;
            int sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy;
            
            int x = x1, y = y1;
            while (true) {
                if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
                    if (grid[y][x] == ' ') {
                        grid[y][x] = '.';  // Road
                    }
                }
                
                if (x == x2 && y == y2) break;
                
                int e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y += sy;
                }
            }
        }
    }
    
    void drawBuildings(const vector<Building>& buildings) {
        for (const auto& building : buildings) {
            int x = building.location.first;
            int y = building.location.second;
            normalizeCoordinates(x, y);
            
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
                if (building.disasterSeverity >= 7) {
                    grid[y][x] = '!';  // High danger building
                } else if (building.disasterSeverity > 0) {
                    grid[y][x] = 'B';  // Building with disaster
                } else {
                    grid[y][x] = 'b';  // Normal building
                }
            }
        }
    }
    
    void drawShelters(const vector<Shelter>& shelters) {
        for (const auto& shelter : shelters) {
            int x = shelter.location.first;
            int y = shelter.location.second;
            normalizeCoordinates(x, y);
            
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
                grid[y][x] = 'S';  // Shelter
            }
        }
    }
    
    void drawPeople(const vector<Person>& people) {
        for (const auto& person : people) {
            int x = person.location.first;
            int y = person.location.second;
            normalizeCoordinates(x, y);
            
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
                if (person.evacuated) {
                    grid[y][x] = 'E';  // Evacuated person
                } else if (person.riskScore >= 8) {
                    grid[y][x] = 'P';  // High risk person
                } else {
                    grid[y][x] = 'p';  // Normal person
                }
            }
        }
    }
    
    void drawPath(const vector<pair<int, int>>& path, char pathChar = '*') {
        for (const auto& point : path) {
            int x = point.first;
            int y = point.second;
            normalizeCoordinates(x, y);
            
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
                if (grid[y][x] == ' ' || grid[y][x] == '.') {
                    grid[y][x] = pathChar;
                }
            }
        }
    }
    
    void visualize(const vector<Building>& buildings,
                   const vector<Shelter>& shelters,
                   const vector<Road>& roads,
                   const vector<Person>& people,
                   const string& title = "CITY MAP") {
        initializeGrid();
        calculateBounds(buildings, shelters, people);
        
        // Draw in order: roads -> buildings -> shelters -> people
        drawRoads(roads);
        drawBuildings(buildings);
        drawShelters(shelters);
        drawPeople(people);
        
        // Print the grid
        cout << "\n";
        cout << "========================================\n";
        cout << "  " << title << "\n";
        cout << "========================================\n";
        cout << "\nLegend:\n";
        cout << "  S = Shelter          b = Normal Building\n";
        cout << "  B = Building (Risk)  ! = High Danger Building\n";
        cout << "  p = Person           P = High Risk Person\n";
        cout << "  E = Evacuated        . = Road\n";
        cout << "  * = Evacuation Path\n";
        cout << "\n";
        
        // Print grid with borders
        cout << "  ";
        for (int i = 0; i < GRID_SIZE; i++) {
            if (i % 10 == 0) cout << (i / 10);
            else cout << " ";
        }
        cout << "\n  ";
        for (int i = 0; i < GRID_SIZE; i++) {
            cout << (i % 10);
        }
        cout << "\n";
        
        for (int i = 0; i < GRID_SIZE; i++) {
            cout << setw(2) << (i % 10);
            for (int j = 0; j < GRID_SIZE; j++) {
                cout << grid[i][j];
            }
            cout << " " << (i % 10) << "\n";
        }
        
        cout << "  ";
        for (int i = 0; i < GRID_SIZE; i++) {
            if (i % 10 == 0) cout << (i / 10);
            else cout << " ";
        }
        cout << "\n  ";
        for (int i = 0; i < GRID_SIZE; i++) {
            cout << (i % 10);
        }
        cout << "\n";
        
        cout << "\nCoordinate Range: X[" << minX << "-" << maxX 
             << "] Y[" << minY << "-" << maxY << "]\n";
        cout << "========================================\n\n";
    }
    
    void visualizeEvacuationPath(int personId, 
                                const vector<pair<int, int>>& path,
                                const vector<Building>& buildings,
                                const vector<Shelter>& shelters,
                                const vector<Road>& roads) {
        initializeGrid();
        calculateBounds(buildings, shelters, vector<Person>());
        
        drawRoads(roads);
        drawBuildings(buildings);
        drawShelters(shelters);
        drawPath(path, '*');
        
        cout << "\n========================================\n";
        cout << "  EVACUATION PATH FOR PERSON " << personId << "\n";
        cout << "========================================\n";
        cout << "Path length: " << path.size() << " steps\n";
        cout << "From: (" << path[0].first << "," << path[0].second << ")\n";
        cout << "To: (" << path.back().first << "," << path.back().second << ")\n\n";
        
        // Print grid
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                cout << grid[i][j];
            }
            cout << "\n";
        }
        cout << "========================================\n\n";
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
    CityGraph roadNetwork;  // Graph representation of city roads
    CityVisualizer visualizer;  // Visualization system
    
    // BFS for shortest path (uses Graph class)
    vector<pair<int, int>> bfsShortestPath(pair<int, int> start, pair<int, int> end) {
        return roadNetwork.shortestPath(start, end);
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
                shelters.emplace_back(id, name, make_pair(x, y), cap);
            }
            else if (type == "ROAD") {
                int x1, y1, x2, y2, dist;
                ss >> x1 >> y1 >> x2 >> y2 >> dist;
                Road road({x1, y1}, {x2, y2}, dist);
                roads.push_back(road);
                // Add edge to graph (only if not blocked)
                roadNetwork.addEdge({x1, y1}, {x2, y2}, dist);
                if (road.isblocked) {
                    roadNetwork.blockEdge({x1, y1}, {x2, y2});
                }
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
    
    // Graph-related methods
    void printRoadNetwork() {
        roadNetwork.printGraph();
    }
    
    void blockRoad(pair<int, int> from, pair<int, int> to) {
        roadNetwork.blockEdge(from, to);
        // Also update the Road object
        for (auto& road : roads) {
            if ((road.start == from && road.end == to) || 
                (road.start == to && road.end == from)) {
                road.isblocked = true;
                break;
            }
        }
        undoStack.pushAction("Blocked road from (" + to_string(from.first) + "," + 
                           to_string(from.second) + ") to (" + to_string(to.first) + 
                           "," + to_string(to.second) + ")");
    }
    
    void unblockRoad(pair<int, int> from, pair<int, int> to) {
        roadNetwork.unblockEdge(from, to);
        // Also update the Road object
        for (auto& road : roads) {
            if ((road.start == from && road.end == to) || 
                (road.start == to && road.end == from)) {
                road.isblocked = false;
                break;
            }
        }
        undoStack.pushAction("Unblocked road from (" + to_string(from.first) + "," + 
                           to_string(from.second) + ") to (" + to_string(to.first) + 
                           "," + to_string(to.second) + ")");
    }
    
    // Get graph statistics
    void printGraphStatistics() {
        cout << "\n========== ROAD NETWORK STATISTICS ==========\n";
        cout << "Total vertices (intersections): " << roadNetwork.getAllVertices().size() << endl;
        cout << "Total edges (roads): " << roadNetwork.getEdgeCount() << endl;
        cout << "==========================================\n";
    }
    
    // Visualization methods
    void visualizeCity(const string& title = "CITY MAP - INITIAL STATE") {
        visualizer.visualize(buildings, shelters, roads, people, title);
    }
    
    void visualizeEvacuationPath(int personId) {
        // Find person and their path to shelter
        Person* targetPerson = nullptr;
        for (auto& p : people) {
            if (p.id == personId) {
                targetPerson = &p;
                break;
            }
        }
        
        if (!targetPerson) {
            cout << "Person " << personId << " not found!\n";
            return;
        }
        
        // Find target shelter
        Shelter* targetShelter = nullptr;
        for (auto& s : shelters) {
            if (s.id == targetPerson->targetShelterId) {
                targetShelter = &s;
                break;
            }
        }
        
        if (!targetShelter) {
            cout << "Shelter not found for person " << personId << "!\n";
            return;
        }
        
        // Get path
        vector<pair<int, int>> path = bfsShortestPath(
            targetPerson->location, targetShelter->location);
        
        visualizer.visualizeEvacuationPath(personId, path, 
                                          buildings, shelters, roads);
    }
    
    void visualizeCurrentState(const string& title = "CITY MAP - CURRENT STATE") {
        visualizer.visualize(buildings, shelters, roads, people, title);
    }
};
int main(){
    CitySystem city;
    
    cout << "========================================\n";
    cout << "  UDESS - Urban Disaster Evacuation\n";
    cout << "      Simulation System\n";
    cout << "========================================\n\n";
    
    // Create sample city map file if it doesn't exist
    ofstream mapFile("city_map.txt");
    if (mapFile.is_open()) {
        mapFile << "# City Map File\n";
        mapFile << "# Format: TYPE ID NAME X Y CAPACITY\n";
        mapFile << "BUILDING 1 Office1 10 20 50\n";
        mapFile << "BUILDING 2 Office2 30 40 30\n";
        mapFile << "BUILDING 3 Apartment1 50 60 100\n";
        mapFile << "SHELTER 1 SafeZone1 100 100 200\n";
        mapFile << "SHELTER 2 SafeZone2 200 200 150\n";
        mapFile << "ROAD 10 20 30 40 20\n";
        mapFile << "ROAD 30 40 50 60 25\n";
        mapFile.close();
    }
    
    // Create sample disaster file
    ofstream disasterFile("disaster_severity.txt");
    if (disasterFile.is_open()) {
        disasterFile << "# Disaster Severity File\n";
        disasterFile << "# Format: BUILDING_ID SEVERITY (0-10)\n";
        disasterFile << "1 8\n";
        disasterFile << "2 5\n";
        disasterFile << "3 9\n";
        disasterFile.close();
    }
    
    // Load city map
    city.loadCityMap("city_map.txt");
    
    // Load disaster severity
    city.loadDisasterSeverity("disaster_severity.txt");
    
    // Display graph structure (NEW: Graph data structure demonstration)
    city.printGraphStatistics();
    city.printRoadNetwork();
    
    // Add some people
    city.addPerson(1, 25, 9, {10, 20});  // High risk person at building 1
    city.addPerson(2, 45, 6, {30, 40});  // Medium risk person at building 2
    city.addPerson(3, 30, 8, {50, 60});  // High risk person at building 3
    city.addPerson(4, 60, 10, {10, 20}); // Very high risk (elderly)
    
    // VISUALIZATION: Show initial state
    city.visualizeCity("CITY MAP - INITIAL STATE (Before Evacuation)");
    
    // Sort dangerous buildings
    city.sortDangerousBuildings();
    
    // Run evacuation
    city.runEvacuation();
    
    // VISUALIZATION: Show state after evacuation
    city.visualizeCurrentState("CITY MAP - AFTER EVACUATION");
    
    // VISUALIZATION: Show evacuation path for person 1
    city.visualizeEvacuationPath(1);
    
    // Print statistics
    city.printStatistics();
    
    // Show movement history for person 1
    cout << "\n";
    city.printPersonHistory(1);
    
    cout << "\nSimulation complete! Check evacuation_report.txt for detailed logs.\n";
    
    return 0;
}

