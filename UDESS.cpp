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

int main(){
    return 0;
}

