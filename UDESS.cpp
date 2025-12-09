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
int main(){
    return 0;
}
// 3. Shelter Class
