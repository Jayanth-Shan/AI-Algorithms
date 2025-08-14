#include <iostream>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <algorithm>
using namespace std;

bool depthFirstRecursion(unordered_map<char, vector<char>>& adjacencyList, char currentVertex, char destination, vector<char>& pathTrace, unordered_set<char>& exploredNodes) {
    if (currentVertex == destination) {
        for (char vertex : pathTrace) {
            cout << vertex << " ";
        }
        cout << endl;
        return true;
    }
    
    vector<char> adjacentVertices = adjacencyList[currentVertex];
    sort(adjacentVertices.begin(), adjacentVertices.end());
    
    for (char nextVertex : adjacentVertices) {
        if (exploredNodes.find(nextVertex) == exploredNodes.end()) {
            exploredNodes.insert(nextVertex);
            pathTrace.push_back(nextVertex);
            
            if (depthFirstRecursion(adjacencyList, nextVertex, destination, pathTrace, exploredNodes)) {
                return true;
            }
            
            pathTrace.pop_back();
            exploredNodes.erase(nextVertex);
        }
    }
    return false;
}

void depthFirstTraversal(unordered_map<char, vector<char>>& adjacencyList, char source, char target) {
    vector<char> pathTrace;
    unordered_set<char> exploredNodes;
    exploredNodes.insert(source);
    pathTrace.push_back(source);
    
    depthFirstRecursion(adjacencyList, source, target, pathTrace, exploredNodes);
}

int main() {
    unordered_map<char, vector<char>> adjacencyList;
    adjacencyList['S'] = {'A', 'B'};
    adjacencyList['A'] = {'D', 'B', 'S'};
    adjacencyList['B'] = {'A', 'C', 'S'};
    adjacencyList['C'] = {'E', 'B'};
    adjacencyList['D'] = {'A', 'G'};
    adjacencyList['E'] = {'C'};
    adjacencyList['G'] = {'D'};
    
    depthFirstTraversal(adjacencyList, 'S', 'G');
    return 0;
}
