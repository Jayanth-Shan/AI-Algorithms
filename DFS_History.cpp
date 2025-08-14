#include <iostream>
#include <unordered_map>
#include <vector>
#include <unordered_set>
using namespace std;

bool depthFirstWithDeadEndTracking(unordered_map<char, vector<char>>& adjacencyList, 
                                   char currentVertex, char goalVertex,
                                   vector<char>& pathTrace, 
                                   unordered_set<char>& visitedNodes,
                                   unordered_set<char>& deadEndNodes) {
    
    if (deadEndNodes.count(currentVertex) || visitedNodes.count(currentVertex)) {
        return false;
    }
    
    visitedNodes.insert(currentVertex);
    pathTrace.push_back(currentVertex);
    
    if (currentVertex == goalVertex) {
        for (char vertex : pathTrace) {
            cout << vertex << " ";
        }
        cout << endl;
        return true;
    }
    
    bool pathFound = false;
    for (char neighborVertex : adjacencyList[currentVertex]) {
        if (visitedNodes.find(neighborVertex) == visitedNodes.end()) {
            if (depthFirstWithDeadEndTracking(adjacencyList, neighborVertex, goalVertex, pathTrace, visitedNodes, deadEndNodes)) {
                pathFound = true;
                break;
            }
        }
    }
    
    if (!pathFound) {
        deadEndNodes.insert(currentVertex);
    }
    
    pathTrace.pop_back();
    return pathFound;
}

void depthFirstSearchWithDeadEnds(unordered_map<char, vector<char>>& adjacencyList, char source, char target) {
    vector<char> pathTrace;
    unordered_set<char> visitedNodes;
    unordered_set<char> deadEndNodes;
    
    depthFirstWithDeadEndTracking(adjacencyList, source, target, pathTrace, visitedNodes, deadEndNodes);
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
    
    depthFirstSearchWithDeadEnds(adjacencyList, 'S', 'G');
    return 0;
}
