#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
using namespace std;

bool depthLimitedTraversal(unordered_map<char, vector<char>>& adjacencyList, char currentNode, char goalNode, int depthLimit, vector<char>& pathSequence) {
    pathSequence.push_back(currentNode);
    
    if (currentNode == goalNode) {
        return true;
    }
    
    if (depthLimit <= 0) {
        pathSequence.pop_back();
        return false;
    }
    
    vector<char> adjacentNodes = adjacencyList[currentNode];
    sort(adjacentNodes.begin(), adjacentNodes.end());
    
    for (char adjacentNode : adjacentNodes) {
        if (find(pathSequence.begin(), pathSequence.end(), adjacentNode) == pathSequence.end()) {
            if (depthLimitedTraversal(adjacencyList, adjacentNode, goalNode, depthLimit - 1, pathSequence)) {
                return true;
            }
        }
    }
    
    pathSequence.pop_back();
    return false;
}

void iterativeDeepeningSearch(unordered_map<char, vector<char>>& adjacencyList, char source, char target) {
    int currentDepth = 0;
    
    while (true) {
        vector<char> pathSequence;
        
        if (depthLimitedTraversal(adjacencyList, source, target, currentDepth, pathSequence)) {
            for (char vertex : pathSequence) {
                cout << vertex << " ";
            }
            cout << endl;
            return;
        }
        currentDepth++;
    }
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
    
    iterativeDeepeningSearch(adjacencyList, 'S', 'G');
    return 0;
}
