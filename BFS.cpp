#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

void breadthFirstTraversal(unordered_map<char, vector<char>>& adjacencyList, char source, char target) {
    queue<pair<char, vector<char>>> explorationQueue;
    explorationQueue.push({source, {source}});
    
    bool goalFound = false;
    
    while (!explorationQueue.empty()) {
        int currentLevelSize = explorationQueue.size();
        vector<vector<char>> pathsAtLevel;
        
        for (int index = 0; index < currentLevelSize; index++) {
            auto frontElement = explorationQueue.front();
            explorationQueue.pop();
            
            char currentVertex = frontElement.first;
            vector<char> traversalPath = frontElement.second;
            
            if (currentVertex == target) {
                goalFound = true;
                pathsAtLevel.push_back(traversalPath);
                continue;
            }
            
            if (!goalFound) {
                for (char neighbor : adjacencyList[currentVertex]) {
                    if (find(traversalPath.begin(), traversalPath.end(), neighbor) == traversalPath.end()) {
                        vector<char> updatedPath = traversalPath;
                        updatedPath.push_back(neighbor);
                        explorationQueue.push({neighbor, updatedPath});
                    }
                }
            }
        }
        
        if (goalFound) {
            for (auto& pathSequence : pathsAtLevel) {
                for (char node : pathSequence) {
                    cout << node << " ";
                }
                cout << endl;
            }
            break;
        }
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
    
    breadthFirstTraversal(adjacencyList, 'S', 'G');
    return 0;
}
