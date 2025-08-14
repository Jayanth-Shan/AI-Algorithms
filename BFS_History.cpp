#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <unordered_set>
using namespace std;

void breadthFirstWithHistory(unordered_map<char, vector<char>>& adjacencyList, char source, char target) {
    queue<vector<char>> pathQueue;
    unordered_set<char> globalVisited;
    
    pathQueue.push({source});
    globalVisited.insert(source);
    
    while (!pathQueue.empty()) {
        int levelCount = pathQueue.size();
        
        for (int i = 0; i < levelCount; i++) {
            vector<char> currentPath = pathQueue.front();
            pathQueue.pop();
            
            char lastNode = currentPath.back();
            
            if (lastNode == target) {
                for (char vertex : currentPath) {
                    cout << vertex << " ";
                }
                cout << endl;
                return;
            }
            
            for (char adjacentNode : adjacencyList[lastNode]) {
                if (globalVisited.find(adjacentNode) == globalVisited.end()) {
                    globalVisited.insert(adjacentNode);
                    vector<char> extendedPath = currentPath;
                    extendedPath.push_back(adjacentNode);
                    pathQueue.push(extendedPath);
                }
            }
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
    
    breadthFirstWithHistory(adjacencyList, 'S', 'G');
    return 0;
}
