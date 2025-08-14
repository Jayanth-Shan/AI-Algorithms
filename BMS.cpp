#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

void exhaustiveSearch(unordered_map<char, vector<char>>& adjacencyList, char source, char target) {
    auto comparator = [](const pair<char, vector<char>>& first, const pair<char, vector<char>>& second) {
        if (first.first != second.first) return first.first > second.first;
        return first.second.size() > second.second.size();
    };
    
    priority_queue<pair<char, vector<char>>, vector<pair<char, vector<char>>>, decltype(comparator)> searchQueue(comparator);
    searchQueue.push({source, {source}});
    
    while (!searchQueue.empty()) {
        auto currentState = searchQueue.top();
        searchQueue.pop();
        
        char currentNode = currentState.first;
        vector<char> currentPath = currentState.second;
        
        if (currentNode == target) {
            for (char vertex : currentPath) {
                cout << vertex << " ";
            }
            cout << endl;
            continue;
        }
        
        for (char adjacentNode : adjacencyList[currentNode]) {
            if (find(currentPath.begin(), currentPath.end(), adjacentNode) == currentPath.end()) {
                vector<char> extendedPath = currentPath;
                extendedPath.push_back(adjacentNode);
                searchQueue.push({adjacentNode, extendedPath});
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
    
    exhaustiveSearch(adjacencyList, 'S', 'G');
    return 0;
}
