#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

bool beamSearchAlgorithm(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                        unordered_map<char, int>& heuristicFunction,
                        char startNode, char targetNode, size_t beamLimit) {
    
    auto nodeComparator = [&](const pair<char, vector<char>>& first, const pair<char, vector<char>>& second) {
        return heuristicFunction.at(first.first) > heuristicFunction.at(second.first);
    };
    
    priority_queue<pair<char, vector<char>>, vector<pair<char, vector<char>>>, decltype(nodeComparator)> currentLevelQueue(nodeComparator);
    currentLevelQueue.push({startNode, {startNode}});
    
    while (!currentLevelQueue.empty()) {
        priority_queue<pair<char, vector<char>>, vector<pair<char, vector<char>>>, decltype(nodeComparator)> nextLevelQueue(nodeComparator);
        
        while (!currentLevelQueue.empty()) {
            pair<char, vector<char>> currentState = currentLevelQueue.top();
            currentLevelQueue.pop();
            
            char currentNode = currentState.first;
            vector<char> currentPath = currentState.second;
            
            if (currentNode == targetNode) {
                cout << "Solution path: ";
                for (char vertex : currentPath) {
                    cout << vertex << " ";
                }
                cout << endl;
                return true;
            }
            
            for (auto& adjacentPair : weightedGraph.at(currentNode)) {
                auto extendedPath = currentPath;
                extendedPath.push_back(adjacentPair.first);
                nextLevelQueue.push({adjacentPair.first, extendedPath});
            }
        }
        
        vector<pair<char, vector<char>>> selectedNodes;
        while (!nextLevelQueue.empty() && selectedNodes.size() < beamLimit) {
            selectedNodes.push_back(nextLevelQueue.top());
            nextLevelQueue.pop();
        }
        
        for (const auto& selectedNode : selectedNodes) {
            currentLevelQueue.push(selectedNode);
        }
        
        if (currentLevelQueue.empty()) {
            break;
        }
    }
    
    return false;
}

int main() {
    unordered_map<char, vector<pair<char, int>>> weightedGraph;
    weightedGraph['S'] = {{'A', 3}, {'B', 5}};
    weightedGraph['A'] = {{'D', 3}, {'B', 4}, {'S', 3}};
    weightedGraph['B'] = {{'A', 4}, {'C', 4}, {'S', 5}};
    weightedGraph['C'] = {{'E', 6}, {'B', 6}};
    weightedGraph['D'] = {{'A', 3}, {'G', 5}};
    weightedGraph['E'] = {{'C', 6}};
    weightedGraph['G'] = {{'D', 5}};
    
    unordered_map<char, int> heuristicFunction = {
        {'S', 10}, {'A', 7}, {'B', 6}, {'C', 7}, {'D', 5}, {'E', 6}, {'G', 0}
    };
    
    size_t beamLimit = 2;
    if (!beamSearchAlgorithm(weightedGraph, heuristicFunction, 'S', 'G', beamLimit)) {
        cout << "Target node unreachable." << endl;
    }
    
    return 0;
}
