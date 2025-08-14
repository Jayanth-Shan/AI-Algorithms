#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
using namespace std;

int exhaustiveOptimalSearch(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                           char currentVertex, char destinationVertex,
                           vector<char>& pathSequence, int pathCost) {
    
    if (currentVertex == destinationVertex) {
        cout << "Complete path: ";
        for (char vertex : pathSequence) cout << vertex << " ";
        cout << "| Path cost: " << pathCost << endl;
        return pathCost;
    }
    
    int minimalCost = INT_MAX;
    
    for (auto& adjacentData : weightedGraph[currentVertex]) {
        if (find(pathSequence.begin(), pathSequence.end(), adjacentData.first) == pathSequence.end()) {
            pathSequence.push_back(adjacentData.first);
            minimalCost = min(minimalCost, exhaustiveOptimalSearch(weightedGraph, adjacentData.first, destinationVertex, pathSequence, pathCost + adjacentData.second));
            pathSequence.pop_back();
        }
    }
    
    return minimalCost;
}

int boundedBranchSearch(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                       char startVertex, char goalVertex, int costBound) {
    
    using SearchState = pair<int, pair<char, vector<char>>>;
    priority_queue<SearchState, vector<SearchState>, greater<SearchState>> searchQueue;
    searchQueue.push({0, {startVertex, {startVertex}}});
    
    while (!searchQueue.empty()) {
        auto [currentCost, nodeData] = searchQueue.top();
        auto [currentNode, currentPath] = nodeData;
        searchQueue.pop();
        
        if (currentCost > costBound)
            return -1;
            
        if (currentNode == goalVertex) {
            cout << "Bounded path: ";
            for (char vertex : currentPath) cout << vertex << " ";
            cout << "| Path cost: " << currentCost << endl;
            return currentCost;
        }
        
        for (auto& neighborData : weightedGraph[currentNode]) {
            vector<char> extendedPath = currentPath;
            extendedPath.push_back(neighborData.first);
            searchQueue.push({currentCost + neighborData.second, {neighborData.first, extendedPath}});
        }
    }
    
    return -1;
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
    
    vector<char> pathSequence = {'S'};
    int optimalCost = exhaustiveOptimalSearch(weightedGraph, 'S', 'G', pathSequence, 0);
    
    if (optimalCost == INT_MAX) {
        cout << "Goal unreachable." << endl;
        return -1;
    } else {
        cout << "Optimal cost (exhaustive): " << optimalCost << endl;
    }
    
    cout << "\n--- Branch and Bound Implementation ---\n";
    int boundedResult = boundedBranchSearch(weightedGraph, 'S', 'G', optimalCost);
    if (boundedResult == -1)
        cout << "Goal unreachable with bound.\n";
    else
        cout << "Optimal cost via Branch and Bound: " << boundedResult << endl;
        
    return 0;
}
