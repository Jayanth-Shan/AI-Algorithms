#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <climits>
using namespace std;

int optimalPathFinder(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                     char currentVertex, char destinationVertex,
                     vector<char>& traversalPath, int accumulatedCost) {
    
    if (currentVertex == destinationVertex) {
        cout << "Optimal route: ";
        for (char vertex : traversalPath) cout << vertex << " ";
        cout << "| Total cost: " << accumulatedCost << endl;
        return accumulatedCost;
    }
    
    int optimalCost = INT_MAX;
    
    for (auto& connectionData : weightedGraph[currentVertex]) {
        if (find(traversalPath.begin(), traversalPath.end(), connectionData.first) == traversalPath.end()) {
            traversalPath.push_back(connectionData.first);
            optimalCost = min(optimalCost, optimalPathFinder(weightedGraph, connectionData.first, destinationVertex, traversalPath, accumulatedCost + connectionData.second));
            traversalPath.pop_back();
        }
    }
    
    return optimalCost;
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
    
    vector<char> traversalPath = {'S'};
    int optimalDistance = optimalPathFinder(weightedGraph, 'S', 'G', traversalPath, 0);
    
    if (optimalDistance == INT_MAX) {
        cout << "Destination unreachable." << endl;
    } else {
        cout << "Minimum path cost (oracle result): " << optimalDistance << endl;
    }
    
    return 0;
}
