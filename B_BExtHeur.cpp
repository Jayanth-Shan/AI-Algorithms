#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <climits>
using namespace std;

int comprehensivePathFinder(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                           char currentVertex, char destinationVertex,
                           vector<char>& pathSequence, int accumulatedCost) {
    
    if (currentVertex == destinationVertex) {
        cout << "Complete solution: ";
        for (char vertex : pathSequence) cout << vertex << " ";
        cout << "| Total cost: " << accumulatedCost << endl;
        return accumulatedCost;
    }
    
    int optimalResult = INT_MAX;
    
    for (auto& adjacentConnection : weightedGraph[currentVertex]) {
        if (find(pathSequence.begin(), pathSequence.end(), adjacentConnection.first) == pathSequence.end()) {
            pathSequence.push_back(adjacentConnection.first);
            optimalResult = min(optimalResult, comprehensivePathFinder(weightedGraph, adjacentConnection.first, destinationVertex, pathSequence, accumulatedCost + adjacentConnection.second));
            pathSequence.pop_back();
        }
    }
    
    return optimalResult;
}

int enhancedBranchBoundHeuristic(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                                unordered_map<char, int>& heuristicEstimator,
                                char startVertex, char goalVertex, int boundLimit) {
    
    using SearchElement = pair<pair<int, int>, pair<char, vector<char>>>;
    priority_queue<SearchElement, vector<SearchElement>, greater<SearchElement>> explorationQueue;
    
    int initialEstimate = heuristicEstimator[startVertex];
    explorationQueue.push({{initialEstimate, 0}, {startVertex, {startVertex}}});
    unordered_set<char> processedNodes;
    
    while (!explorationQueue.empty()) {
        auto [costData, nodeData] = explorationQueue.top();
        auto [estimatedCost, actualCost] = costData;
        auto [currentNode, traversalPath] = nodeData;
        explorationQueue.pop();
        
        if (actualCost > boundLimit) return -1;
        
        if (currentNode == goalVertex) {
            cout << "Optimal solution found: ";
            for (char vertex : traversalPath) cout << vertex << " ";
            cout << "| Path cost: " << actualCost << endl;
            return actualCost;
        }
        
        processedNodes.insert(currentNode);
        
        for (auto& neighborConnection : weightedGraph[currentNode]) {
            if (!processedNodes.count(neighborConnection.first)) {
                int updatedActualCost = actualCost + neighborConnection.second;
                int updatedEstimate = updatedActualCost + heuristicEstimator[neighborConnection.first];
                vector<char> extendedPath = traversalPath;
                extendedPath.push_back(neighborConnection.first);
                explorationQueue.push({{updatedEstimate, updatedActualCost}, {neighborConnection.first, extendedPath}});
            }
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
    
    unordered_map<char, int> heuristicEstimator = {
        {'S', 10}, {'A', 7}, {'B', 6}, {'C', 7}, {'D', 5}, {'E', 6}, {'G', 0}
    };
    
    vector<char> pathSequence = {'S'};
    int optimalDistance = comprehensivePathFinder(weightedGraph, 'S', 'G', pathSequence, 0);
    
    if (optimalDistance == INT_MAX) {
        cout << "Destination unreachable." << endl;
        return -1;
    } else {
        cout << "Oracle optimal distance: " << optimalDistance << endl;
    }
    
    cout << "\n--- Enhanced Branch and Bound with Heuristic ---\n";
    int searchResult = enhancedBranchBoundHeuristic(weightedGraph, heuristicEstimator, 'S', 'G', optimalDistance);
    if (searchResult == -1)
        cout << "Target unreachable with enhanced heuristic.\n";
    else
        cout << "Optimal cost via Enhanced B&B + Heuristic: " << searchResult << endl;
    
    return 0;
}
