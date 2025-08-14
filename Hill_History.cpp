#include <iostream>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <algorithm>
using namespace std;

bool greedySearchWithMemory(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                           unordered_map<char, int>& heuristicValues,
                           char currentNode, char goalNode,
                           unordered_set<char>& exploredSet,
                           vector<char>& solutionPath) {
    
    solutionPath.push_back(currentNode);
    
    if (currentNode == goalNode) {
        return true;
    }
    
    exploredSet.insert(currentNode);
    
    vector<pair<char, int>> neighborList = weightedGraph[currentNode];
    sort(neighborList.begin(), neighborList.end(),
         [&](const auto& first, const auto& second) {
             return heuristicValues[first.first] < heuristicValues[second.first];
         });
    
    for (const auto& neighborPair : neighborList) {
        char nextNode = neighborPair.first;
        if (exploredSet.find(nextNode) == exploredSet.end()) {
            if (greedySearchWithMemory(weightedGraph, heuristicValues, nextNode, goalNode, exploredSet, solutionPath)) {
                return true;
            }
        }
    }
    
    solutionPath.pop_back();
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
    
    unordered_map<char, int> heuristicValues = {
        {'S', 10}, {'A', 7}, {'B', 6}, {'C', 7}, {'D', 5}, {'E', 6}, {'G', 0}
    };
    
    unordered_set<char> exploredSet;
    vector<char> solutionPath;
    
    if (greedySearchWithMemory(weightedGraph, heuristicValues, 'S', 'G', exploredSet, solutionPath)) {
        cout << "Solution found: ";
        for (char node : solutionPath) {
            cout << node << " ";
        }
        cout << endl;
    } else {
        cout << "Target node unreachable." << endl;
    }
    
    return 0;
}
