#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <climits>
using namespace std;

int exhaustiveSearchOracle(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                          char currentState, char goalState,
                          vector<char>& solutionPath, int totalCost) {
    
    if (currentState == goalState) {
        cout << "Oracle solution: ";
        for (char state : solutionPath) cout << state << " ";
        cout << "| Total cost: " << totalCost << endl;
        return totalCost;
    }
    
    int minimalCost = INT_MAX;
    
    for (auto& adjacentState : weightedGraph[currentState]) {
        if (find(solutionPath.begin(), solutionPath.end(), adjacentState.first) == solutionPath.end()) {
            solutionPath.push_back(adjacentState.first);
            minimalCost = min(minimalCost, exhaustiveSearchOracle(weightedGraph, adjacentState.first, goalState, solutionPath, totalCost + adjacentState.second));
            solutionPath.pop_back();
        }
    }
    
    return minimalCost;
}

using AStarState = pair<int, pair<int, pair<char, vector<char>>>>;

int aStarOptimalSearch(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                      unordered_map<char, int>& heuristicFunction,
                      char initialState, char targetState, int upperBound) {
    
    priority_queue<AStarState, vector<AStarState>, greater<AStarState>> priorityQueue;
    int initialHeuristic = heuristicFunction[initialState];
    vector<char> initialPath = {initialState};
    priorityQueue.push({initialHeuristic, {0, {initialState, initialPath}}});
    
    unordered_set<char> exploredStates;
    int optimalCost = INT_MAX;
    
    while (!priorityQueue.empty()) {
        auto [fValue, stateInfo] = priorityQueue.top();
        auto [gValue, nodeInfo] = stateInfo;
        auto [currentState, pathToState] = nodeInfo;
        priorityQueue.pop();
        
        if (gValue > optimalCost || fValue > upperBound) {
            continue;
        }
        
        if (currentState == targetState) {
            if (gValue < optimalCost) {
                optimalCost = gValue;
            }
            
            cout << "A* solution discovered: ";
            for (char state : pathToState) cout << state << " ";
            cout << "| Path cost: " << gValue << endl;
            continue;
        }
        
        if (exploredStates.count(currentState)) {
            continue;
        }
        
        exploredStates.insert(currentState);
        
        for (auto& adjacentState : weightedGraph[currentState]) {
            char nextState = adjacentState.first;
            int transitionCost = adjacentState.second;
            
            if (!exploredStates.count(nextState)) {
                int newGValue = gValue + transitionCost;
                int newHValue = heuristicFunction[nextState];
                int newFValue = newGValue + newHValue;
                
                if (newGValue < optimalCost) {
                    vector<char> extendedPath = pathToState;
                    extendedPath.push_back(nextState);
                    priorityQueue.push({newFValue, {newGValue, {nextState, extendedPath}}});
                }
            }
        }
    }
    
    return (optimalCost == INT_MAX) ? -1 : optimalCost;
}

int main() {
    unordered_map<char, vector<pair<char, int>>> weightedGraph;
    weightedGraph['S'] = {{'A', 3}, {'B', 5}};
    weightedGraph['A'] = {{'D', 3}, {'B', 4}};
    weightedGraph['B'] = {{'A', 4}, {'C', 4}};
    weightedGraph['C'] = {{'E', 6}};
    weightedGraph['D'] = {{'A', 3}, {'G', 5}};
    weightedGraph['E'] = {{'C', 6}};
    weightedGraph['G'] = {{'D', 5}};
    
    unordered_map<char, int> heuristicFunction = {
        {'S', 10}, {'A', 7}, {'B', 6}, {'C', 7}, {'D', 5}, {'E', 6}, {'G', 0}
    };
    
    char initialState = 'S';
    char targetState = 'G';
    
    cout << "--- Oracle Search (Exhaustive) ---\n";
    vector<char> solutionPath = {initialState};
    int oracleResult = exhaustiveSearchOracle(weightedGraph, initialState, targetState, solutionPath, 0);
    
    if (oracleResult == INT_MAX) {
        cout << "Target unreachable by oracle." << endl;
        return -1;
    } else {
        cout << "Oracle optimal cost: " << oracleResult << endl;
    }
    
    cout << "\n--- A* Search Algorithm ---\n";
    int aStarResult = aStarOptimalSearch(weightedGraph, heuristicFunction, initialState, targetState, oracleResult);
    if (aStarResult == -1)
        cout << "Target unreachable by A* search.\n";
    else
        cout << "A* optimal cost: " << aStarResult << endl;
    
    return 0;
}
