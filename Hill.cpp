#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
using namespace std;

bool greedyHillClimbingDFS(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                          unordered_map<char, int>& heuristicValues,
                          char currentState, char goalState,
                          vector<char>& solutionPath) {
    
    solutionPath.push_back(currentState);
    
    if (currentState == goalState) {
        return true;
    }
    
    vector<pair<char, int>> adjacentStates = weightedGraph[currentState];
    sort(adjacentStates.begin(), adjacentStates.end(),
         [&](const pair<char, int>& first, const pair<char, int>& second) {
             return heuristicValues[first.first] < heuristicValues[second.first];
         });
    
    for (const pair<char, int>& adjacentState : adjacentStates) {
        char nextState = adjacentState.first;
        if (greedyHillClimbingDFS(weightedGraph, heuristicValues, nextState, goalState, solutionPath)) {
            return true;
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
    
    vector<char> solutionPath;
    if (greedyHillClimbingDFS(weightedGraph, heuristicValues, 'S', 'G', solutionPath)) {
        cout << "Solution path found: ";
        for (char state : solutionPath) {
            cout << state << " ";
        }
        cout << endl;
    } else {
        cout << "No path to goal found." << endl;
    }
    
    return 0;
}
