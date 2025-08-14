#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
using namespace std;

bool beamSearchWithGlobalMemory(unordered_map<char, vector<pair<char, int>>>& weightedGraph,
                                unordered_map<char, int>& heuristicFunction,
                                char startNode, char targetNode, size_t beamCapacity) {
    
    unordered_set<char> globalExploredSet;
    
    auto priorityComparator = [&](const pair<char, vector<char>>& first, const pair<char, vector<char>>& second) {
        return heuristicFunction.at(first.first) > heuristicFunction.at(second.first);
    };
    
    priority_queue<pair<char, vector<char>>, vector<pair<char, vector<char>>>, decltype(priorityComparator)> activeLevelQueue(priorityComparator);
    activeLevelQueue.push({startNode, {startNode}});
    globalExploredSet.insert(startNode);
    
    while (!activeLevelQueue.empty()) {
        priority_queue<pair<char, vector<char>>, vector<pair<char, vector<char>>>, decltype(priorityComparator)> nextIterationQueue(priorityComparator);
        
        while (!activeLevelQueue.empty()) {
            auto currentElement = activeLevelQueue.top();
            activeLevelQueue.pop();
            
            char currentVertex = currentElement.first;
            vector<char> pathToVertex = currentElement.second;
            
            if (currentVertex == targetNode) {
                cout << "Solution discovered: ";
                for (char vertex : pathToVertex) {
                    cout << vertex << " ";
                }
                cout << endl;
                return true;
            }
            
            for (auto& neighborData : weightedGraph.at(currentVertex)) {
                if (globalExploredSet.find(neighborData.first) == globalExploredSet.end()) {
                    auto expandedPath = pathToVertex;
                    expandedPath.push_back(neighborData.first);
                    nextIterationQueue.push({neighborData.first, expandedPath});
                }
            }
        }
        
        vector<pair<char, vector<char>>> candidateNodes;
        while (!nextIterationQueue.empty() && candidateNodes.size() < beamCapacity) {
            auto candidateNode = nextIterationQueue.top();
            nextIterationQueue.pop();
            candidateNodes.push_back(candidateNode);
            globalExploredSet.insert(candidateNode.first);
        }
        
        for (const auto& candidateNode : candidateNodes) {
            activeLevelQueue.push(candidateNode);
        }
        
        if (activeLevelQueue.empty()) {
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
    
    size_t beamCapacity = 2;
    if (!beamSearchWithGlobalMemory(weightedGraph, heuristicFunction, 'S', 'G', beamCapacity)) {
        cout << "Target node unreachable." << endl;
    }
    
    return 0;
}
