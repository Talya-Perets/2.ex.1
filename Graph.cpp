#include "Graph.hpp"
#include <iostream>

using namespace ariel;
Graph::Graph() {}

void Graph::loadGraph(const std::vector<std::vector<int>>& matrix) {
    adjacencyMatrix = matrix;
}

void Graph::printGraph() const {
    // Print the adjacency matrix
    for (const auto& row : adjacencyMatrix) {
        for (int val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<std::vector<int>> Graph::getAdjacencyMatrix() const {
    return adjacencyMatrix;
}
