#include "Graph.hpp"
#include <iostream>

namespace ariel {

    Graph::Graph() {}

    void Graph::loadGraph(const std::vector<std::vector<int>>& graph_matrix) {
        // Checks if the graph matrix is square
        if (graph_matrix.size() != graph_matrix[0].size()) {
            throw std::invalid_argument("Invalid graph: The graph is not a square matrix.");
        }

        // Copies the graph matrix
        this->graph_matrix = graph_matrix;
    }

    void Graph::printGraph() const {
        std::cout << "Graph with " << graph_matrix.size() << " vertices and " << countEdges() << " edges." << std::endl;

        // Prints the adjacency matrix
        for (const std::vector<int>& row : graph_matrix) {
            for (int element : row) {
                std::cout << element << " ";
            }
            std::cout << std::endl;
        }
    }

    int Graph::countEdges() const {
        int edges_count = 0;

        for (int i = 0; i < graph_matrix.size(); ++i) {
           for (int j = i + 1; j < graph_matrix.size(); ++j) {
            if (static_cast<unsigned int>(graph_matrix[i][j]) != 0) {
              edges_count++;
        }
    }
 }

        return edges_count;
    }



    int Graph::isWeighted() const {
        // Checks if the graph is weighted and if it contains negative edges
        for (const std::vector<int>& row : graph_matrix) {
            for (int element : row) {
                if (element != 0 && element != 1) {
                    return 1; // The graph is weighted
                }
                if (element < 0) {
                    return -1; // The graph contains negative edges
                }
            }
        }

        return 0; // The graph is unweighted and contains no negative edges
    }

    const std::vector<std::vector<int>>& Graph::getGraphMatrix() const {
        return graph_matrix;
    }

}
