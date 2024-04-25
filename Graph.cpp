#include "Graph.hpp"
#include <iostream>

using namespace std;

Graph::Graph() {}

void Graph::loadGraph(const vector<vector<int>>& graph_matrix) {
    // Checks if the graph matrix is square
    if (graph_matrix.size() != graph_matrix[0].size()) {
        throw invalid_argument("Invalid graph: The graph is not a square matrix.");
    }

    // Copies the graph matrix
    this->graph_matrix = graph_matrix;
}

void Graph::printGraph() const {
    cout << "Graph with " << graph_matrix.size() << " vertices and " << count_edges() << " edges." << endl;

    // Prints the adjacency matrix
    for (const vector<int>& row : graph_matrix) {
        for (int element : row) {
            cout << element << " ";
        }
        cout << endl;
    }
}

int Graph::count_edges() const {
    int edges_count = 0;

    for (int i = 0; i < graph_matrix.size(); ++i) {
        for (int j = i + 1; j < graph_matrix.size(); ++j) {
            if (graph_matrix[i][j] != 0) {
                edges_count++;
            }
        }
    }

    return edges_count;
}

bool Graph::isDirected() const {
    // Checks if the adjacency matrix is symmetric
    for (int i = 0; i < graph_matrix.size(); ++i) {
        for (int j = 0; j < graph_matrix.size(); ++j) {
            if (graph_matrix[i][j] != graph_matrix[j][i]) {
                return true; // The graph is directed
            }
        }
    }

    return false; // The graph is undirected
}

int Graph::isWeighted() const {
    // Checks if the graph is weighted and if it contains negative edges
    for (const vector<int>& row : graph_matrix) {
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
