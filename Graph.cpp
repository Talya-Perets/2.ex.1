// id: 322353780 mail:talyape123@gmail.com

#include "Graph.hpp"
#include <iostream>
#include <stdexcept>

namespace ariel
{

    Graph::Graph() {}

    void Graph::loadGraph(const std::vector<std::vector<int>> &graph_matrix)
    {

        if (graph_matrix.empty() || graph_matrix[0].empty())
        {
            throw std::invalid_argument("Invalid graph: The graph matrix is empty.");
        }
        // Check if diagonal elements are all zero
        for (size_t i = 0; i < graph_matrix.size(); ++i) {
            if (graph_matrix[i][i] != 0) {
                throw std::invalid_argument("Invalid graph: Diagonal elements must be zero.");
            }
        }
        if (graph_matrix.size() != graph_matrix[0].size())
        {
            throw std::invalid_argument("Invalid graph: The graph is not a square matrix.");
        }

        // Copies the graph matrix
        this->graph_matrix = graph_matrix;
    }

    void Graph::printGraph() const
    {
        std::cout << "Graph with " << graph_matrix.size() << " vertices and " << countEdges() << " edges." << std::endl;

        // Prints the adjacency matrix
        for (const std::vector<int> &row : graph_matrix)
        {
            for (int element : row)
            {
                std::cout << element << " ";
            }
            std::cout << std::endl;
        }
    }

    bool Graph::isDirected() const
    {
        // Checks if the graph matrix represents a directed graph
        for (size_t i = 0; i < graph_matrix.size(); ++i)
        {
            for (size_t j = 0; j < graph_matrix.size(); ++j)
            {
                if (graph_matrix[i][j] != graph_matrix[j][i])
                {
                    return true; // There is an edge from i to j but not from j to i - the graph is directed
                }
            }
        }

        return false; // If there are no differing edges, the graph is undirected
    }

    int Graph::countEdges() const
    {
        int edges_count = 0;

        // Check if the graph is directed
        bool directed = isDirected();

        for (size_t i = 0; i < graph_matrix.size(); ++i)
        {
            for (size_t j = 0; j < graph_matrix.size(); ++j)
            {
                // For directed graph, count only if there's an edge from i to j
                // For undirected graph, count if there's any edge between i and j
                if ((graph_matrix[i][j] > 0) && (!directed || (directed && graph_matrix[j][i] > 0)))
                {
                    edges_count++;
                }
            }
        }

        return edges_count;
    }

    int Graph::isWeighted() const
    {
        // Checks if the graph is weighted and if it contains negative edges
        for (const std::vector<int> &row : graph_matrix)
        {
            for (int element : row)
            {
                if (element < 0)
                {
                    return -1; // The graph contains negative edges
                }
                if (element != 0 && element != 1)
                {
                    return 1; // The graph is weighted
                }
            }
        }

        return 0; // The graph is unweighted and contains no negative edges
    }

    const std::vector<std::vector<int>> &Graph::getGraphMatrix() const
    {
        return graph_matrix;
    }

}
