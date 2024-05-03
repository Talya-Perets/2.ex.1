#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

namespace ariel {
    class Graph {
    private:
        std::vector<std::vector<int>> adjacencyMatrix;

    public:
        Graph(); // Constructor
        void loadGraph(const std::vector<std::vector<int>>& matrix);
        void printGraph() const;
        std::vector<std::vector<int>> getAdjacencyMatrix() const; // פונקציה חסרה
    };
}

#endif // GRAPH_HPP
