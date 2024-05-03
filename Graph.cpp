#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <stdexcept>

namespace ariel {

    class Graph {
    public:
        Graph();
        void loadGraph(const std::vector<std::vector<int>>& graph_matrix);
        void printGraph() const;
        int countEdges() const;
        bool isDirected() const;
        int isWeighted() const; 
        const std::vector<std::vector<int>>& getGraphMatrix() const;

    private:
        std::vector<std::vector<int>> graph_matrix;
    };

}

#endif
