#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include "Graph.hpp"
#include <string>

namespace ariel {
    class Algorithms {
    public:
        static int checkGraphType(const std::vector<std::vector<int>>& graph);
        static bool Algorithms::dfsCheckCycle(const Graph& g, int current, int parent, std::vector<bool>& visited) ;
        static bool isConnected(const Graph& g);
        static std::string shortestPath(const Graph& g, int start, int end);
        static bool isContainsCycle(const Graph& g);
        static std::string isBipartite(const Graph& g);
        static bool negativeCycle(const Graph& g);
    };
}

#endif // ALGORITHMS_HPP