#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include <string>
#include <queue>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <limits>
#include "Graph.hpp"

namespace ariel {

    class Algorithms {
    public:
        static bool isConnected(const Graph& g);
        static std::string shortestPath(const Graph& g, int start, int end);
        static bool isContainsCycle(const Graph& g);
        static std::string isBipartite(const Graph& g);
        static bool negativeCycle(const Graph& g);
        static std::string BFS(const Graph& g, size_t start, size_t end);
        static std::string Dijkstra(const Graph& g, size_t start, size_t end);
        static std::string BellmanFord(const Graph& g, size_t start, size_t end);
    };

}

#endif
