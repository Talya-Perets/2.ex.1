// id: 322353780 mail:talyape123@gmail.com

#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include <string>
#include <queue>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <limits>
#include "Graph.hpp"

namespace ariel
{

    class Algorithms
    {
    public:
        static bool isConnected(const Graph &g);
        static std::string shortestPath(const Graph &g, int start, int end);
        static std::string isContainsCycle(const Graph &g);
        static std::string isBipartite(const Graph &g);
        static bool negativeCycle(const Graph &g);

    private:
        static void dfs(const std::vector<std::vector<int>> &graph_matrix, std::vector<bool> &visited, size_t current_node);
        static std::string buildCycleString(const std::vector<size_t> &cycle);
        static bool dfsForCycle(const std::vector<std::vector<int>> &graph_matrix, size_t v, std::vector<bool> &visited, std::vector<bool> &recStack, std::vector<size_t> &cycle, int parent);
        static std::string BFS(const Graph &g, size_t start, size_t end);
        static std::string Dijkstra(const Graph &g, size_t start, size_t end);
        static std::string BellmanFord(const Graph &g, size_t start, size_t end);
    };

}

#endif
