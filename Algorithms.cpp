// id: 322353780 mail:talyape123@gmail.com

#include "Algorithms.hpp"
#include "Graph.hpp"
#include <queue>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace std;

namespace ariel
{

    bool Algorithms::isConnected(const Graph &g)
    {
        const vector<vector<int>> &graph_matrix = g.getGraphMatrix();
        // Check if the graph is directed
        bool directed = g.isDirected();

        // Create a visited vector to track visited nodes
        std::vector<bool> visited(g.getGraphMatrix().size(), false);

        // Perform DFS traversal starting from an arbitrary vertex
        size_t start_node = 0; // You can choose any vertex
        dfs(graph_matrix, visited, start_node);

        // Check if all vertices were visited in the DFS
        for (bool v : visited)
        {
            if (!v)
            {
                // Not connected if any vertex is not visited
                return false;
            }
        }

        // If directed graph, reset visited vector for second DFS from a different vertex
        if (directed)
        {
            fill(visited.begin(), visited.end(), false);
            for (size_t i = 1; i < visited.size(); ++i)
            {
                if (g.getGraphMatrix()[i][0] > 0)
                { // Check for incoming edge to vertex i from vertex 0
                    start_node = i;
                    break; // Use this vertex as the starting point
                }
            }
            start_node = (start_node + 1) % visited.size(); // Choose a different starting vertex
            dfs(g.getGraphMatrix(), visited, start_node);

            // Not connected if any vertex is not visited in the second DFS
            for (bool v : visited)
            {
                if (!v)
                {
                    return false;
                }
            }
        }

        // If all vertices were visited in both DFS traversals (or one for undirected), the graph is connected
        return true;
    }

    // Helper function for DFS traversal (already defined in previous code)
    void Algorithms::dfs(const std::vector<std::vector<int>> &graph_matrix, std::vector<bool> &visited, size_t current_node)
    {
        visited[current_node] = true;
        for (size_t neighbor = 0; neighbor < graph_matrix.size(); ++neighbor)
        {
            if ((graph_matrix[current_node][neighbor] > 0) && !visited[neighbor])
            {
                dfs(graph_matrix, visited, neighbor);
            }
        }
    }

    string Algorithms::shortestPath(const Graph &g, int start, int end)
    {

        const vector<vector<int>> &graph_matrix = g.getGraphMatrix();
        int n = graph_matrix.size();
        if (start < 0 || start >= n)
        {
            throw runtime_error("Invalid start vertex");
        }
        if (end < 0 || end >= n)
        {
            throw runtime_error("Invalid end vertex");
        }

        size_t t_start = (size_t)start;
        size_t t_end = (size_t)end;

        // Check if the graph is unweighted
        if (g.isWeighted() == 0)
        {
            // Use BFS for unweighted graph
            return BFS(g, t_start, t_end);
        }

        // Check if the graph has negative edges
        if (g.isWeighted() == -1)
        {
            // Use Bellman-Ford for graphs with negative edges
            return BellmanFord(g, t_start, t_end);
        }

        // Use Dijkstra for graphs with positive edges
        return Dijkstra(g, t_start, t_end);
    }

    std::string Algorithms::BFS(const Graph &g, size_t start, size_t end)
    {
        const vector<vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size(); // Use size_t for size

        vector<size_t> prev(n, static_cast<size_t>(-1)); // Cast -1 to size_t
        vector<bool> visited(n, false);

        queue<size_t> q; // queue of size_t
        q.push(start);
        visited[start] = true;

        while (!q.empty())
        {
            size_t curr = q.front();
            q.pop();

            // If we reached the destination vertex
            if (curr == end)
            {
                string path = to_string(end);
                // Building the path from the last vertex to the first
                for (size_t at = end; prev[at] != -1; at = prev[at])
                {
                    path = to_string(prev[at]) + "->" + path;
                }
                return path;
            }

            // Scan only the neighbors of the current vertex that haven't been visited yet
            for (size_t i = 0; i < n; ++i)
            {
                if (graph_matrix[curr][i] && !visited[i])
                {
                    q.push(i);
                    visited[i] = true;
                    prev[i] = curr; // Save the previous vertex for the current vertex
                }
            }
        }

        // If no path found, return "-1"
        return "-1";
    }
    std::string Algorithms::Dijkstra(const Graph &g, size_t start, size_t end)
    {

        // Get the adjacency matrix of the graph
        const std::vector<std::vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size();

        // Initialize distance and previous vertex vectors
        std::vector<int> dist(n, std::numeric_limits<int>::max());
        std::vector<int> prev(n, -1);

        // Priority queue to store vertices by their distance from the start vertex
        std::priority_queue<std::pair<int, size_t>, std::vector<std::pair<int, size_t>>, std::greater<std::pair<int, size_t>>> pq;

        // Initialize distance to start vertex as 0
        dist[start] = 0;
        pq.push({0, start});

        // Dijkstra's algorithm main loop
        while (!pq.empty())
        {
            size_t curr = pq.top().second;
            pq.pop();

            for (size_t i = 0; i < n; ++i)
            {
                if (graph_matrix[curr][i] != 0)
                {
                    size_t next = i;
                    int weight = graph_matrix[curr][i];
                    if (dist[curr] + weight < dist[next])
                    {
                        // Update distance and previous vertex if a shorter path is found
                        dist[next] = dist[curr] + weight;
                        prev[next] = curr;
                        pq.push({dist[next], next});
                    }
                }
            }
        }

        // If there's no path from start to end, return "-1"
        if (dist[end] == std::numeric_limits<int>::max())
        {
            return "-1";
        }

        // Reconstruct the shortest path
        std::string path = std::to_string(end);
        for (size_t at = static_cast<size_t>(end); prev[at] != -1; at = static_cast<size_t>(prev[at]))
        {
            path = std::to_string(prev[at]) + "->" + path;
        }

        return path;
    }

    std::string Algorithms::BellmanFord(const Graph &g, size_t start, size_t end)
    {

        const std::vector<vector<int>> &graph_matrix = g.getGraphMatrix();
        if (negativeCycle(g))
        {
            return "-1";
        }

        size_t n = graph_matrix.size();
        std::vector<int> dist(n, std::numeric_limits<int>::max());
        std::vector<int> prev(n, -1);

        dist[start] = 0;

        // Relax all edges |V| - 1 times
        for (size_t i = 0; i < n - 1; ++i)
        {
            for (size_t u = 0; u < n; ++u)
            {
                // Iterate only over neighbors of u (assuming neighbor information is available)
                // for (size_t v : g.getNeighbors(u)) { // If neighbor information is accessible
                for (size_t v = 0; v < n; ++v)
                {
                    if (graph_matrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() && dist[u] + graph_matrix[u][v] < dist[v])
                    {
                        dist[v] = dist[u] + graph_matrix[u][v];
                        prev[v] = u;
                    }
                }
            }
        }

        // Check for negative-weight cycles
        for (size_t u = 0; u < n; ++u)
        {
            for (size_t v = 0; v < n; ++v)
            {
                if (graph_matrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() && dist[u] + graph_matrix[u][v] < dist[v])
                {
                    return "Graph contains negative weight cycle";
                }
            }
        }

        // Reconstruct the path
        if (dist[end] == std::numeric_limits<int>::max())
        {
            return "-1"; // No path exists
        }

        std::string path = std::to_string(end);
        for (size_t at = static_cast<size_t>(end); prev[at] != -1; at = static_cast<size_t>(prev[at]))
        {
            path = std::to_string(prev[at]) + "->" + path;
        }

        return path;
    }

    std::string Algorithms::isContainsCycle(const Graph &g)
    {
        const std::vector<std::vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size();

        // Check if the graph is directed
        bool directed = g.isDirected();
        if (directed)
        {
            return directedGetCycle(g);
        }
        return undirectedGetCycle(g);

        // If no cycle found, return "0"
    }

    std::string Algorithms::directedGetCycle(const Graph &g)
    {
        const std::vector<std::vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size();
        std::vector<bool> visited(n, false);
        std::vector<bool> recStack(n, false);
        std::vector<size_t> cycle;

        for (size_t i = 0; i < n; ++i)
        {
            if (!visited[i] && directedDFSForCycle(graph_matrix, i, visited, recStack, cycle))
            {
                std::string result;
                for (size_t j = 0; j < cycle.size(); ++j)
                {
                    result += std::to_string(cycle[j]);
                    if (j != cycle.size() - 1)
                        result += " -> ";
                }
                return result;
            }
        }

        return "0";
    }

    bool Algorithms::directedDFSForCycle(const std::vector<std::vector<int>> &graph_matrix, size_t v, std::vector<bool> &visited, std::vector<bool> &recStack, std::vector<size_t> &cycle)
    {
        visited[v] = true;
        recStack[v] = true;

        for (size_t i = 0; i < graph_matrix.size(); ++i)
        {
            if (graph_matrix[v][i] != 0)
            {
                if (!visited[i])
                {
                    if (directedDFSForCycle(graph_matrix, i, visited, recStack, cycle))
                    {
                        cycle.push_back(v);
                        return true;
                    }
                }
                else if (recStack[i])
                {
                    cycle.push_back(i);
                    cycle.push_back(v);
                    return true;
                }
            }
        }

        recStack[v] = false; // Backtrack
        return false;
    }

    std::string Algorithms::undirectedGetCycle(const Graph &g)
    {
        const std::vector<std::vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size();
        std::vector<bool> visited(n, false);
        std::vector<size_t> cycle;

        for (size_t i = 0; i < n; ++i)
        {
            if (!visited[i] && undirectedDFSForCycle(graph_matrix, i, visited, -1, cycle))
            {
                std::string result;
                for (size_t j = 0; j < cycle.size(); ++j)
                {
                    result += std::to_string(cycle[j]);
                    if (j != cycle.size() - 1)
                        result += " -> ";
                }
                return result;
            }
        }

        return "0";
    }

    bool Algorithms::undirectedDFSForCycle(const std::vector<std::vector<int>> &graph_matrix, size_t v, std::vector<bool> &visited, int parent, std::vector<size_t> &cycle)
    {
        visited[v] = true;

        for (size_t i = 0; i < graph_matrix.size(); ++i)
        {
            if (graph_matrix[v][i] != 0)
            {
                if (!visited[i])
                {
                    if (undirectedDFSForCycle(graph_matrix, i, visited, v, cycle))
                    {
                        cycle.push_back(v);
                        return true;
                    }
                }
                else if (i != parent)
                {
                    cycle.push_back(i);
                    cycle.push_back(v);
                    return true;
                }
            }
        }

        return false;
    }

    std::string Algorithms::isBipartite(const Graph &g)
    {
        const std::vector<std::vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size();

        std::vector<int> color(n, -1); // Colors of nodes: -1 for uncolored, 0 and 1 for two colors
        std::queue<size_t> q;
        std::vector<size_t> A, B;

        // Start BFS traversal from each uncolored node
        for (size_t i = 0; i < n; ++i)
        {
            if (color[i] == -1)
            {
                q.push(i);
                color[i] = 0; // Color the starting node with color 0
                A.push_back(i);

                while (!q.empty())
                {
                    size_t curr = q.front();
                    q.pop();

                    for (size_t j = 0; j < n; ++j)
                    {
                        if (graph_matrix[curr][j] != 0)
                        {
                            if (color[j] == -1)
                            {
                                // Color adjacent node with opposite color of current node
                                color[j] = 1 - color[curr];
                                q.push(j);
                                if (color[j] == 0)
                                {
                                    A.push_back(j);
                                }
                                else
                                {
                                    B.push_back(j);
                                }
                            }
                            else if (color[j] == color[curr])
                            {
                                // If adjacent node already colored and it has the same color as current node, not bipartite
                                return "0";
                            }
                        }
                    }
                }
            }
        }

        // If reached here, all nodes are colored without conflict, so the graph is bipartite
        std::string result = "The graph is bipartite: A={";
        for (size_t i = 0; i < A.size(); ++i)
        {
            result += std::to_string(A[i]);
            if (i != A.size() - 1)
            {
                result += ", ";
            }
        }
        result += "}, B={";
        for (size_t i = 0; i < B.size(); ++i)
        {
            result += std::to_string(B[i]);
            if (i != B.size() - 1)
            {
                result += ", ";
            }
        }
        result += "}";
        return result;
    }

    bool Algorithms::negativeCycle(const Graph &g)
    {
        const std::vector<std::vector<int>> &graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size();

        // בדיקת מעגלים שליליים באמצעות אלגוריתם Bellman-Ford
        std::vector<int> dist(n, 0);  // כאן נשמור את המרחקים מהצומת המקור
        std::vector<int> prev(n, -1); // כאן נשמור את הצומת הקודמת במסלול הקצר ביותר

        // חישוב כל המרחקים מצומת המקור
        for (size_t i = 0; i < n; ++i)
        {
            for (size_t u = 0; u < n; ++u)
            {
                for (size_t v = 0; v < n; ++v)
                {
                    if (graph_matrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() && dist[u] + graph_matrix[u][v] < dist[v])
                    {
                        dist[v] = dist[u] + graph_matrix[u][v];
                        prev[v] = u;
                        if (i == n - 1)
                        {
                            // יש מעגל שלילי - חזרה על עצמו
                            return true;
                        }
                    }
                }
            }
        }

        // אין מעגלים שליליים
        return false;
    }

}
