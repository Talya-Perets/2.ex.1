#include "Algorithms.hpp"
#include "Graph.hpp"
#include <queue>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace std;

namespace ariel {

    bool Algorithms::isConnected(const Graph& g) {
        const vector<vector<int>>& graph_matrix = g.getGraphMatrix();
        int n = graph_matrix.size();
        unordered_set<int> visited;

        queue<size_t> q;
        q.push(0); // Start from vertex 0
        visited.insert(0);

        while (!q.empty()) {
            size_t curr = q.front();
            q.pop();

            for (size_t i = 0; i < n; ++i) {
                if (graph_matrix[curr][i] && visited.find(i) == visited.end()) {
                    q.push(i);
                    visited.insert(i);
                }
            }
        }

        return visited.size() == n;
    }

    string Algorithms::shortestPath(const Graph& g, int start, int end) {
        const vector<vector<int>>& graph_matrix = g.getGraphMatrix();
        int n = graph_matrix.size();
        if (start < 0 || start >= n) {
            throw runtime_error("Invalid start vertex");
        }
        if (end < 0 || end >= n) {
            throw runtime_error("Invalid end vertex");
        }

        size_t t_start = (size_t) start;
        size_t t_end = (size_t) end;

        // Check if the graph is unweighted
        if (g.isWeighted() == 0) {
            // Use BFS for unweighted graph
            return BFS(g, t_start, t_end);
        }

        // Check if the graph has negative edges
        if (g.isWeighted() == -1) {
            // Use Bellman-Ford for graphs with negative edges
            return BellmanFord(g, t_start, t_end);
        }

        // Use Dijkstra for graphs with positive edges
        return Dijkstra(g, t_start, t_end);
    }

    std::string Algorithms::BFS(const Graph& g, size_t start, size_t end) {
        const vector<vector<int>>& graph_matrix = g.getGraphMatrix();
        size_t n = graph_matrix.size(); // Use size_t for size

        vector<size_t> prev(n, static_cast<size_t>(-1)); // Cast -1 to size_t
        vector<bool> visited(n, false);

        queue<size_t> q; // queue of size_t
        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            size_t curr = q.front();
            q.pop();

            // If we reached the destination vertex
            if (curr == end) {
                string path = to_string(end);
                // Building the path from the last vertex to the first
                for (size_t at = end; prev[at] != -1; at = prev[at]) {
                    path = to_string(prev[at]) + "->" + path;
                }
                return path;
            }

            // Scan all neighbors of the current vertex
            for (size_t i = 0; i < n; ++i) {
                if (graph_matrix[curr][i] && !visited[i]) {
                    q.push(i);
                    visited[i] = true;
                    prev[i] = curr; // Save the previous vertex for the current vertex
                }
            }
        }

        // If no path found, return "-1"
        return "-1";
    }

std::string Algorithms::Dijkstra(const Graph& g, size_t start, size_t end) {
    const std::vector<std::vector<int>>& graph_matrix = g.getGraphMatrix();
    size_t n = graph_matrix.size();
    std::vector<int> dist(n, std::numeric_limits<int>::max());
    std::vector<int> prev(n, -1);
    std::priority_queue<std::pair<int, size_t>, std::vector<std::pair<int, size_t>>, std::greater<std::pair<int, size_t>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        size_t curr = pq.top().second;
        pq.pop();

        for (size_t i = 0; i < n; ++i) {
            if (graph_matrix[curr][i] != 0) {
                size_t next = i;
                int weight = graph_matrix[curr][i];
                if (dist[curr] + weight < dist[next]) {
                    dist[next] = dist[curr] + weight;
                    prev[next] = curr;
                    pq.push({dist[next], next});
                }
            }
        }
    }

    if (dist[end] == std::numeric_limits<int>::max()) {
        return "-1"; // No path exists
    }

    std::string path = std::to_string(end);
    for (size_t at = static_cast<size_t>(end); prev[at] != -1; at = static_cast<size_t>(prev[at])) {
            path = std::to_string(prev[at]) + "->" + path;
    }

    return path;
}
std::string Algorithms::BellmanFord(const Graph& g, size_t start, size_t end) {
  const std::vector<vector<int>>& graph_matrix = g.getGraphMatrix();
  size_t n = graph_matrix.size();
  std::vector<int> dist(n, std::numeric_limits<int>::max());
  std::vector<int> prev(n, -1);

  dist[start] = 0;

  // Relax all edges |V| - 1 times
  for (size_t i = 0; i < n - 1; ++i) {
    for (size_t u = 0; u < n; ++u) {
      // Iterate only over neighbors of u (assuming neighbor information is available)
      // for (size_t v : g.getNeighbors(u)) { // If neighbor information is accessible
      for (size_t v = 0; v < n; ++v) {
        if (graph_matrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() && dist[u] + graph_matrix[u][v] < dist[v]) {
          dist[v] = dist[u] + graph_matrix[u][v];
          prev[v] = u;
        }
      }
    }
  }

  // Check for negative-weight cycles
  for (size_t u = 0; u < n; ++u) {
    for (size_t v = 0; v < n; ++v) {
      if (graph_matrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() && dist[u] + graph_matrix[u][v] < dist[v]) {
        return "Graph contains negative weight cycle";
      }
    }
  }

  // Reconstruct the path
  if (dist[end] == std::numeric_limits<int>::max()) {
    return "-1"; // No path exists
  }

  std::string path = std::to_string(end);
 for (size_t at = static_cast<size_t>(end); prev[at] != -1; at = static_cast<size_t>(prev[at])) {
        path = std::to_string(prev[at]) + "->" + path;
  }

  return path;
}

bool Algorithms::isContainsCycle(const Graph& g) {
    const std::vector<std::vector<int>>& graph_matrix = g.getGraphMatrix();
    size_t n = graph_matrix.size();

    // בדיקה עבור כל קודקוד בגרף
    for (size_t i = 0; i < n; ++i) {
        std::vector<bool> visited(n, false); // מערך שמסמן האם קודקוד נבדק
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            size_t curr = q.front();
            q.pop();

            // בדיקה עבור כל השכנים של הקודקוד הנוכחי
            for (size_t j = 0; j < n; ++j) {
                if (graph_matrix[curr][j] != 0) {
                    size_t next = j;
                    // אם הקשת קיימת והקודקוד הבא כבר נבדק - יש מעגל
                    if (visited[next]) {
                        return true;
                    }
                    q.push(next);
                    visited[next] = true;
                }
            }
        }
    }

    // אם לא נמצאו מעגלים
    return false;
}

std::string Algorithms::isBipartite(const Graph& g) {
    const std::vector<std::vector<int>>& graph_matrix = g.getGraphMatrix();
    size_t n = graph_matrix.size();

    // יצירת מערכים לשמירת הצביעות של כל צומת והבדיקה האם כבר נבדק
    std::vector<int> colors(n, -1); // הערכים של הצביעות: 0 או 1
    std::vector<bool> visited(n, false); // מערך שמסמן האם צומת נבדק

    // בדיקת החלוקה של הגרף לגרף דו-צדדי באמצעות BFS
    for (size_t i = 0; i < n; ++i) {
        if (!visited[i]) {
            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;
            colors[i] = 0;

            while (!q.empty()) {
                size_t curr = q.front();
                q.pop();

                for (size_t j = 0; j < n; ++j) {
                    if (graph_matrix[curr][j] != 0) {
                        size_t next = j;
                        if (!visited[next]) {
                            q.push(next);
                            visited[next] = true;
                            colors[next] = 1 - colors[curr]; // צביעת הצומת הבאה בצבע ההפוך
                        } else if (colors[next] == colors[curr]) {
                            return "The graph is not bipartite";
                        }
                    }
                }
            }
        }
    }

    // בדיקה שהגרף כן דו-צדדי
    return "The graph is bipartite";
}

bool Algorithms::negativeCycle(const Graph& g) {
    const std::vector<std::vector<int>>& graph_matrix = g.getGraphMatrix();
    size_t n = graph_matrix.size();

    // בדיקת מעגלים שליליים באמצעות אלגוריתם Bellman-Ford
    std::vector<int> dist(n, 0); // כאן נשמור את המרחקים מהצומת המקור
    std::vector<int> prev(n, -1); // כאן נשמור את הצומת הקודמת במסלול הקצר ביותר

    // חישוב כל המרחקים מצומת המקור
    for (size_t i = 0; i < n; ++i) {
        for (size_t u = 0; u < n; ++u) {
            for (size_t v = 0; v < n; ++v) {
                if (graph_matrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() 
                    && dist[u] + graph_matrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph_matrix[u][v];
                    prev[v] = u;
                    if (i == n - 1) {
                        // יש מעגל שלילי - חזרה על עצמו
                        std::cout << "Negative weight cycle found" << std::endl;
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


