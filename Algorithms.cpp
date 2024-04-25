#include "Algorithms.hpp"
#include <limits.h>
#include <queue>

using namespace ariel;

int Algorithms::checkGraphType(const Graph& g) {
    bool hasPositiveWeights = false;
    bool hasNegativeWeights = false;

    // Get the adjacency matrix of the graph
    const auto& adjacencyMatrix = g.getAdjacencyMatrix();

    // Iterate over the adjacency matrix to check for positive and negative weights
    for (const auto& row : adjacencyMatrix) {
        for (int weight : row) {
            if (weight > 0) {
                hasPositiveWeights = true;
            } else if (weight < 0) {
                hasNegativeWeights = true;
            }
        }
    }

    // Determine the graph type based on the presence of positive and negative weights
    if (hasPositiveWeights && !hasNegativeWeights) {
        return 1; // Positive weighted graph
    } else if (!hasPositiveWeights && hasNegativeWeights) {
        return -1; // Negative weighted graph
    } else {
        return 0; // Unweighted graph
    }
}

bool Algorithms::dfsCheckCycle(const Graph& g, int current, int parent, std::vector<bool>& visited) {
    // Mark the current vertex as visited
    visited[current] = true;

    // Visit all adjacent vertices of the current vertex
    const auto& adjMatrix = g.getAdjacencyMatrix();
    for (int neighbor = 0; neighbor < adjMatrix[current].size(); ++neighbor) {
        if (adjMatrix[current][neighbor] != 0) {
            if (!visited[neighbor]) {
                if (dfsCheckCycle(g, neighbor, current, visited)) {
                    return true; // If a cycle is found in the DFS traversal, return true
                }
            } else if (neighbor != parent) {
                return true; // If a visited vertex is found that is not the parent, a cycle exists
            }
        }
    }

    return false; // If no cycle is found in the DFS traversal, return false
}

bool Algorithms::isConnected(const Graph& g) {
    const auto& adjMatrix = g.getAdjacencyMatrix();
    int n = adjMatrix.size(); // Number of vertices in the graph

    // Create a queue for BFS
    std::queue<int> q;
    std::vector<bool> visited(n, false);

    // Choose an arbitrary vertex to start BFS
    int startVertex = 0;
    q.push(startVertex);
    visited[startVertex] = true;

    // Perform BFS traversal
    while (!q.empty()) {
        int current = q.front();
        q.pop();

        // Visit all adjacent vertices of the current vertex
        for (int i = 0; i < n; ++i) {
            if (adjMatrix[current][i] != 0 && !visited[i]) {
                visited[i] = true;
                q.push(i);
            }
        }
    }

    // Check if all vertices have been visited
    for (bool v : visited) {
        if (!v) {
            return false;
        }
    }
    return true;
}

std::string Algorithms::shortestPath(const Graph& g, int start, int end) {
    // Check the graph type using the previously implemented checkGraphType function
    int graphType = checkGraphType(g);
    // Handle unweighted graphs using Breadth-First Search (BFS)
    if (graphType == 0) {
        return bfsShortestPath(g, start, end);
    } else if (graphType == 1) {
        return dijkstraShortestPath(g, start, end);
    } else {
        return ""; // Handle negative weighted graphs (optional)
    }
}

bool Algorithms::isContainsCycle(const Graph& g) {
    const auto& adjMatrix = g.getAdjacencyMatrix();
    int n = adjMatrix.size(); // Number of vertices in the graph

    // Create a vector to keep track of visited vertices
    std::vector<bool> visited(n, false);

    // Perform DFS traversal for each vertex to find a cycle
    for (int i = 0; i < n; ++i) {
        if (!visited[i] && dfsCheckCycle(g, i, -1, visited)) {
            return true;
        }
    }

    return false;
}

std::string Algorithms::isBipartite(const Graph& g) {
    const auto& adjList = g.getAdjacencyMatrix();
    int n = adjList.size(); // Number of vertices in the graph

    // Create a vector to keep track of the color of each vertex
    std::vector<int> color(n, -1);

    // Perform BFS traversal from each unvisited vertex to check for bipartiteness
    for (int i = 0; i < n; ++i) {
        if (color[i] == -1) {
            if (!bfsCheckBipartite(g, i, color)) {
                return "The graph is not bipartite";
            }
        }
    }

    // Construct the two sets of vertices based on their colors
    std::vector<int> A, B;
    for (int i = 0; i < n; ++i) {
        if (color[i] == 0) {
            A.push_back(i);
        } else {
            B.push_back(i);
        }
    }

    // Construct the result string
    std::string result = "The graph is bipartite: A={";
    for (int vertex : A) {
        result += std::to_string(vertex) + ", ";
    }
    result.pop_back(); // Remove trailing comma
    result.pop_back(); // Remove space
    result += "}, B={";
    for (int vertex : B) {
        result += std::to_string(vertex) + ", ";
    }
    result.pop_back(); // Remove trailing comma
    result.pop_back(); // Remove space
    result += "}";

    return result;
}

bool Algorithms::negativeCycle(const Graph& g) {
    const auto& adjMatrix = g.getAdjacencyMatrix();
    int n = adjMatrix.size(); // Number of vertices in the graph

    // Initialize distances to infinity (INT_MAX)
    std::vector<int> dist(n, INT_MAX);
    dist[0] = 0; // Start from vertex 0

    // Perform relaxation for |V| - 1 iterations
    for (int i = 0; i < n - 1; ++i) {
        for (int u = 0; u < n; ++u) {
            for (int v = 0; v < n; ++v) {
                if (adjMatrix[u][v] != 0 && dist[u] != INT_MAX && dist[u] + adjMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + adjMatrix[u][v];
                }
            }
        }
    }

    // Check for negative cycles
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (adjMatrix[u][v] != 0 && dist[u] != INT_MAX && dist[u] + adjMatrix[u][v] < dist[v]) {
                return true; // Negative cycle found
            }
        }
    }

    return false; // No negative cycle found
}

std::string Algorithms::bfsShortestPath(const Graph& g, int start, int end) {
    const auto& adjMatrix = g.getAdjacencyMatrix();
    int n = adjMatrix.size(); // Number of vertices in the graph

    // Visited array to keep track of visited vertices
    std::vector<bool> visited(n, false);

    // Distance array to store distances from the start vertex
    std::vector<int> distance(n, -1);

    // Parent array to reconstruct the shortest path
    std::vector<int> parent(n, -1);

    // Create a queue for BFS
    std::queue<int> q;

    // Mark the starting vertex as visited and enqueue it
    visited[start] = true;
    distance[start] = 0;
    q.push(start);

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        // If we reach the destination vertex, reconstruct the path and return it
        if (current == end) {
            std::string path = "";
            int prev = current;
            while (prev != -1) {
                path = std::to_string(prev) + "->" + path;
                prev = parent[prev];
            }
            path = std::to_string(start) + "->" + path.substr(0, path.length() - 2);
            return path;
        }

        // Explore adjacent vertices
        for (int i = 0; i < n; ++i) {
            if (adjMatrix[current][i] == 1 && !visited[i]) {
                visited[i] = true;
                distance[i] = distance[current] + 1;
                parent[i] = current;
                q.push(i);
            }
        }
    }

    // If no path is found, return "-1"
    return "-1";
}

std::string dijkstraShortestPath(const Graph& g, int start, int end) {
    const auto& adjMatrix = g.getAdjacencyMatrix();
    int n = adjMatrix.size(); // Number of vertices in the graph

    // Initialize distance and predecessor vectors
    std::vector<int> distance(n, INT_MAX); // Initialize distances to infinity (INT_MAX)
    distance[start] = 0; // Distance from start to itself is 0
    std::vector<int> predecessor(n, -1); // Initialize predecessors to -1

    // Create a priority queue (min-heap) to efficiently select the node with the smallest distance
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
    pq.push({0, start}); // Add starting node with distance 0

    while (!pq.empty()) {
        int currentDistance = pq.top().first;
        int current = pq.top().second;
        pq.pop(); // Remove the node with the smallest distance

        // Check if we reached the destination node
        if (current == end) {
            // Reconstruct the shortest path using the predecessor vector
            std::string path = "";
            int current = end;
            while (current != -1) {
                path = std::to_string(current) + "->" + path;
                current = predecessor[current];
            }
            path = std::to_string(start) + "->" + path.substr(0, path.length() - 2);
            return path; // Return the reconstructed path
        }

        // Explore adjacent vertices
        for (int v = 0; v < n; ++v) {
            if (adjMatrix[current][v] != 0) {
                int newDistance = currentDistance + adjMatrix[current][v];
                if (newDistance < distance[v]) {
                    distance[v] = newDistance;
                    predecessor[v] = current;
                    pq.push({newDistance, v}); // Update distance and predecessor in the priority queue
                }
            }
        }
    }

    // If no path is found, return an empty string
    return "-1";
}

bool Algorithms::bfsCheckBipartite(const Graph& g, int start, std::vector<int>& color) {
    const auto& adjList = g.getAdjacencyMatrix();
    int n = adjList.size(); // Number of vertices in the graph

    // Create a queue for BFS traversal
    std::queue<int> q;

    // Initialize color of start vertex and enqueue it
    color[start] = 0; // Color 0 indicates group A, color 1 indicates group B
    q.push(start);

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        // Visit all adjacent vertices of the current vertex
        for (int neighbor = 0; neighbor < adjList[current].size(); ++neighbor) {
            // If neighbor has not been colored yet
            if (color[neighbor] == -1) {
                // Color neighbor with the opposite color of the current vertex
                color[neighbor] = 1 - color[current];
                q.push(neighbor);
            } else if (color[neighbor] == color[current]) {
                return false; // If adjacent vertices have the same color, the graph is not bipartite
            }
        }
    }

    return true; // If no adjacent vertices have the same color, the graph is bipartite
}