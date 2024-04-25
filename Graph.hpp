#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
namespace ariel{
using namespace std;

class Graph {
public:
    Graph();
    void loadGraph(const vector<vector<int>>& graph_matrix);
    void printGraph() const;
    int count_edges() const;
    bool isDirected() const;
    int isWeighted() const; // Function added to check if the graph is weighted (1), unweighted and non-negative (0), or contains negative edges (-1)
private:
    vector<vector<int>> graph_matrix;
};
}
#endif
