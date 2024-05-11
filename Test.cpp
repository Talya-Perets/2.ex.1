#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"

using namespace std;

TEST_CASE("Test isWeighted")
{
  ariel::Graph g;
  // Create a graph with negative edges
  std::vector<std::vector<int>> graph1 = {
      {0, -1, 0},
      {-1, 0, -1},
      {0, -1, 0}};
  g.loadGraph(graph1);
  CHECK(g.isWeighted() == -1);

  // Create a graph without negative edges
  std::vector<std::vector<int>> graph2 = {
      {0, 1, 0},
      {1, 0, 1},
      {0, 1, 0}};
  g.loadGraph(graph2);
  CHECK(g.isWeighted() == 0);

  std::vector<std::vector<int>> graph3 = {
      {0, 2, 0},
      {-1, 0, -1},
      {0, 2, 0}};
  g.loadGraph(graph3);
  CHECK(g.isWeighted() == 1);
}

TEST_CASE("Test isDirected")
{
  ariel::Graph graph;
  std::vector<std::vector<int>> graph1 = {
      {0, 1, 0},
      {1, 0, 1},
      {0, 1, 0}};
  graph.loadGraph(graph1);
  CHECK(graph.isDirected() == 0);
  
  std::vector<std::vector<int>> graph2= {
      {0, -1, 2, -1, 4},
      {1, 0, -1, 3, -1},
      {-1, 2, 0, -1, 5},
      {3, -1, 4, 0, -1},
      {-1, 1, -1, 2, 0}};
 graph.loadGraph(graph2);
  CHECK(graph.isDirected() == true);
}

TEST_CASE("Test isConnected with directed connected graph")
{
  ariel::Graph g;
  vector<vector<int>> graph1 = {
      {0, 0, 7, 0},
      {0, 0, 6, 0},
      {0, 11, 0, 1},
      {3, 0, 0, 0}};
  ;
  g.loadGraph(graph1);
  CHECK(ariel::Algorithms::isConnected(g) == true);
  vector<vector<int>> graph2 = {
      {0, 0, 1},
      {1, 0, 0},
      {0, 1, 0},
  };
  ;
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected with directed disconnected graph")
{
  ariel::Graph g;
  vector<vector<int>> g1 = {
      {0, 3, 0, 0, 0},
      {0, 0, 4, 0, 0},
      {0, 0, 0, 2, 0},
      {4, 0, 0, 0, 5},
      {0, 0, 0, 0, 0}};
  ;
  g.loadGraph(g1);
  CHECK(ariel::Algorithms::isConnected(g) == false);
  std::vector<std::vector<int>> g2 = {
      {0, 0, 5, 0},
      {0, 0, 0, 7},
      {0, 0, 0, 1},
      {0, 7, 0, 0}};
  g.loadGraph(g2);
  CHECK(ariel::Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected for undirected connected graph")
{
  ariel::Graph g;
  // Undirected connected graph
  std::vector<std::vector<int>> undirected_connected_graph = {
      {0, 1, 1, 0, 0},
      {1, 0, 0, 1, 0},
      {1, 0, 0, 1, 0},
      {0, 1, 1, 0, 1},
      {0, 0, 0, 1, 0}};
  g.loadGraph(undirected_connected_graph);
  CHECK(ariel::Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected for undirected disconnected graph")
{
  ariel::Graph g;
  // Undirected disconnected graph
  std::vector<std::vector<int>> undirected_disconnected_graph = {
      {0, 1, 0, 0, 0},
      {1, 0, 0, 0, 0},
      {0, 0, 0, 1, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(undirected_disconnected_graph);
  CHECK(ariel::Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected with single vertex graph")
{
  ariel::Graph g;
  // Create a graph with a single vertex
  std::vector<std::vector<int>> single_vertex_graph = {{0}};
  g.loadGraph(single_vertex_graph);
  CHECK(ariel::Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected  with disconnected single edges")
{
  ariel::Graph g;
  // Graph with disconnected single edges
  std::vector<std::vector<int>> disconnected_single_edges_graph = {
      {0, 1, 0, 0, 0},
      {1, 0, 0, 0, 0},
      {0, 0, 0, 1, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(disconnected_single_edges_graph);
  CHECK(ariel::Algorithms::isConnected(g) == false);
}

TEST_CASE("Test shortestPath")
{
  ariel::Graph g;
  vector<vector<int>> graph = {
      {0, 1, 0},
      {1, 0, 1},
      {0, 1, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

  vector<vector<int>> graph2 = {
      {0, 1, 1, 0, 0},
      {1, 0, 1, 0, 0},
      {1, 1, 0, 1, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");
}


TEST_CASE("Test shortestPath graph without weights")
{
  ariel::Graph g;
  // Positive weighted graph with a path
  std::vector<std::vector<int>> g1 = {
      {0, 1, 0, 0},
      {1, 0, 0, 1},
      {0, 0, 0, 1},
      {0, 1, 1, 0}};
  g.loadGraph(g1);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->3->2");

  // Positive weighted graph without a path
  std::vector<std::vector<int>> g2 = {
      {0, 2, 4, 0, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 2, 0},
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(g2);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");
}

TEST_CASE("Test shortestPath graph with positive weights")
{
  ariel::Graph g;
  // Positive weighted graph with a path
  std::vector<std::vector<int>> g1 = {
      {0, 2, 4, 0, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 2, 0},
      {0, 0, 0, 0, 3},
      {0, 0, 0, 0, 0}};
  g.loadGraph(g1);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "0->1->2->3->4");

  // Positive weighted graph without a path
  std::vector<std::vector<int>> g2 = {
      {0, 2, 4, 0, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 2, 0},
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(g2);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");
}

TEST_CASE("Test2 shortestPath graph with positive weights")
{
  // The same directed graph and not that in a directed graph there is no route in an undirected one there is
  ariel::Graph g;
  // Positive weighted graph with a path
  std::vector<std::vector<int>> g1 = {
      {0, 1, 0, 2},
      {0, 0, 1, 0},
      {0, 0, 0, 3},
      {0, 0, 0, 0}};
  g.loadGraph(g1);
  CHECK(ariel::Algorithms::shortestPath(g, 3, 0) == "-1");

  // same graph like g1 but not direct
  std::vector<std::vector<int>> g2 = {
      {0, 1, 0, 2},
      {1, 0, 1, 0},
      {0, 1, 0, 3},
      {2, 0, 3, 0}};
  g.loadGraph(g2);
  CHECK(ariel::Algorithms::shortestPath(g, 3, 0) == "3->0");
}

TEST_CASE("Test shortestPath with negative weights")
{
  ariel::Graph g;
  // Graph with negative weights with un negetive cicle
  std::vector<std::vector<int>> graph_with_negative_weights = {
      {0, 3, 0, 0, 0},
      {0, 0, -1, 0, 0},
      {0, 0, 0, -2, 0},
      {4, 0, 0, 0, 5},
      {0, 0, 0, 0, 0}};
  g.loadGraph(graph_with_negative_weights);
  CHECK(ariel::Algorithms::BellmanFord(g, 0, 4) == "0->1->2->3->4");
}

TEST_CASE("Test shortestPath with negative weights2")
{
  ariel::Graph g;
  // Graph with negative weights
  std::vector<std::vector<int>> graph = {
      {0, 1, -5, 1},
      {1, 0, 3, 0},
      {-5, 3, 0, 0},
      {1, 0, 0, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 3) == "-1");
}
TEST_CASE("Test shortestPath with negative weights3")
{
  ariel::Graph g;
  // Graph with negative weights
  std::vector<std::vector<int>> graph = {
      {0, 0, 1, -2},
      {-5, 0, 0, 0},
      {0, 0, 0, 2},
      {2, 0, 0, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 3) == "0->3");
}

TEST_CASE("Test isContainsCycle")
{
  ariel::Graph g;
  vector<vector<int>> graph = {
      {0, 1, 0},
      {1, 0, 1},
      {0, 1, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

  vector<vector<int>> graph2 = {
      {0, 1, 1, 0, 0},
      {1, 0, 1, 0, 0},
      {1, 1, 0, 1, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::isContainsCycle(g) == "0 -> 2 -> 1 -> 0");
}

TEST_CASE("Test isBipartite")
{
  ariel::Graph g;
  vector<vector<int>> graph = {
      {0, 1, 0},
      {1, 0, 1},
      {0, 1, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}");

  vector<vector<int>> graph2 = {
      {0, 1, 1, 0, 0},
      {1, 0, 1, 0, 0},
      {1, 1, 0, 1, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::isBipartite(g) == "0");
}

TEST_CASE("Test isBipartite")
{
  ariel::Graph g;
  vector<vector<int>> graph = {
      {0, 1, 0},
      {1, 0, 1},
      {0, 1, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}");

  vector<vector<int>> graph2 = {
      {0, 1, 1, 0, 0},
      {1, 0, 1, 0, 0},
      {1, 1, 0, 1, 0},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::isBipartite(g) == "0");
}

TEST_CASE("Test negativeCycle")
{
  ariel::Graph g;

  // Test case 1: Graph with negative cycle
  vector<vector<int>> graph1 = {
      {0, -1, 0},
      {0, 0, -1},
      {-1, 0, 0}};
  g.loadGraph(graph1);
  CHECK(ariel::Algorithms::negativeCycle(g) == true);

  // Test case 2: Graph without negative cycle
  vector<vector<int>> graph2 = {
      {0, 1, 1},
      {1, 0, 1},
      {1, 1, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test negativeCycle2")
{
  ariel::Graph g;

  // Test case 1: Graph with negative cycle
  vector<vector<int>> graph1 = {
      {0, 1, 0, 0, 1},
      {0, 0, 3, 0, 0},
      {-1, 0, 0, 0, 0},
      {0, 0, 0, 0, -5},
      {0, 0, 0, 2, 0}};
  g.loadGraph(graph1);
  CHECK(ariel::Algorithms::negativeCycle(g) == true);
}
TEST_CASE("Test negative cycle detection")
{
  ariel::Graph g;
  // Graph with a negative cycle
  std::vector<std::vector<int>> negative_cycle_graph = {
      {0, -1, 0, 0, 0},
      {0, 0, -2, 0, 0},
      {-3, 0, 0, 0, 0},
      {0, 0, 0, 0, -4},
      {0, 0, 0, -5, 0}};
  g.loadGraph(negative_cycle_graph);
  bool has_negative_cycle = ariel::Algorithms::negativeCycle(g);
  // Ensure the negative cycle is detected
  CHECK(has_negative_cycle == true);
}
TEST_CASE("Test negative cycle detection")
{
  ariel::Graph g;
  // Graph with a negative cycle
  std::vector<std::vector<int>> graph = {
      {0, -5, 1},
      {-5, 0, 1},
      {1, 1, 0}};
  g.loadGraph(graph);
  // Ensure the negative cycle is detected
  CHECK(ariel::Algorithms::negativeCycle(g) == true);
}

TEST_CASE("Test negative cycle detection with additional graph")
{
  ariel::Graph g;
  // A graph with two circles, one negative and one not
  std::vector<std::vector<int>> graph = {
      {0, -1, 0, 0, 0, 0},
      {0, 0, 1, 0, 0, 0},
      {1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 2},
      {0, 0, 0, -1, 0, 0},
      {0, 0, 0, 2, -5, 0}};
  g.loadGraph(graph);
  // Ensure the additional negative cycle is detected
  CHECK(ariel::Algorithms::negativeCycle(g) == true);
}
TEST_CASE("Test invalid graph")
{
  ariel::Graph g;
  vector<vector<int>> graph = {
      {0, 1, 2, 0},
      {1, 0, 3, 0},
      {2, 3, 0, 4},
      {0, 0, 4, 0},
      {0, 0, 0, 5}};
  CHECK_THROWS(g.loadGraph(graph));
}
