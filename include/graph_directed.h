#pragma once

#include <functional>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>
#include <limits>


namespace std {

/// Graph class for a directed graph with vertices of type VertexType
template <typename VertexType>
class GraphDirected {
 public:
  GraphDirected() = default;                                 // Default constructor to create an empty graph
  GraphDirected(const GraphDirected&) = default;             // Copy constructor
  GraphDirected(GraphDirected&&) = default;                  // Move constructor
  GraphDirected& operator=(const GraphDirected&) = default;  // Copy assignment operator
  GraphDirected& operator=(GraphDirected&&) = default;       // Move assignment operator
  ~GraphDirected() = default;                                // Destructor

  /// Adds a vertex to the graph if it doesn't already exist, otherwise has no effect
  void AddVertex(const VertexType& v) { vertices_.insert(v); }

  /// Add a directed edge to the graph from vertex src to vertex dst with weight w
  void AddDirectedEdge(const VertexType& src, const VertexType& dst, double w) {
    AddVertex(src);
    AddVertex(dst);
    assert(0.0 <= w);             // Assert edge cost is non-negative
    adjacency_list_[src][dst] = w;  // Add edge to the adjacency list
  }

  /// Returns the number of vertices in the graph
  int NumVertices() const { return vertices().size(); }

  /// Returns the number of edges in the graph
  int NumEdges() const {
    int numEdges = 0;
    for (const auto &v_it : adjacency_list()) {
      numEdges += v_it.second.size();
    }
    return numEdges;
  }

  /// Perform Dijkstra's algorithm to find the shortest path between vertices src and dst
  pair<vector<VertexType>, double> CalcMinCostPathDijkstra(const VertexType& src, const VertexType& dst) const {
    // Initialize distances, visited array, and parent map
    map<VertexType, double> dist;
    map<VertexType, bool> visited;
    map<VertexType, VertexType> parent;

    for (const VertexType& v : vertices()) {
      dist[v] = numeric_limits<double>::infinity();
      visited[v] = false;
      parent[v] = VertexType();
    }

    dist[src] = 0;

    // Priority queue to store vertices and their distances
    priority_queue<pair<double, VertexType>, vector<pair<double, VertexType>>, greater<pair<double, VertexType>>> pq;

    // Add the source vertex to the priority queue
    pq.emplace(dist[src], src);

    // Perform Dijkstra's algorithm
    while (!pq.empty()) {
      VertexType u = pq.top().second;
      pq.pop();

      if (visited[u]) { continue; }
      visited[u] = true;

      for (const auto &vertex_and_edge_cost : adjacency_list().at(u)) {
        const auto& [v, cost_uv] = vertex_and_edge_cost;
        double dist_alt = dist[u] + cost_uv;
        if (!visited[v] && dist_alt < dist[v]) {
          dist[v] = dist_alt;
          parent[v] = u;  // Update parent of v
          pq.emplace(dist[v], v);
        }
      }
    }

    // Reconstruct the shortest path
    vector<VertexType> path;
    VertexType current = dst;
    while (current != src) {
      path.push_back(current);
      current = parent[current];
      if (current == VertexType()) {
        // The current vertex has no parent, which means that it is not
        // reachable from the source vertex, and hence there is no valid path
        return make_pair(vector<VertexType>(), numeric_limits<double>::infinity());
      }
    }

    path.push_back(src);
    reverse(path.begin(), path.end());
    return make_pair(path, dist[dst]);  // Return the shortest path and its cost
  }

  /// Returns the set of vertices in the graph
  const set<VertexType>& vertices() const { return vertices_; }

  /// Returns the adjacency list of the graph
  const map<VertexType, map<VertexType, double>>& adjacency_list() const { return adjacency_list_; }

 private:
  set<VertexType> vertices_;  // List of vertices in the graph
  map<VertexType, map<VertexType, double>> adjacency_list_;  // Adjacency list of the graph
};

};  // namespace std
