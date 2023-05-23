#include "../include/graph_directed.h"

#include <string>

#include <gtest/gtest.h>



namespace std {

//
//            4       5                     //
//      A----------►D----►E                 //
//      |           ▲    ◥                  //
//      |           |   /                   //
//    1 |         0 |  / 3                  //
//      |           | /                     //
//      ▼           ▼/                      //
//      B----------►C                       //
//           2                              //
//
static GraphDirected<string> makeTestGraph() {
  GraphDirected<string> graph;
  graph.AddDirectedEdge("A", "B", 1.0);
  graph.AddDirectedEdge("A", "D", 4.0);
  graph.AddDirectedEdge("B", "C", 2.0);
  graph.AddDirectedEdge("C", "E", 3.0);
  graph.AddDirectedEdge("C", "D", 0.0);
  graph.AddDirectedEdge("D", "C", 0.0);
  graph.AddDirectedEdge("D", "E", 5.0);
  return graph;
}

TEST(GraphTest, VerticesAndEdges) {
  const auto graph = makeTestGraph();

  // Verifies the number of vertices and edges
  ASSERT_EQ(graph.NumVertices(), 5);
  ASSERT_EQ(graph.NumEdges(), 7);

  // Verifies the size of the adjacency list
  ASSERT_EQ(graph.adjacency_list().size(), 4);
  ASSERT_EQ(graph.adjacency_list().at("A").size(), 2);
  ASSERT_EQ(graph.adjacency_list().at("B").size(), 1);
  ASSERT_EQ(graph.adjacency_list().at("C").size(), 2);
  ASSERT_EQ(graph.adjacency_list().at("D").size(), 2);
  ASSERT_THROW(graph.adjacency_list().at("E").size(), std::out_of_range);

  // Verifies the contents of the adjacency list
  ASSERT_EQ(graph.adjacency_list().at("A").at("B"), 1.0);
  ASSERT_EQ(graph.adjacency_list().at("A").at("D"), 4.0);
  ASSERT_EQ(graph.adjacency_list().at("B").at("C"), 2.0);
  ASSERT_EQ(graph.adjacency_list().at("C").at("E"), 3.0);
  ASSERT_EQ(graph.adjacency_list().at("C").at("D"), 0.0);
  ASSERT_EQ(graph.adjacency_list().at("D").at("C"), 0.0);
  ASSERT_EQ(graph.adjacency_list().at("D").at("E"), 5.0);
}

TEST(GraphTest, DijkstraA) {
  const auto graph = makeTestGraph();
  {
    // "A" --> "A"
    const auto dj_AA = graph.CalcMinCostPathDijkstra("A", "A");
    EXPECT_EQ(dj_AA.second, 0.0);
    EXPECT_EQ(dj_AA.first, vector<string>({"A"}));
  }
  {
    // "A" --> "B"
    const auto dj_AB = graph.CalcMinCostPathDijkstra("A", "B");
    EXPECT_EQ(dj_AB.second, 1.0);
    EXPECT_EQ(dj_AB.first, vector<string>({"A", "B"}));
  }
  {
    // "A" --> "C"
    const auto dj_AC = graph.CalcMinCostPathDijkstra("A", "C");
    EXPECT_EQ(dj_AC.second, 3.0);
    EXPECT_EQ(dj_AC.first, vector<string>({"A", "B", "C"}));
  }
  {
    // "A" --> "D"
    const auto dj_AD = graph.CalcMinCostPathDijkstra("A", "D");
    EXPECT_EQ(dj_AD.second, 3.0);
    EXPECT_EQ(dj_AD.first, vector<string>({"A", "B", "C", "D"}));
  }
  {
    // "A" --> "E"
    const auto dj_AE = graph.CalcMinCostPathDijkstra("A", "E");
    EXPECT_EQ(dj_AE.second, 6.0);
    EXPECT_EQ(dj_AE.first, vector<string>({"A", "B", "C", "E"}));
  }
}

TEST(GraphTest, DijkstraB) {
  const auto graph = makeTestGraph();
  {
    // "B" --> "A"
    const auto dj_BA = graph.CalcMinCostPathDijkstra("B", "A");
    EXPECT_EQ(dj_BA.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_BA.first, vector<string>());
  }
  {
    // "B" --> "B"
    const auto dj_BB = graph.CalcMinCostPathDijkstra("B", "B");
    EXPECT_EQ(dj_BB.second, 0.0);
    EXPECT_EQ(dj_BB.first, vector<string>({"B"}));
  }
  {
    // "B" --> "C"
    const auto dj_BC = graph.CalcMinCostPathDijkstra("B", "C");
    EXPECT_EQ(dj_BC.second, 2.0);
    EXPECT_EQ(dj_BC.first, vector<string>({"B", "C"}));
  }
  {
    // "B" --> "D"
    const auto dj_BD = graph.CalcMinCostPathDijkstra("B", "D");
    EXPECT_EQ(dj_BD.second, 2.0);
    EXPECT_EQ(dj_BD.first, vector<string>({"B", "C", "D"}));
  }
  {
    // "B" --> "E"
    const auto dj_BE = graph.CalcMinCostPathDijkstra("B", "E");
    EXPECT_EQ(dj_BE.second, 5.0);
    EXPECT_EQ(dj_BE.first, vector<string>({"B", "C", "E"}));    
  }
}


TEST(GraphTest, DijkstraC) {
  const auto graph = makeTestGraph();
  {
    // "C" --> "A"
    const auto dj_CA = graph.CalcMinCostPathDijkstra("C", "A");
    EXPECT_EQ(dj_CA.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_CA.first, vector<string>({}));
  }
  {
    // "C" --> "B"
    const auto dj_CB = graph.CalcMinCostPathDijkstra("C", "B");
    EXPECT_EQ(dj_CB.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_CB.first, vector<string>({}));
  }
  {
    // "C" --> "C"
    const auto dj_CC = graph.CalcMinCostPathDijkstra("C", "C");
    EXPECT_EQ(dj_CC.second, 0.0);
    EXPECT_EQ(dj_CC.first, vector<string>({"C"}));
  }
  {
    // "C" --> "D"
    const auto dj_CD = graph.CalcMinCostPathDijkstra("C", "D");
    EXPECT_EQ(dj_CD.second, 0.0);
    EXPECT_EQ(dj_CD.first, vector<string>({"C", "D"}));
  }
  {
    // "C" --> "E"
    const auto dj_CE = graph.CalcMinCostPathDijkstra("C", "E");
    EXPECT_EQ(dj_CE.second, 3.0);
    EXPECT_EQ(dj_CE.first, vector<string>({"C", "E"}));
  }
}

TEST(GraphTest, DijkstraD) {
  const auto graph = makeTestGraph();
  {
    // "D" --> "A"
    const auto dj_DA = graph.CalcMinCostPathDijkstra("D", "A");
    EXPECT_EQ(dj_DA.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_DA.first, vector<string>({}));
  }
  {
    // "D" --> "B"
    const auto dj_DB = graph.CalcMinCostPathDijkstra("D", "B");
    EXPECT_EQ(dj_DB.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_DB.first, vector<string>({}));
  }
  {
    // "D" --> "C"
    const auto dj_DC = graph.CalcMinCostPathDijkstra("D", "C");
    EXPECT_EQ(dj_DC.second, 0.0);
    EXPECT_EQ(dj_DC.first, vector<string>({"D", "C"}));
  }
  {
    // "D" --> "D"
    const auto dj_DD = graph.CalcMinCostPathDijkstra("D", "D");
    EXPECT_EQ(dj_DD.second, 0.0);
    EXPECT_EQ(dj_DD.first, vector<string>({"D"}));
  }
  {
    // "D" --> "E"
    const auto dj_DE = graph.CalcMinCostPathDijkstra("D", "E");
    EXPECT_EQ(dj_DE.second, 3.0);
    EXPECT_EQ(dj_DE.first, vector<string>({"D", "C", "E"}));
  }
}

TEST(GraphTest, DijkstraE) {
  const auto graph = makeTestGraph();
  {
    // "E" --> "A"
    const auto dj_EA = graph.CalcMinCostPathDijkstra("E", "A");
    EXPECT_EQ(dj_EA.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_EA.first, vector<string>({}));
  }
  {
    // "E" --> "B"
    const auto dj_EB = graph.CalcMinCostPathDijkstra("E", "B");
    EXPECT_EQ(dj_EB.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_EB.first, vector<string>({}));
  }
  {
    // "E" --> "C"
    const auto dj_EC = graph.CalcMinCostPathDijkstra("E", "C");
    EXPECT_EQ(dj_EC.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_EC.first, vector<string>({}));
  }
  {
    // "E" --> "D"
    const auto dj_ED = graph.CalcMinCostPathDijkstra("E", "D");
    EXPECT_EQ(dj_ED.second, numeric_limits<double>::infinity());
    EXPECT_EQ(dj_ED.first, vector<string>({}));
  }
  {
    // "E" --> "E"
    const auto dj_EE = graph.CalcMinCostPathDijkstra("E", "E");
    EXPECT_EQ(dj_EE.second, 0.0);
    EXPECT_EQ(dj_EE.first, vector<string>({"E"}));
  }
}

}  // namespace std
