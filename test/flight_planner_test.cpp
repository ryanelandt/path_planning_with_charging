#include "flight_planner.h"
#include "airports.h"

#include <gtest/gtest.h>
#include <filesystem>
#include <regex>



static bool USE_FAST_TEST = true;


namespace std {


TEST(GraphCheck, RegressionTestExact) {
  const auto airport_graph = FlightPlannerExact(airports);
  ASSERT_EQ(airport_graph.numAirports(), 303);
  ASSERT_EQ(airport_graph.getNumVertices(), 9918);
  ASSERT_EQ(airport_graph.getNumEdges(), 28542);
}

TEST(GraphCheck, RegressionTestGrid_8) {
  const auto airport_graph = FlightPlannerGrid(airports, 8);
  ASSERT_EQ(airport_graph.numAirports(), 303);
  ASSERT_EQ(airport_graph.getNumVertices(), 2424);  
  ASSERT_EQ(airport_graph.getNumEdges(), 19328);
}

TEST(GraphCheck, RegressionTestGrid_32) {
  const auto airport_graph = FlightPlannerGrid(airports, 32);
  ASSERT_EQ(airport_graph.numAirports(), 303);
  ASSERT_EQ(airport_graph.getNumVertices(), 9696);  
  ASSERT_EQ(airport_graph.getNumEdges(), 77212);
}

TEST(GraphCheck, RegressionTestGrid_128) {
  const auto airport_graph = FlightPlannerGrid(airports, 128);
  ASSERT_EQ(airport_graph.numAirports(), 303);
  ASSERT_EQ(airport_graph.getNumVertices(), 38784);  
  ASSERT_EQ(airport_graph.getNumEdges(), 309080);
}

TEST(GraphCheck, RegressionTestGrid_512) {
  const auto airport_graph = FlightPlannerGrid(airports, 512);
  ASSERT_EQ(airport_graph.numAirports(), 303);
  ASSERT_EQ(airport_graph.getNumVertices(), 155136);  
  ASSERT_EQ(airport_graph.getNumEdges(), 1236666);
}


// Function to generate all possible pairs of distinct cities
std::vector<std::pair<std::string, std::string>> generateCityPairs(const std::array<row, 303>& cities) {
  std::vector<std::pair<std::string, std::string>> cityPairs;

  if (USE_FAST_TEST) {
    cityPairs.emplace_back("Manteca_CA", "Fort_Myers_FL");
    cityPairs.emplace_back("Manteca_CA", "Coeur_d'Alene_ID");
  } else {
    for (size_t i = 0; i < cities.size(); ++i) {
      for (size_t j = i + 1; j < cities.size(); ++j) {
        if (i == j) continue;  // Reference solution does not handle this case
        const string& name_i = cities[i].name;
        const string& name_j = cities[j].name;
        cityPairs.emplace_back(name_i, name_j);
      }
    }
  }

  return cityPairs;
}


// Below are examples of the output from the checker program:
//
//    Finding Path Between Manteca_CA and Fort_Myers_FL
//    Reference result: Success, cost was 67.3259
//    Candidate result: Success, cost was 66.1095
//
//    Finding Path Between Manteca_CA and Fort_Myers_FL
//    Reference result: Success, cost was 67.3259
//    Candidate result: Ran out fuel between Onalaska_WI and Sheboygan_WI
class FlightPathTester : public ::testing::Test {
 protected:
  // Helper function to execute the checker and capture its output
  std::string runTestCities(const pair<string, string>& cityPair) const {
    const string escape_city_name_1 = escapeCityName(cityPair.first);
    const string escape_city_name_2 = escapeCityName(cityPair.second);

    // TODO: detect operating system and use the appropriate checker
    std::string command = "../checker_linux \"$(./flight_planner " +
                          escape_city_name_1 + " " + escape_city_name_2 + ")\"";
    return runTestCommand(command);
  }

  std::string runTestCommand(const std::string& command) const {
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return "";

    char buffer[128];
    std::string result;
    while (!feof(pipe)) {
      if (fgets(buffer, 128, pipe) != NULL)
        result += buffer;
    }

    pclose(pipe);
    return result;
  }

  // Checks if the Candidate solution was successful
  bool checkCandidateSuccess(const std::string& output) const {
    return output.find(candidate_success_string()) != std::string::npos;
  }

  // Checks if the Reference solution was successful
  bool checkReferenceSuccess(const std::string& output) const {
    return output.find(reference_success_string()) != std::string::npos;
  }

  // Escapes the city name so that it can be passed as a command line argument.
  // This step is necessary because of the city below:
  //    Coeur_d'Alene_ID
  static std::string escapeCityName(const std::string& cityName) {
    // Replace single quotes with escaped single quotes
    std::regex singleQuoteRegex("'");
    std::string escape = std::regex_replace(cityName, singleQuoteRegex, "'\\''");

    // Add single quotes around the escape city name
    escape = "'" + escape + "'";
    return escape;
  }

  // Finds the number after searchString in the output.
  // Throws an exception if the searchString is not found.
  double findNumberAfterString(const std::string& output, const std::string& searchString) const {
    size_t candidatePos = output.find(searchString);
    if (candidatePos != std::string::npos) {
      size_t costStartPos = candidatePos + searchString.length();
      size_t costEndPos = output.find_first_of("\n", costStartPos);
      if (costEndPos != std::string::npos) {
        std::string candidateCostStr = output.substr(costStartPos, costEndPos - costStartPos);
        return std::stod(candidateCostStr);
      }
    }
    throw std::runtime_error("Could not find " + searchString + " in output");
  }

  double getReferenceCost(const std::string& output) const {
    return findNumberAfterString(output, reference_success_string());
  }
  double getCandidateCost(const std::string& output) const {
    return findNumberAfterString(output, candidate_success_string());
  }

  const string& reference_success_string() const { return reference_success_string_; }
  const string& candidate_success_string() const { return candidate_success_string_; }

 private:
  const string reference_success_string_ = "Reference result: Success, cost was ";
  const string candidate_success_string_ = "Candidate result: Success, cost was ";
};





// Test that the tester calculated travel times agree with the console output
TEST_F(FlightPathTester, TestParseOutput) {
  const string output = runTestCities({"Manteca_CA", "Fort_Myers_FL"});
  ASSERT_EQ(getReferenceCost(output), 67.3259);
  ASSERT_EQ(getCandidateCost(output), 66.1095);
}

// Test that the tester can correctly handle cities with single quotes
TEST_F(FlightPathTester, TestSingleQuote) {
  ASSERT_NO_THROW(runTestCities({"Manteca_CA", "Coeur_d'Alene_ID"}));
}


// Test that the Optimal solution is better than the reference solution for every tested case.
TEST_F(FlightPathTester, TestCityPaths) {
  const std::vector<std::pair<std::string, std::string>> cityPairs = generateCityPairs(airports);

  for (const auto& cityPair : cityPairs) {
    // Run the test program and capture the output
    std::string output = runTestCities(cityPair);

    // Checks that both the Candidate and Reference solutions were successful
    EXPECT_TRUE(checkCandidateSuccess(output));
    EXPECT_TRUE(checkReferenceSuccess(output));

    // Extract the candidate and reference costs from the output
    const double cost_ref = getReferenceCost(output);
    const double cost_cand = getCandidateCost(output);

    // Checks that the Candidate solution is at least as good as the Reference solution
    EXPECT_TRUE(cost_cand <= cost_ref);
  }
}


// Test that the Optimal solution is better than the evenly spaced solution for every tested case.
TEST(ExactVsGrid, TestFull) {
  const std::vector<std::pair<std::string, std::string>> cityPairs = generateCityPairs(airports);

  const int kNumBatteryLevels = 32;

  auto graph_exact = FlightPlannerExact(airports);
  auto graph_evenly_spaced = FlightPlannerGrid(airports, kNumBatteryLevels);

  for (const auto& cityPair : cityPairs) {
    const char* city_1 = cityPair.first.data();
    const char* city_2 = cityPair.second.data();

    double cost_exact = graph_exact.SolvePath(city_1, city_2).second;
    double cost_evenly_spaced = graph_evenly_spaced.SolvePath(city_1, city_2).second;

    EXPECT_TRUE(cost_exact <= cost_evenly_spaced);
  }
}


// Test that the solution quality of the linspaced solution improves as the number
// of battery levels increases
TEST(Linspaced, Convergence) {
  const char* city_1 = "Manteca_CA";
  const char* city_2 = "Fort_Myers_FL";

  double cost_2 = FlightPlannerGrid(airports, 2).SolvePath(city_1, city_2).second;
  double cost_4 = FlightPlannerGrid(airports, 4).SolvePath(city_1, city_2).second;
  double cost_8 = FlightPlannerGrid(airports, 8).SolvePath(city_1, city_2).second;
  double cost_16 = FlightPlannerGrid(airports, 16).SolvePath(city_1, city_2).second;
  double cost_32 = FlightPlannerGrid(airports, 32).SolvePath(city_1, city_2).second;
  double cost_64 = FlightPlannerGrid(airports, 64).SolvePath(city_1, city_2).second;
  double cost_128 = FlightPlannerGrid(airports, 128).SolvePath(city_1, city_2).second;
  double cost_256 = FlightPlannerGrid(airports, 256).SolvePath(city_1, city_2).second;

  EXPECT_LT(cost_4, cost_2);
  EXPECT_LT(cost_8, cost_4);
  EXPECT_LT(cost_16, cost_8);
  EXPECT_LT(cost_32, cost_16);
  EXPECT_LT(cost_64, cost_32);
  EXPECT_LT(cost_128, cost_64);
  EXPECT_LT(cost_256, cost_128);
}

};  // namespace std
