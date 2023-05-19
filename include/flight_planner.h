#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include<functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "airports.h"
#include "graph_directed.h"


namespace std {

/// Represents the ID of a city in the airport list
class IdCity {
 public:
  explicit IdCity(int id) : id_(id) {}
  IdCity() : id_(-1) {}

  bool operator==(const IdCity& other) const { return id_ == other.id_; }
  bool operator!=(const IdCity& other) const { return !(*this == other); }
  bool operator<(const IdCity& other) const { return id_ < other.id_; }

  int id() const { return id_; }

 private:
  int id_;
};


/// Represents the aircraft's state consisting of:
///  1. The city the aircraft is in and
///  2. The battery's remaining flight time
class StateAircraft {
 public:
  StateAircraft(IdCity id_city, double battery_level_hours) :
    id_city_(id_city), battery_level_hours_(battery_level_hours) {}
  StateAircraft() : id_city_(IdCity()), battery_level_hours_(-1.0) {}

  double CalcBatteryDelta(const StateAircraft& other) const {
    assert(id_city_ == other.id_city_);
    return other.battery_level_hours_ - battery_level_hours_;
  }

  bool operator==(const StateAircraft& other) const {
    return id_city_ == other.id_city_ && battery_level_hours_ == other.battery_level_hours_;
  }
  bool operator!=(const StateAircraft& other) const { return !(*this == other); }
  bool operator<(const StateAircraft& other) const {
    if (id_city_ < other.id_city_) { return true; }
    if (id_city_ == other.id_city_ && battery_level_hours_ < other.battery_level_hours_) { return true; }
    return false;
  }

  IdCity id_city() const { return id_city_; }
  double battery_level_hours() const { return battery_level_hours_; }

 private:
  IdCity id_city_;
  double battery_level_hours_;
};


/// Represents the possible routes for the battery powered aircraft.
class FlightPlannerBase : public GraphDirected<StateAircraft> {
 public:
  explicit FlightPlannerBase(const std::array<row, 303>& airports) : airports_(airports) {}

  /// Solves for the minimum time path
  pair<vector<StateAircraft>, double> SolvePath(const char* char_src, const char* char_dst) const {
    IdCity id_src = GetIdCityFromName(char_src);
    IdCity id_dst = GetIdCityFromName(char_dst);

    // Calculate shortest distance from vertex src to every other vertex
    StateAircraft src{id_src, MaxBatteryHours()};  // Source city with fully battery
    StateAircraft dst{id_dst, MinBatteryHours()};  // Destination city
    return calcMinCostPathDijkstra(src, dst);
  }

  /// Solves for the minimum time path and prints it out
  void SolvePathAndPrint(const char* char_src, const char* char_dst) const {
    const auto [v_path, path_cost] = SolvePath(char_src, char_dst);
    PrintPath(v_path);
  }

  /// Returns the airports
  const auto& airports() const { return airports_; }

  /// Returns the number of airports
  int numAirports() const { return airports().size(); }

  /// Returns the airport with the given IdCity
  const row& getAirport(const IdCity id) const { return airports()[id.id()]; }

 protected:
  /// Adds charging edges for each city in the graph
  void AddEdgesCharge() {
    // Create a map from IdCity to the set of VertexStates that have that IdCity
    map<IdCity, set<StateAircraft>> map_id_city_to_vertex_states;
    for (auto vertex : vertices()) {
      map_id_city_to_vertex_states[vertex.id_city()].insert(vertex);
    }

    // Add charging edges for each city
    for (int i = 0; i < numAirports(); i++) {
      AddEdgesCharge(IdCity{i}, map_id_city_to_vertex_states.at(IdCity{i}));
    }
  }

  /// Calculates the flight time between two cities in hours
  double CalcFlightTimeHours(IdCity src, IdCity dst) const {
    return CalcDistKm(src, dst) / SpeedKmPerHr();
  }

  static constexpr double MinBatteryHours() { return 0.0; }
  static constexpr double MaxBatteryHours() { return 320.0 / SpeedKmPerHr(); }
  static constexpr double MaxFlightTimeHours() { return MaxBatteryHours() - MinBatteryHours(); }

 private:
  // Adds charging edges for a given city
  void AddEdgesCharge(const IdCity id_city, const set<StateAircraft>& vertex_states) {
    // Iterate over the elements of vertex_states which is a set
    for (auto it = vertex_states.begin(); it != vertex_states.end(); it++) {
      StateAircraft vertex_state = *it;
      assert(vertex_state.id_city() == id_city);  // Sanity check

      auto next_it = it;
      next_it++;
      if (next_it != vertex_states.end()) {
        StateAircraft next_vertex_state = *next_it;

        double charge_time = CalcChargeTime(vertex_state, next_vertex_state);

        // Represents charging edge
        addDirectedEdge(vertex_state, next_vertex_state, charge_time);

        // Represents free transition to zero battery level for easier graph search
        addDirectedEdge(next_vertex_state, *vertex_states.begin(), 0.0);
      }
    }
  }

  // Gets the IdCity from the name of the city
  IdCity GetIdCityFromName(const string& name) const {
    for (int i = 0; i < numAirports(); i++) {
      if (getAirport(IdCity{i}).name == name) {
        return IdCity{i};
      }
    }
    throw runtime_error("City " + name + " not found");
  }

  void PrintCity(const IdCity& id_city) const { cout << getAirport(id_city).name; }
  void PrintCity(const StateAircraft& state) const { PrintCity(state.id_city()); }

  // Prints out the path
  void PrintPath(const vector<StateAircraft>& v_path) const {
    IdCity id_src = v_path[0].id_city();
    IdCity id_dst = v_path[v_path.size() - 1].id_city();

    cout << endl;
    PrintCity(id_src); cout << ", " << endl;      // Print first city
    PrintChargingCitiesAndTimes(v_path, id_dst);  // Print out each city the plane charges at
    PrintCity(id_dst);                            // Print last city
    cout << endl;
  }

  // Prints out the cities the plane charges at and the time it charges at each city
  void PrintChargingCitiesAndTimes(const vector<StateAircraft>& v_path, IdCity id_dst) const {
    auto fn_find_last_index_of_city = [this](vector<StateAircraft> v_path, int i) {
      int j = i;
      while (j < v_path.size() && v_path[i].id_city() == v_path[j].id_city()) {
        j++;
      }
      return j - 1;
    };

    // Prints out the change time
    auto fn_print_charge_time = [this](vector<StateAircraft> v_path, int i, int j) {
      PrintCity(v_path[i]);
      double charge_time_hours = CalcChargeTime(v_path[i], v_path[j]);
      cout << ", " << setprecision(16) << charge_time_hours << "," << endl;
    };

    // Runs the lambda on each city the plane charges at
    int ind_arrive = 1;
    int ind_depart = -1;

    while (v_path[ind_arrive].id_city() != id_dst) {
      ind_depart = fn_find_last_index_of_city(v_path, ind_arrive);
      fn_print_charge_time(v_path, ind_arrive, ind_depart);
      ind_arrive = ind_depart + 1;
    }
  }

  // Returns the latitude and longitude of the city in radians
  pair<double, double> getLatLonRadians(IdCity id_city) const {
    const row& city = getAirport(id_city);
    double lat_rad = city.lat * M_PI / 180;
    double lon_rad = city.lon * M_PI / 180;
    return make_pair(lat_rad, lon_rad);
  }

  // Calculates the distance between two cities in km
  double CalcDistKm(IdCity src, IdCity dst) const {
    double lat1r, lon1r, lat2r, lon2r, u, v;

    // Get the latitude and longitude of the source and destination cities in radians
    tie(lat1r, lon1r) = getLatLonRadians(src);
    tie(lat2r, lon2r) = getLatLonRadians(dst);

    // Copilot auto-generated code
    u = sin((lat2r - lat1r) / 2);
    v = sin((lon2r - lon1r) / 2);
    return 2.0 * RadEarth() * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
  }

  // Calculates the charge time between two states in hours
  double CalcChargeTime(const StateAircraft& state_lo, const StateAircraft& state_hi) const {
    const IdCity& id = state_lo.id_city();
    assert(id == state_hi.id_city());  // Charging needs to happen at the same city

    const double flight_time_per_charge_hr = CalcChargeRateKmPerHr(id) / SpeedKmPerHr();
    return state_lo.CalcBatteryDelta(state_hi) / flight_time_per_charge_hr;
  }

  // Calculates the charge rate at a city in km/hr
  double CalcChargeRateKmPerHr(const IdCity& id_city) const { return getAirport(id_city).rate; }

  static constexpr double RadEarth() { return 6356.752; }  // Radius of Earth in km
  static constexpr double SpeedKmPerHr() { return 105.0; }  // Speed of aircraft in km/hr

  const std::array<row, 303> airports_;  // List of airports copied from the non-constant global in airports.h
};


/// Represents the possible routes for the battery powered aircraft created with the exact
/// approach described in the readme.
class FlightPlannerExact : public FlightPlannerBase {
 public:
  explicit FlightPlannerExact(const std::array<row, 303>& airports) : FlightPlannerBase(airports) {
    AddEdgesFlightsExact();
    AddEdgesCharge();
  }

 private:
  void AddEdgesFlightsExact() {
    for (int i = 0; i < numAirports(); i++) {
      for (int j = 0; j < numAirports(); j++) {
        if (i == j) { continue; }

        IdCity src = IdCity{i};
        IdCity dst = IdCity{j};

        double flight_time = CalcFlightTimeHours(src, dst);
        if (flight_time < MaxFlightTimeHours()) {
          // Flight that departs with full charge
          double max_battery = MaxBatteryHours();
          addDirectedEdge(StateAircraft{src, max_battery}, StateAircraft{dst, max_battery - flight_time}, flight_time);

          // Flight that arrives with zero charge
          double min_battery = MinBatteryHours();
          addDirectedEdge(StateAircraft{src, flight_time + min_battery}, StateAircraft{dst, min_battery}, flight_time);
        }
      }
    }
  }
};


/// Represents the possible routes for the battery powered aircraft created with the grid
/// approach described in the readme.
class FlightPlannerGrid : public FlightPlannerBase {
 public:
  FlightPlannerGrid(const std::array<row, 303>& airports, int n_levels) : FlightPlannerBase(airports),
      n_levels_(n_levels),
      v_battery_levels_(CreateGridBatteryLevels(n_levels))
      {
    AddVerticesGrid();
    AddEdgesDrivingGrid();
    AddEdgesCharge();
  }

  int n_levels() const { return n_levels_; }
  const vector<double>& v_battery_levels() const { return v_battery_levels_; }

 private:
  static vector<double> CreateGridBatteryLevels(int n_levels) {
    const double delta_battery = (MaxBatteryHours() - MinBatteryHours()) / (n_levels - 1);

    vector<double> v_battery_levels;
    for (int i = 0; i < n_levels; i++) {
      double battery_level = MinBatteryHours() + delta_battery * i;
      v_battery_levels.push_back(battery_level);
    }

    return v_battery_levels;
  }

  void AddVerticesGrid() {
    for (int i = 0; i < numAirports(); i++) {
      for (double battery_level : v_battery_levels()) {
        addVertex(StateAircraft(IdCity{i}, battery_level));
      }
    }
  }

  int findBatteryLevel(const double battery_level) const {
    const vector<double>& v_bat = v_battery_levels();

    for (int j = 1; j < n_levels(); j++) {
      int i = j - 1;
      if (v_bat[i] <= battery_level && battery_level < v_bat[j]) {
        return i;
      }
    }
    throw runtime_error("Battery level " + to_string(battery_level) + " not found");
  }

  void AddEdgesDrivingGrid() {
    for (auto vertex_i : vertices()) {
      for (int j = 0; j < numAirports(); j++) {
        if (vertex_i.id_city() == IdCity{j}) { continue; }
        double flight_time_ij = CalcFlightTimeHours(vertex_i.id_city(), IdCity{j});
        double battery_i = vertex_i.battery_level_hours();
        double battery_post = battery_i - flight_time_ij;
        double battery_min = MinBatteryHours();

        if (battery_post < battery_min) { continue; }
        int ind_lo = findBatteryLevel(battery_post);
        double battery_level_lo = v_battery_levels()[ind_lo];
        StateAircraft vertex_j_lo{IdCity{j}, battery_level_lo};
        addDirectedEdge(vertex_i, vertex_j_lo, flight_time_ij);
      }
    }
  }

  const int n_levels_;
  const vector<double> v_battery_levels_;
};

};  // namespace std
