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

#include "./airports.h"
#include "./graph_directed.h"


/// Represents the ID of a city in the airport list
///
/// The purpose of this class is type checking. If this functionality was needed in a
/// larger project, it would be better to use a library like boost::strong_typedef.
class IdCity {
 public:
  explicit IdCity(int id = -1) : id_(id) {}

  /// Returns true if the two IdCity objects are equal
  bool operator==(const IdCity& other) const { return id_ == other.id_; }

  /// Returns true if the two IdCity objects are not equal
  bool operator!=(const IdCity& other) const { return !(*this == other); }

  /// Returns true if the current IdCity object is less than the other
  bool operator<(const IdCity& other) const { return id_ < other.id_; }

  /// Returns the id of the city
  int id() const { return id_; }

 private:
  int id_;  // Id of the city
};


/// Represents the aircraft's state consisting of:
///  1. The city the aircraft is in and
///  2. The battery's remaining flight time
class StateAircraft {
 public:
  StateAircraft(IdCity id_city, double battery_level_hours) :
    id_city_(id_city), battery_level_hours_(battery_level_hours) {}
  StateAircraft() : id_city_(IdCity()), battery_level_hours_(-1.0) {}

  /// Calculates the change in battery flying time from this state to another state
  ///   DeltaBattery = BatteryOther - BatteryThis
  double CalcBatteryDelta(const StateAircraft& other) const {
    assert(id_city_ == other.id_city_);
    return other.battery_level_hours_ - battery_level_hours_;
  }

  /// Checks if two StateAircraft objects are equal. Two StateAircraft objects are equal if
  /// they have the same id_city_ and battery_level_hours_.
  bool operator==(const StateAircraft& other) const {
    return id_city_ == other.id_city_ && battery_level_hours_ == other.battery_level_hours_;
  }

  /// Checks if two StateAircraft objects are not equal.
  bool operator!=(const StateAircraft& other) const { return !(*this == other); }
  
  /// Checks if the current StateAircraft object is less than the other. The ordering is:
  ///  1. IdCity
  ///  2. BatteryLevel
  bool operator<(const StateAircraft& other) const {
    if (id_city_ != other.id_city_) {
      return id_city_ < other.id_city_;
    } else {
      return battery_level_hours_ < other.battery_level_hours_;
    }
  }

  /// Returns the id of the city
  IdCity id_city() const { return id_city_; }

  /// Returns the remaining battery flight time in hours
  double battery_level_hours() const { return battery_level_hours_; }

 private:
  IdCity id_city_;  // Id of the city
  double battery_level_hours_;  // Remaining battery flight time in hours
};


/// Represents the possible routes for the battery powered aircraft.
class FlightPlannerBase : public GraphDirected<StateAircraft> {
 public:
  explicit FlightPlannerBase(const std::array<row, 303>& airports) : airports_(airports) {}

  /// Solves for the minimum time path
  std::pair<std::vector<StateAircraft>, double> SolvePath(const char* char_src, const char* char_dst) const;

  /// Solves for the minimum time path and prints it out
  void SolvePathAndPrint(const char* char_src, const char* char_dst) const {
    const auto [v_path, path_cost] = SolvePath(char_src, char_dst);
    PrintPath(v_path);
  }

  /// Returns the airports
  const auto& airports() const { return airports_; }

  /// Returns the number of airports
  int NumAirports() const { return airports().size(); }

  /// Returns the airport with the given IdCity
  const row& GetAirport(const IdCity id) const { return airports()[id.id()]; }

 protected:
  /// Adds charging edges for each city in the graph
  void AddEdgesCharge();

  /// Calculates the flight time between two cities in hours
  double CalcFlightTimeHours(const IdCity& src, const IdCity& dst) const {
    return CalcDistKm(src, dst) / SpeedKmPerHr();
  }

  /// Returns the minimum allowed battery capacity the aircraft is allowed to
  /// have in hours. This is 0.0.
  static constexpr double MinBatteryHours() { return 0.0; }

  /// Returns the maximum battery flytime in hours. The maximum flytime is the
  /// maximum flight distance 320km divided by the speed of the aircraft.
  static constexpr double MaxBatteryHours() { return 320.0 / SpeedKmPerHr(); }

  /// Returns the speed of the aircraft in km/hr
  static constexpr double MaxFlightTimeHours() { return MaxBatteryHours() - MinBatteryHours(); }

 private:
  // Adds charging edges for a given city
  void AddEdgesCharge(const IdCity& id_city, const std::set<StateAircraft>& vertex_states);

  // Gets the IdCity from the name of the city
  IdCity GetIdCity(const std::string& name) const;

  // Prints out the city name
  void PrintCity(const IdCity& id_city) const { std::cout << GetAirport(id_city).name; }

  // Prints out the city name
  void PrintCity(const StateAircraft& state) const { PrintCity(state.id_city()); }

  // Prints out the path
  void PrintPath(const std::vector<StateAircraft>& v_path) const;

  // Prints out the cities the plane charges at and the time it charges at each city
  void PrintChargingCitiesAndTimes(const std::vector<StateAircraft>& v_path) const;

  // Returns the latitude and longitude of the city in radians
  std::pair<double, double> GetLatLonRadians(const IdCity& id_city) const;

  // Calculates the distance in km between a `src` city and a `dst` city
  double CalcDistKm(const IdCity& src, const IdCity& dst) const;

  // Calculates the charge time between two states in hours
  double CalcChargeTime(const StateAircraft& state_lo, const StateAircraft& state_hi) const;

  // Calculates the charge rate at a city in km/hr
  double CalcChargeRateKmPerHr(const IdCity& id_city) const { return GetAirport(id_city).rate; }

  // Radius of the earth in km from the problem statement
  static constexpr double RadEarth() { return 6356.752; }  // 6356.752 km

  // Speed of aircraft in km/hr from the problem statement
  static constexpr double SpeedKmPerHr() { return 105.0; }  // 105.0 km/hr

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
  // Adds edges for this graph construction approach
  void AddEdgesFlightsExact();
};


/// Represents the possible routes for the battery powered aircraft created with the grid
/// approach described in the readme.
///
/// This approach is approximate and will produce longer flight times than the exact approach.
/// This approach was coded to generate content for the "Exact graph vs approximate graph comparison"
/// section of the readme. 
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

  /// Returns the number of battery levels in the grid
  int n_levels() const { return n_levels_; }

  /// Returns the vector of evenly spaced battery levels
  const std::vector<double>& v_battery_levels() const { return v_battery_levels_; }

 private:
  // Creates a vector of evenly spaced battery levels between the minimum and maximum battery levels
  static std::vector<double> CreateGridBatteryLevels(int n_levels) {
    const double delta_battery = (MaxBatteryHours() - MinBatteryHours()) / (n_levels - 1);

    std::vector<double> v_battery_levels;
    for (int i = 0; i < n_levels; i++) {
      double battery_level = MinBatteryHours() + delta_battery * i;
      v_battery_levels.push_back(battery_level);
    }

    return v_battery_levels;
  }

  // Adds vertices linearly spaced between the minimum and maximum battery levels
  void AddVerticesGrid();

  // Finds the battery level in the grid that is closest to the given battery level
  int FindBatteryLevel(const double battery_level) const;

  // Adds approximate edges for this graph construction approach
  void AddEdgesDrivingGrid();

  const int n_levels_;  // Number of evenly spaced battery levels in the grid
  const std::vector<double> v_battery_levels_;  // Vector of evenly spaced battery levels
};
