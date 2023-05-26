#include "../include/flight_planner.h"


// FlightPlannerBase

std::pair<std::vector<StateAircraft>, double> FlightPlannerBase::SolvePath(const char* char_src,
                                                                           const char* char_dst) const {
  StateAircraft src{GetIdCity(char_src), MaxBatteryHours()};      // Source city with fully battery
  StateAircraft dst{GetIdCity(char_dst), MinBatteryHours()};      // Destination city
  const auto pair_path_cost = CalcMinCostPathDijkstra(src, dst);  // Solve for the minimum cost path

  const auto& path = pair_path_cost.first;  // The path

  // Remove unneeded vertices from the path. The example below explains why the path has unneeded vertices.
  //
  // A plane starts in city 0. It flies to city 1 and recharges before flying to city 2. The path for this
  // journey is:
  //     (city 0, 1.0)
  //     (city 1, 0.1)  **Charging starts**
  //     (city 1, 0.2)      (unneeded)
  //     (city 1, 0.3)      (unneeded)
  //     (city 1, 0.4)      (unneeded)
  //     (city 1, 0.5)  **Charging ends**
  //     (city 2, 0.0)
  //
  // Create a bool vector with all path vertices marked as true
  std::vector<bool> v_needed(path.size(), true);

  // Mark the vertices that are not needed with false
  for (int i = 1; i < (path.size() - 1); i++) {
    const auto id_prev = path[i - 1].id_city();
    const auto id_curr = path[i].id_city();
    const auto id_next = path[i + 1].id_city();
    if (id_prev == id_curr && id_curr == id_next) {
      v_needed[i] = false;
    }
  }

  // Create a new path vector with only the needed vertices
  std::vector<StateAircraft> path_filter;
  for (int i = 0; i < path.size(); i++) {
    if (v_needed[i]) {
      path_filter.push_back(path[i]);
    }
  }

  // Return the new path and the cost
  return std::make_pair(path_filter, pair_path_cost.second);
}

void FlightPlannerBase::AddEdgesCharge() {
  // Create a map from IdCity to the set of VertexStates that have that IdCity
  std::map<IdCity, std::set<StateAircraft>> map_id_city_to_vertex_states;
  for (auto vertex : vertices()) {
    map_id_city_to_vertex_states[vertex.id_city()].insert(vertex);
  }

  // Add charging edges for each city
  for (int i = 0; i < NumAirports(); i++) {
    AddEdgesCharge(IdCity{i}, map_id_city_to_vertex_states.at(IdCity{i}));
  }
}

void FlightPlannerBase::AddEdgesCharge(const IdCity& id_city, const std::set<StateAircraft>& vertex_states) {
  // Iterate over the elements of vertex_states which is a set
  for (auto it = vertex_states.begin(); it != vertex_states.end(); it++) {
    StateAircraft vertex_state = *it;
    assert(vertex_state.id_city() == id_city);  // Sanity check

    auto next_it = it;
    next_it++;
    if (next_it != vertex_states.end()) {
      StateAircraft next_vertex_state = *next_it;

      const double charge_time = CalcChargeTime(vertex_state, next_vertex_state);

      // Represents charging edge
      AddDirectedEdge(vertex_state, next_vertex_state, charge_time);

      // Represents free transition to zero battery level for easier graph search
      AddDirectedEdge(next_vertex_state, *vertex_states.begin(), 0.0);
    }
  }
}

IdCity FlightPlannerBase::GetIdCity(const std::string& name) const {
  for (int i = 0; i < NumAirports(); i++) {
    if (GetAirport(IdCity{i}).name == name) {
      return IdCity{i};
    }
  }
  throw std::runtime_error("City " + name + " not found");
}

void FlightPlannerBase::PrintPath(const std::vector<StateAircraft>& v_path) const {
  std::cout << std::endl;
  PrintCity(v_path[0]);
  PrintChargingCitiesAndTimes(v_path);  // Print out each intermediate city and charging time
  std::cout << std::endl;
}

void FlightPlannerBase::PrintChargingCitiesAndTimes(const std::vector<StateAircraft>& v_path) const {
  const IdCity& id_dst = v_path[v_path.size() - 1].id_city();  // Id of the last city

  for (int i = 1; i < v_path.size(); ++i) {
    const auto& state_prev = v_path[i - 1];
    const auto& state_curr = v_path[i];

    // If state_prev is the last city return
    if (state_prev.id_city() == id_dst) { return; }

    // If city changes print new city
    if (state_prev.id_city() != state_curr.id_city()) {
      std::cout << "," << std::endl;
      PrintCity(state_curr);

    // If city is the same print charging time
    } else {
      double charge_time_hours = CalcChargeTime(state_prev, state_curr);
      std::cout << ", " << std::setprecision(16) << charge_time_hours;
    }
  }
}

std::pair<double, double> FlightPlannerBase::GetLatLonRadians(const IdCity& id_city) const {
  const row& city = GetAirport(id_city);
  double lat_rad = city.lat * M_PI / 180;
  double lon_rad = city.lon * M_PI / 180;
  return std::make_pair(lat_rad, lon_rad);
}

double FlightPlannerBase::CalcDistKm(const IdCity& src, const IdCity& dst) const {
  const auto lat1r_lon1r = GetLatLonRadians(src);
  const double lat1r = lat1r_lon1r.first;
  const double lon1r = lat1r_lon1r.second;

  const auto lat2r_lon2r = GetLatLonRadians(dst);
  const double lat2r = lat2r_lon2r.first;
  const double lon2r = lat2r_lon2r.second;

  // Copilot auto-generated code
  const double u = sin((lat2r - lat1r) / 2);
  const double v = sin((lon2r - lon1r) / 2);
  return 2.0 * RadEarth() * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double FlightPlannerBase::CalcChargeTime(const StateAircraft& state_lo, const StateAircraft& state_hi) const {
  const IdCity& id = state_lo.id_city();
  assert(id == state_hi.id_city());  // Aircraft can't change cities while charging

  const double flight_time_per_charge_hr = CalcChargeRateKmPerHr(id) / SpeedKmPerHr();  // Called R in the readme
  return state_lo.CalcBatteryDelta(state_hi) / flight_time_per_charge_hr;
}


// FlightPlannerExact

void FlightPlannerExact::AddEdgesFlightsExact() {
  for (int i = 0; i < NumAirports(); i++) {
    for (int j = 0; j < NumAirports(); j++) {
      if (i == j) { continue; }

      IdCity src = IdCity{i};
      IdCity dst = IdCity{j};

      double flight_time = CalcFlightTimeHours(src, dst);
      if (flight_time < MaxFlightTimeHours()) {
        {
          // Flight that departs with full charge
          double max_battery = MaxBatteryHours();
          AddDirectedEdge(StateAircraft{src, max_battery}, StateAircraft{dst, max_battery - flight_time}, flight_time);
        }
        {
          // Flight that arrives with zero charge
          double min_battery = MinBatteryHours();
          AddDirectedEdge(StateAircraft{src, flight_time + min_battery}, StateAircraft{dst, min_battery}, flight_time);
        }
      }
    }
  }
}


// FlightPlannerGrid

void FlightPlannerGrid::AddVerticesGrid() {
  for (int i = 0; i < NumAirports(); i++) {
    for (double battery_level : v_battery_levels()) {
      AddVertex(StateAircraft(IdCity{i}, battery_level));
    }
  }
}

// Finds the nearest battery level without going over
int FlightPlannerGrid::FindBatteryLevel(const double battery_level) const {
  const std::vector<double>& v_bat = v_battery_levels();

  for (int j = 1; j < n_levels(); j++) {
    int i = j - 1;
    if (v_bat[i] <= battery_level && battery_level < v_bat[j]) {
      return i;
    }
  }
  throw std::runtime_error("Battery level " + std::to_string(battery_level) + " not found");
}

void FlightPlannerGrid::AddEdgesDrivingGrid() {
  for (auto vertex_i : vertices()) {
    for (int j = 0; j < NumAirports(); j++) {
      if (vertex_i.id_city() == IdCity{j}) { continue; }
      const double flight_time_ij = CalcFlightTimeHours(vertex_i.id_city(), IdCity{j});
      const double battery_i = vertex_i.battery_level_hours();
      const double battery_post = battery_i - flight_time_ij;
      const double battery_min = MinBatteryHours();

      if (battery_post < battery_min) { continue; }
      const int ind_lo = FindBatteryLevel(battery_post);
      const double battery_level_lo = v_battery_levels()[ind_lo];
      StateAircraft vertex_j_lo{IdCity{j}, battery_level_lo};
      AddDirectedEdge(vertex_i, vertex_j_lo, flight_time_ij);
    }
  }
}
