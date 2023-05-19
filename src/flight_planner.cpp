#include "flight_planner.h"


namespace std {

// FlightPlannerBase

pair<vector<StateAircraft>, double> FlightPlannerBase::SolvePath(const char* char_src, const char* char_dst) const {
  IdCity id_src = GetIdCityFromName(char_src);
  IdCity id_dst = GetIdCityFromName(char_dst);

  // Calculate shortest distance from vertex src to every other vertex
  StateAircraft src{id_src, MaxBatteryHours()};  // Source city with fully battery
  StateAircraft dst{id_dst, MinBatteryHours()};  // Destination city
  return calcMinCostPathDijkstra(src, dst);
}

void FlightPlannerBase::AddEdgesCharge() {
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

void FlightPlannerBase::AddEdgesCharge(const IdCity id_city, const set<StateAircraft>& vertex_states) {
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

IdCity FlightPlannerBase::GetIdCityFromName(const string& name) const {
  for (int i = 0; i < numAirports(); i++) {
    if (getAirport(IdCity{i}).name == name) {
      return IdCity{i};
    }
  }
  throw runtime_error("City " + name + " not found");
}

void FlightPlannerBase::PrintPath(const vector<StateAircraft>& v_path) const {
  IdCity id_src = v_path[0].id_city();
  IdCity id_dst = v_path[v_path.size() - 1].id_city();

  cout << endl;
  PrintCity(id_src); cout << ", " << endl;      // Print first city
  PrintChargingCitiesAndTimes(v_path, id_dst);  // Print out each city the plane charges at
  PrintCity(id_dst);                            // Print last city
  cout << endl;
}

void FlightPlannerBase::PrintChargingCitiesAndTimes(const vector<StateAircraft>& v_path, IdCity id_dst) const {
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

pair<double, double> FlightPlannerBase::getLatLonRadians(IdCity id_city) const {
  const row& city = getAirport(id_city);
  double lat_rad = city.lat * M_PI / 180;
  double lon_rad = city.lon * M_PI / 180;
  return make_pair(lat_rad, lon_rad);
}

double FlightPlannerBase::CalcDistKm(IdCity src, IdCity dst) const {
  double lat1r, lon1r, lat2r, lon2r, u, v;

  // Get the latitude and longitude of the source and destination cities in radians
  tie(lat1r, lon1r) = getLatLonRadians(src);
  tie(lat2r, lon2r) = getLatLonRadians(dst);

  // Copilot auto-generated code
  u = sin((lat2r - lat1r) / 2);
  v = sin((lon2r - lon1r) / 2);
  return 2.0 * RadEarth() * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double FlightPlannerBase::CalcChargeTime(const StateAircraft& state_lo, const StateAircraft& state_hi) const {
  const IdCity& id = state_lo.id_city();
  assert(id == state_hi.id_city());  // Charging needs to happen at the same city

  const double flight_time_per_charge_hr = CalcChargeRateKmPerHr(id) / SpeedKmPerHr();
  return state_lo.CalcBatteryDelta(state_hi) / flight_time_per_charge_hr;
}


// FlightPlannerExact

void FlightPlannerExact::AddEdgesFlightsExact() {
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


// FlightPlannerGrid

void FlightPlannerGrid::AddVerticesGrid() {
  for (int i = 0; i < numAirports(); i++) {
    for (double battery_level : v_battery_levels()) {
      addVertex(StateAircraft(IdCity{i}, battery_level));
    }
  }
}

int FlightPlannerGrid::findBatteryLevel(const double battery_level) const {
  const vector<double>& v_bat = v_battery_levels();

  for (int j = 1; j < n_levels(); j++) {
    int i = j - 1;
    if (v_bat[i] <= battery_level && battery_level < v_bat[j]) {
      return i;
    }
  }
  throw runtime_error("Battery level " + to_string(battery_level) + " not found");
}

void FlightPlannerGrid::AddEdgesDrivingGrid() {
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

}  // namespace std
