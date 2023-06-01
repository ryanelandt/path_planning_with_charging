#include <emscripten.h>

#include "../include/airports.h"
#include "../include/flight_planner.h"

#include <iostream>
#include <vector>
#include <tuple>
#include <array>
#include <cassert>
#include <cmath>


using namespace std;


// NOTE: To compile this into a wasm file, run:
//
// emcc -std=c++17 src/*.cpp -Iinclude -o flight_planner.js -s EXPORTED_FUNCTIONS="['getValue', '_malloc', '_free']" -s NO_EXIT_RUNTIME=1 -s EXPORTED_RUNTIME_METHODS="['ccall', 'cwrap']" -s WASM=1 -s MODULARIZE -s EXPORT_ES6=1 -O3


int main() {
  return 0;
}

extern "C" {
  EMSCRIPTEN_KEEPALIVE
  void getCityName(double* data, int* length, int i) {
    // Get the name of the city
    const string& name = airports[i].name;

    // Write the name into data
    for (size_t i = 0; i < name.size(); ++i) {
      data[i] = name[i];
    }

    *length = name.size();

    return;
  }
}

// Writes the cartesian position of all airports into data
extern "C" {
  EMSCRIPTEN_KEEPALIVE
  void voidAirportXyz(double* data, int* length) {
    // Lambda to calculate the cartesian position of an airport
    auto calcCartesian = [](const row& r) {
      const double lat_r = r.lat * M_PI / 180.0;
      const double lon_r = r.lon * M_PI / 180.0;
      const double x = cos(lat_r) * cos(lon_r);
      const double y = cos(lat_r) * sin(lon_r);
      const double z = sin(lat_r);
      return make_tuple(x, y, z);
    };

    size_t index = 0;
    for (const auto& airport : airports) {
      const auto xyz = calcCartesian(airport);
      data[index++] = get<0>(xyz);
      data[index++] = get<1>(xyz);
      data[index++] = get<2>(xyz);
    }
    *length = index;
    return;   
  }
}

// Create a global
FlightPlannerExact flight_planner_exact(airports);

// Writes the city ids of the path from src to dest into data
extern "C" {
  EMSCRIPTEN_KEEPALIVE
  void voidPathOutReal(double* data, int* length, int id_src, int id_dest) {
    const string& name_src = airports[id_src].name;
    const string& name_dest = airports[id_dest].name;
    const auto [v_path, path_cost] = flight_planner_exact.SolvePath(name_src.data(), name_dest.data()); 

    size_t index = 0;
    for (const auto& state : v_path) {
      data[index++] = state.id_city().id();
    }
    *length = index;
    return;   
  }
}
