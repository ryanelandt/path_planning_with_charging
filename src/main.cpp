#include "../include/airports.h"
#include "../include/flight_planner.h"


int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Invalid number of arguments" << std::endl;
    return 1;
  }

  FlightPlannerExact(airports).SolvePathAndPrint(argv[1], argv[2]);  // Run the exact planner
  // FlightPlannerGrid(airports, 4).SolvePathAndPrint(argv[1], argv[2]);  // Run the grid planner

  return 0;
}
