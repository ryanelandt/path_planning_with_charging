#include "airports.h"
#include "flight_planner.h"


using namespace std;


int main(int argc, char** argv) {
  if (argc != 3) {
    cout << "Invalid number of arguments" << endl;
    return 1;
  }

  FlightPlannerExact(airports).SolvePathAndPrint(argv[1], argv[2]);  // Run the exact planner
  // FlightPlannerGrid(airports, 4).SolvePathAndPrint(argv[1], argv[2]);  // Run the grid planner

  return 0;
};
