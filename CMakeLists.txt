cmake_minimum_required (VERSION 3.15)
project (path_planner)

# GoogleTest requires at least C++14
# Structured bindings require at least C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O1")
set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(CMAKE_CURRENT_BINARY_DIR ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(FlightPathLibrary
	src/airports.cpp
	src/flight_planner.cpp
)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()


# Create solution
add_executable(flight_planner
  src/main.cpp
)
target_link_libraries(flight_planner
  FlightPathLibrary
)


# Create test
add_executable(flight_planner_test
  test/flight_planner_test.cpp
)
target_link_libraries(flight_planner_test
  FlightPathLibrary
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(flight_planner_test)

