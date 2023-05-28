<div align="center">
  <strong>
    <a href="https://ryanelandt.github.io/projects/cpp_path_planning/">[ *** Click for interactive planner *** ]</a>
  </strong>
</div>

# Problem summary

This project is a solution to the minimum time path planning problem for a small electric plane as described in the file [`problem_statement.md`](problem_statement.md).
The plane has a limited range and can recharge at intermediate airports along its journey.
The plane charges faster at some city's airports than at others.
The plane leaves with a full charge.
This readme describes how to find the optimal solution to this problem.


# Running the code

To build with CMake first create a build directory:

    mkdir build && cd build

Then run CMake and make from the `build` directory:

    cmake .. && make -j4

Running this command will create two executables: `flight_planner` and `flight_planner_test`.
The instructions for running `flight_planner` are described in the [`problem_statement.md`](problem_statement.md).
The unit test can be run with the following command:
  
    ./flight_planner_test


# Problem formulation

The plane takes time doing two things: flying and charging.
Finding the minimum time path requires minimizing the sum of the flying time and the charging time.

    total time = flying time + charging time

This minimum time path is found by performing a graph search over a directed graph.

# Directed graph for path planning + charging

A directed graph is made up of vertices and directed edges.
For this problem, each vertex represents an allowed state of the plane.
The plane's state consists of city and the amount of flight time the battery has remaining.

    state = (city, battery_remaining)

Edges represent allowed transitions between states.
For this problem, there are two types of state transitions: flying and charging.
For both edge types, the cost is the transition time.

    edge cost = flight time  (flying edges)
    edge cost = charge time  (charging edges)

**Flying edges** describe transitions from one city to another.

    Flying
      (city_i, battery_remaining) --> (city_j, battery_remaining - flight time)

**Charging edges** describe increases is battery level.

    Charging
      (city_i, battery_u) --> (city_i, battery_u+)

The charging time depends on $R$, the charging rate at each city.
This term relates increases in battery level to time spent charging.
  
    R = d(battery level) / d(charging time)

The time required to charge from battery level $u^-$ to battery level $u^+$ is as follows:

    charge_time = (battery_u^+ - battery_u^-) / R.


**Shortest paths.**
The minimum time path between two states is approximately the minimum time path between the corresponding vertices in the graph.
This approximate path can be found with a shortest "path" algorithm such as Djikstra's or A*.
For specially constructed graphs, there is no approximation, and the shortest path is the minimum time path.
I discuss two graph construction methods in the next section.
The first method constructs a graph that gives an approximate solution.
The second method constructs a graph that gives the exact solution.


## Evenly-spaced battery levels (Approximate Graph)

In this approach, the range of possible battery levels is discretized into a grid of $n$ evenly spaced points.
Discretizing battery level in this way results in a tradeoff between graph size and solution quality.
This tradeoff is illustrated by the example below for the case where the maximum battery is one.

**Example:** The plane starts in city A with 1.00 battery and uses 0.63 battery to fly to city B. 

| $n$ |        Discrete levels         | Battery B </br> (true) | Battery B </br> (discrete) | Error |
|:---:|:-----------------------------:|:--------------:|:------------------:|:-----:|
|  6  | (0.00, 0.20, 0.40, ..., 1.00) | 0.37           |  0.20              | 0.17
| 11  | (0.00, 0.10, 0.20, ..., 1.00) | 0.37           |  0.30              | 0.07
| 21  | (0.00, 0.05, 0.10, ..., 1.00) | 0.37           |  0.35              | 0.02


The table above shows how the discretization error tends to decrease as $n$ increases.
The error appraoches zero as $n$ approaches infinity.
The next section describes how to get zero error with a finite sized graph.


## Exact battery levels (Optimal Graph)

This path planning problem is structured in such a way that only certain city-specific battery levels can be present in the optimal solution.
By constructing a graph that contains only these battery levels using the algorithm described below, the optimal solution is guaranteed.

**Optimal graph construction algorithm:**
The plane's flight time on a full battery is $t_{max}$.
The flight time from city $i$ to city $j$ is $t_{ij}$.
For each city $i$ and city $j$, if $t_{ij} ≤ t_{max}$, add the following edges and associated vertices to the graph:


&emsp;
&emsp;
&emsp;
(city $i$, $t_{max}$)
--> 
(city $j$, $t_{max}$ - $t_{ij}$)
&emsp;
**(Leave $i$ with full battery)**


&thinsp;
&thinsp;
&emsp;
&emsp;
&emsp;
(city $i$, $t_{ij}$)
--> 
(city $j$, $0$)
&emsp;
&emsp;
&emsp;
**(Arrive in $j$ with zero battery)**

For a graph constructed with this approach, the minimum time path found with a shortest path algorithm is the optimal solution.
The proof for this result can be found in the [proof.md](proof.md) file.

#

# Exact graph vs approximate graph comparison

I implemented both graph construction methods and compared the results.

## Implementation difficulty

The exact graph is notably easier to implement than the approximate graph.
The implementation for the approximate graph is more involved because flying transitions are inexact.
Finding the discrete battery level closest to the true battery level without going over adds an extra step.
The code contains an implementation of both graph construction methods.

## Graph size + solution quality

As shown in the table below, the exact graph has 9,918 vertices and 28,542 edges.
This vertex count is similar to the approximate graph with $n = 32$.
The edge count is similar to the approximate graph with $n = 10$.
In terms of solution quality, the exact graph is always better.
The approximate solution approaches the exact solution as $n$ goes to $\infty$.
The table below illustrates this convergence.

| Graph          | Vertex # | Edge #    | Total Time*|
|:--------------:|---------:|----------:|-----------:|
| n = 8          |  2,424   | 19,328    | 68.8800    |
| n = 32         |  9,696   | 77,212    | 67.1495    |
| n = 128        | 38,784   | 309,080   | 66.3071    |
| n = 512        | 155,136  | 1,236,666 | 66.1738    |
|                |          |           |            |
| Exact          | 9,918    | 28,542    | 66.1095    |

\* Manteca_CA to Fort_Myers_FL

