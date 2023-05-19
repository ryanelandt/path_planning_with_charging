
# Problem summary

This project is a solution to the minimum time path planning problem for a small electric plane as described in the file `problem_statement.md`.
The plane has a limited range and can recharge at intermediate airports along its journey.
The plane charges faster at some airports than at others.
The plane leaves with a full charge.
This readme describes how to find the optimal solution to this problem.


# Running the code

To build with CMake first create a build directory:

    mkdir build && cd build

Then run CMake and make:

    cmake .. && make -j4

Running this command will create two executables: `flight_planner` and `flight_planner_test`.
The instructions for running `flight_planner` are described in the `problem_statement.md`.
You can run the unit test with the following command;
  
      ./flight_planner_test


# Problem formulation

The plane takes time doing two things: flying and charging.
To find the minimum time path, one needs to minimize the sum of the flying time and the charging time.

    total time = flying time + charging time

This minimum time path is found with the help of a directed graph.

# Directed Graph for path planning + charging

A directed graph is made up of vertices and directed edges.
For this problem, each vertex represents an allowed state of the plane.
The plane's state is made up of an airport and the amount of flight time the battery has remaining.

    state = (airport, battery_remaining)

Edges represent allowed transitions between states.
For this problem, the edge cost is the time a transition takes to complete.

    edge cost = time

This problem has two types of transitions: flying and charging.
The flying transition is described below.

    Flying
      (airport_i, battery_remaining) --> (airport_j, battery_remaining - flight time)
      edge cost = flight time

Each airport has a charging speed R.
If R=2, this means that one hour of charging time gives the plane two hours of flight time.
The charging transition is described below.

    Charging:
      (airport_i, battery_initial) --> (airport_i, battery_initial + Ri * charge time)
      edge cost = change time

The minimum time path between two states is approximately the minimum cost path between the corresponding vertices in the graph.
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

| $n$ |        Range                  | True battery B | Discrete battery B | Error
|-----|-------------------------------|----------------|--------------------|----------------------
|  6  | (0.00, 0.20, 0.40, ..., 1.00) | 0.37           |  0.20              | 0.17
| 11  | (0.00, 0.10, 0.20, ..., 1.00) | 0.37           |  0.30              | 0.07
| 21  | (0.00, 0.05, 0.10, ..., 1.00) | 0.37           |  0.35              | 0.02

The table above shows how the discretization error tends to decrease as $n$ increases.
The error appraoches zero as $n$ approaches infinity.
The next section describes how to get zero error with a finite sized graph.


## Exact battery levels (Optimal Graph)

This path planning problem is structured in such a way that only certain city-specific battery levels can be present in the optimal solution.
By constructing a graph that contains only these battery levels using the algorithm described below, the optimal solution is gauranteed.

**Optimal graph construction algorithm:**
The plane's flight time on a full battery is $t_{max}$.
The flight time from City $i$ to City $j$ is $t_{ij}$.
For each City $i$ and City $j$, if $t_{ij} ≤ t_{max}$, add the following edges and associated vertices to the graph:

- (City $i$, $t_{max}$) --> (City $j$, $t_{max}$ - $t_{ij}$)  $~~$ **(Leave $i$ with full battery)**

- (City $i$, $t_{ij}$) --> (City $j$, 0)        $~~~~~~~~~~~~~~~~~$ **(Arrive in $j$ with zero battery)**

For a graph constructed with this approach, the minimum time path found with a shortest path algorithm is the optimal solution.
The sequence of ASCII art drawings below explain why this is true.


#
### Case 1:
The plane flys directly to the destination city.
- City 0→1, the plane **leaves city 0 with a full charge** and arrives in city 1.
```
 Max   ___       ___                                      
        |\_       |                                       
        |  \_     |                                       
        |    \_   |                                       
        |      \_ |                                       
Charge  |        \|                                       
        |         |                                       
        |         |                                       
  0    _|_       _|_                                      
     City 0     City 1                                    
```
#
### Case 2:

The plane recharges once.
- City 0→1, the plane **leaves city 0 with a full charge** and arrives in city 1.
- City 1→2, the plane recharges in city 1 just enough to **arrive in city 2 with zero charge**.
```
 Max   ___       ___         ___                          
        |\_       |           |                           
        |  \_     |           |                           
        |    \_   |\_         |                           
        |      \_ |  \_       |                           
Charge  |        \|    \_     |                           
        |         |      \_   |                           
        |         |        \_ |                           
  0    _|_       _|_         \|_                          
     City 0     City 1       City 2                       
```

#
### Case 3A (R1 ≤ R2): 

The plane recharges twice.
- City 0→1, the plane **leaves city 0 with a full charge** and arrives in city 1.
- City 1→2, the plane recharges in city 1 before departing for city 2. The plane charges just enough to **arrive in city 2 with zero charge** because the charger in city 2 is faster.
- City 2→3, the plane recharges in city 2 just enough to **arrive in city 3 with zero charge**.

```
                  R1     ≤    R2                          
 Max   ___       ___         ___         ___              
        |\_       |           |           |               
        |  \_     |           |           |               
        |    \_   |\_         |\_         |               
        |      \_ |  \_       |  \_       |               
Charge  |        \|    \_     |    \_     |               
        |         |      \_   |      \_   |               
        |         |        \_ |        \_ |               
  0    _|_       _|_         \|_         \|_              
     City 0     City 1       City 2        City 3         
```

#
### Case 3B (R1 ≥ R2): 



The plane recharges twice.
- City 0→1, the plane **leaves city 0 with a full charge** and arrives in city 1.
- City 1→2, the plane recharges in city 1 before departing for city 2. The plane recharges fully so it can **leave city 1 with a full charge** because the charger in city 2 is slower.
- City 2→3, the plane recharges in city 2 just enough to **arrive in city 3 with zero charge**.

```
                  R1     ≥    R2                          
       ___       ___         ___         ___              
  Max   |\_       |\_         |           |               
        |  \_     |  \_       |           |               
        |    \_   |    \_     |\_         |               
        |      \_ |      \_   |  \_       |               
Charge  |        \|        \_ |    \_     |               
        |         |          \|      \_   |               
        |         |           |        \_ |               
  0    _|_       _|_          |_        \_|_              
     City 0     City 1       City 2       City 3          
```

#

# Exact graph vs approximate graph comparison

I implemented both graph construction methods and compared the results.

## Implementation difficulty

The exact graph is notably easier to implement than the approximate graph.
The implementation for the approximate graph is more involved because flying transitions are inexact.
Finding the discrete battery level closest to the true battery level without going over adds an extra step.

## Graph size + solution quality

The exact graph size is fixed, and is comparable to an approximate graph when $8 ≤ n ≤ 32$ as shown in the table below.
For the graph that uses a approximate of $n$ battery levels, the flight time decreases as $n$ increases.
The decreasing flying times approach the flying time of the exact solution.

| Battery Levels | Vertex # | Edge #    | Drive Time*|
|----------------|----------|-----------|------------|
| n = 8          |  2,424   | 19,328    | 68.8800    |
| n = 32         |  9,696   | 77,212    | 67.1495    |
| n = 128        | 38,784   | 309,080   | 66.3071    |
| n = 512        | 155,136  | 1,236,666 | 66.1738    |
|                |          |           |            |
| Exact          | 9,918    | 28,542    | 66.1095    |

\* Manteca_CA to Fort_Myers_FL






