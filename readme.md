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
The proof for this result can be found in the "Proof of Optimality" section below.


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

#

# Proof of optimality

This proof has six parts. &nbsp; &thinsp; &ensp; &emsp;

## Part 1. 
**Minimizing the charging time for a fixed city sequence is a linear programming (LP) problem.**

For a fixed city sequence, the plane leaves city 0 with a full battery and visits cities $1, 2, ..., n$ before arriving in city $n + 1$.
To formulate this LP, I define the quantities below.




- $x_i^-$: 
&emsp; &ensp; 
plane's charge arriving in city $i$
- $x_i^+$: 
&emsp; &nbsp;&nbsp;
plane's charge departing city $i$
- $t_{ ~ i}^{ ~ i+1}$:
&emsp;
flight time from city $i$ to city $i + 1$
- $R_i$: 
&emsp; &nbsp; &nbsp;
the charging rate in city $i$


The goal for this problem is to minimize the total time spent charging.

&ensp; &ensp; &ensp;
$\min$ $\sum \frac{1}{R_i} (x_i^+ - x_i^-)$
&emsp;
for $i$ = $1, 2, ..., n$


The plane leaves city $i$ with charge $x_i^+$ and arrives in city $i + 1$ with charge $x_{i+1}^-$.
The time required to fly from city $i$ to city $i + 1$ is $t_{i}^{i+1}$.
The plane leaves cities $0$ through $n$, therefore the equality constraints below must apply.

&emsp; &ensp;
$t_{ i }^{ i+1 } = x_{ i }^+ - x_{ i+1 }^-$ for all $i = 0, 1, ..., n$


The plane charges at cities $1$ through $n$.
At each of these cities, the plane's charge must satisfy three inequality constraints.
These constraints say that the plane:
1. can't leave with more than the maximum charge,
2. can't arrive in the next city with negative charge, and
3. can't charge for negative time.


Mathematically, these constraints are:
1. &ensp; $x_i^+ ≤ t_{max}$ &emsp; &nbsp; &nbsp; &nbsp; for all $i = 1, 2, ..., n$
2. &ensp; $0 ≤ x_i^+ - t_{i}^{i+1}$ &nbsp; &nbsp; for all $i = 1, 2, ..., n$
3. &ensp; $x_i^- ≤ x_i^{+}$ &emsp;&emsp;&emsp;&nbsp; for all $i = 1, 2, ..., n$

The problem is therefore structured as follows:



**Minimize:**

&ensp; &ensp; &ensp;
$\sum \frac{1}{R_i} (x_i^+ - x_i^-)$
&emsp;
for $i$ = $1, 2, ..., n$

**Subject to:**

&ensp; &ensp; &ensp; $t_{ ~ i}^{ ~ i+1} = x_{i}^+ - x_{i+1}^{-}$ &nbsp; for all $i = 0, 1, ..., n$

(1) &ensp; $x_i^+ ≤ t_{max} $
&emsp; &ensp; &ensp; &ensp; &nbsp;
for all $i = 1, 2, ..., n$

(2) &ensp; $0 ≤ x_i^+ - t_{i}^{i+1}$
&emsp; &ensp; &nbsp;
for all $i = 1, 2, ..., n$

(3) &ensp; $x_i^- ≤ x_i^{+}$
&emsp; &ensp; &ensp; &ensp; &ensp; &ensp;
for all $i = 1, 2, ..., n$


This problem has a linear objective function. 
It has linear equality constraints and linear inequality constraints.
The problem is therefore a linear programming (LP) problem.


## Part 2.

**This LP is both feasible and bounded under reasonable assumptions.**

This LP is feasible assuming that the plane's fixed path is made up of flights that are less than or equal to the maximum flight time.
At all times, the plane's charge must be between $0$ and the maximum charge $t_{max}$.
These constraints are enforced by the inequality constraints (1), (2), and (3).
Bounded charge values result in a bounded objective function.
This LP is therefore both feasible and bounded.


## Part 3.

**Feasible and bounded LP problems have optimal solutions that lie on constraint boundaries.**

The objective function for LPs take the form $\min \mathbf{c}^T \mathbf{x}$, where $\mathbf{c}$ is a constant vector and $\mathbf{x}$ is a variable vector.
Each element of $\mathbf{c}$ gets multiplied by a corresponding element of $\mathbf{x}$.
The constraints describe a convex feasible region in which the elements of $\mathbf{x}$ must lie.
If one finds a good solution to this problem that lies inside the feasible region, then scaling this solution until it touches the boundary of the feasible region will make the solution even better.
This is a property of linear objective functions and convex feasible regions.
It follows that feasible and bounded linear programming problems (LPs) have optimal solutions that lie on constraint boundaries.


## Part 4.

**Minimizing charging time for a fixed city sequence requires the plane do one of the following at each city: 1. leave with a full battery, 2. arrive in the next city with a zero battery, 3. charge for zero time.**

As discussed earlier, the plane charges at cities $1$ through $n$.
Inequality constraints at each city say that the plane:
1. can't leave with more than the maximum charge,
2. can't arrive in the next city with negative charge, and
3. can't charge for negative time.

Because this problem is a feasible and bounded LP, the optimal solution must lie on one of these constraint boundaries.
When the solution lies on a constraint boundary, the constraint is satisfied as an equality constraint.
For this problem, lying on a constraint boundary means that at each city the plane does one of the following:
1. leaves with a full battery, 
2. arrives in the next city with a zero battery, or 
3. charges for zero time.


## Part 5. 

**Triangle inequality says charging for zero time is bad.**

Distances between cities in this problem obey the triangle inequality.
It's always faster to fly directly from one city to another than it is to stop somewhere else first.
If the plane charges for zero time, then it's faster to skip this city and fly directly to the next city instead.
The only reason to stop in a city is to charge the plane.
If for a fixed city sequence, charging for zero time is necessary to minimize charging time, then this city sequence is suboptimal.
We only care about finding optimal city sequences, so we can safely ignore zero charge time transitions.
Charging for zero time is bad because the triangle equality says so.



## Part 6.
**Putting it all together**

To minimize the total time, one obstenibly needs to optimize over all possible city sequences and all possible charge times.
But because of the triangle inequality argument above, this isn't actually necessary.
The only city sequences that we need to optimize over are the ones that are potentially optimal.
City sequences that result in the plane charging for zero time cannot be optimal, so we don't need to even consider them.


Minimizing the charge time still requires an active inequality constraint at each city.
So without the zero charge time constraint active, the plane must do one of the following at each city:
1. leave with a full battery, or
2. arrive in the next city with a zero battery.

This result can be used to construct a graph that is gauranteed to find the optimal solution using the following algorithm.
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


This graph contains all possible ways to leave a city with a full battery and all possible ways to arrive in a city with a zero battery.
It therefore contains all the transitions necessary for the minimum charging time paths for every possible potentially optimal city sequence.
One of these paths is the optimal solution.
A shortest path search on this graph will find out which one it is. &emsp; $\Box$




