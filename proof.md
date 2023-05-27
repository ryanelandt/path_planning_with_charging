## Exact battery levels (Optimal Graph)

This path planning problem is structured in such a way that only certain city-specific battery levels can be present in the optimal solution.
By constructing a graph that contains only these battery levels using the algorithm described below, the optimal solution is guaranteed.

**Optimal graph construction algorithm:**
The plane's flight time on a full battery is $t_{max}$.
The flight time from City $i$ to City $j$ is $t_{ij}$.
For each City $i$ and City $j$, if $t_{ij} ≤ t_{max}$, add the following edges and associated vertices to the graph:

- (City $i$, $t_{max}$) --> (City $j$, $t_{max}$ - $t_{ij}$)  $~~$ **(Leave $i$ with full battery)**

- (City $i$, $t_{ij}$) --> (City $j$, 0)        $~~~~~~~~~~~~~~~~~$ **(Arrive in $j$ with zero battery)**

For a graph constructed with this approach, the minimum time path found with a shortest path algorithm is the optimal solution.
A proof of this is given below.

# Proof

## Part 1. 
**Minimizing the charging time for a fixed city sequence is a linear programming (LP) problem.**

For a fixed city sequence, the plane leaves city 0 with a full battery and visits cities $1, 2, ..., n$ before arriving in city $n + 1$.
To formulate this LP, I define the quantities below.
- $x_i^-$:$~~~~$ plane's charge arriving in city $i$
- $x_i^+$:$~~~~$ plane's charge departing city $i$
- $t_{~i}^{~i+1}$:  $~$ flight time from city $i$ to city $i + 1$
- $R_i$: $~~~~$ the charging rate in city $i$

The goal for this problem is to minimize the total time spent charging.

$~~\min$ $\sum_{i=1}^{n} \frac{1}{R_i} (x_i^+ - x_i^-)$

The plane leaves city $i$ with charge $x_i^+$ and arrives in city $i + 1$ with charge $x_{i+1}^-$.
The time required to fly from city $i$ to city $i + 1$ is $t_{~i}^{~i+1}$.
The plane leaves cities $0$ through $n$, therefore the equality constraints below must apply.

$~~t_{~i}^{~i+1} = x_{i}^+ - x_{i+1}^-$ for all $i = 0, 1, ..., n$

The plane charges at cities $1$ through $n$.
At each of these cities, the plane's charge must satisfy three inequality constraints.
These constraints say that the plane:
1. can't leave with more than the maximum charge,
2. can't arrive in the next city with negative charge, and
3. can't charge for negative time.

Mathematically, these constraints are:
1. $~~x_i^+ ≤ t_{max}~~~~~~~~~$ for all $i = 1, 2, ..., n$
2. $~~0 ≤ x_i^+ - t_{~i}^{~i+1}~~$ for all $i = 1, 2, ..., n$
3. $~~x_i^- ≤ x_i^+~~~~~~~~~~~~$ for all $i = 1, 2, ..., n$

The problem is therefore structured as follows:

**Minimize:**

$~~~~~~\sum_{i=1}^{n} \frac{1}{R_i} (x_i^+ - x_i^-)$

**Subject to:**

$~~~~~~t_{~i}^{~i+1} = x_{i}^+ - x_{i+1}^-~$ for all $i = 0, 1, ..., n$

(1) $~~x_i^+ ≤ t_{max}~~~~~~~~~~~~~$ for all $i = 1, 2, ..., n$

(2) $~~0 ≤ x_i^+ - t_{~i}^{~i+1}~~~~~~$ for all $i = 1, 2, ..., n$

(3) $~~x_i^- ≤ x_i^+~~~~~~~~~~~~~~~~$ for all $i = 1, 2, ..., n$

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

The objective function for LPs take the form $\min \bold{c}^T \bold{x}$, where $\bold{c}$ is a constant vector and $\bold{x}$ is a variable vector.
Each element of $\bold{c}$ gets multiplied by a corresponding element of $\bold{x}$.
The constraints describe a convex feasible region in which the elements of $\bold{x}$ must lie.
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

**City sequences are suboptimal if minimizing charging time means charging for zero time.**

Distances between cities in this problem obey the triangle inequality.
It's always faster to fly directly from one city to another than it is to stop somewhere else first.
If the plane charges for zero time, then it's faster to skip this city and fly directly to the next city instead.
The only reason to stop in a city is to charge the plane.
City sequences are suboptimal if minimizing charging time means charging for zero time.

## Part 6.
**Putting it all together**

To minimize the total time, one obstenibly needs to optimize over all possible city sequences and all possible charge times.
But because of the triangle inequality argument above, this isn't actually necessary.
The only city sequences that we need to optimize over are the ones that are potentially optimal.
City sequences that result in the plane charging for zero time cannot be optimal, so we don't need to even consider them.
But, minimizing the charge time still requires an active inequality constraint at each city.
So without the zero charge time constraint active, the plane must do one of the following at each city:
1. leave with a full battery, or
2. arrive in the next city with a zero battery.

This result can be used to construct a graph that is gauranteed to find the optimal solution using the following algorithm.
For each City $i$ and City $j$, if $t_{ij} ≤ t_{max}$, add the following edges and associated vertices to the graph:

- (City $i$, $t_{max}$) --> (City $j$, $t_{max}$ - $t_{ij}$)  $~~$ **(Leave $i$ with full battery)**

- (City $i$, $t_{ij}$) --> (City $j$, 0)        $~~~~~~~~~~~~~~~~~$ **(Arrive in $j$ with zero battery)**

This graph contains all possible ways to leave a city with a full battery and all possible ways to arrive in a city with a zero battery.
It therefore contains all the transitions necessary for the minimum charging time path for every possible potentially optimal city sequence.
One of these paths is the optimal solution.
Performing a shortest path search on this graph will find which one.

QED