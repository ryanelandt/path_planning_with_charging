
# Proof of optimality

This is a proof of optimality for the exact graph construction algorithm described in the [readme.md](readme.md) file.
This algorithm say that the only edges that need to be added to the graph are those that correspond to the plane leaving a city with a full battery and arriving in a city with a zero battery.
The transitions for these edges are below.


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
**(Arrive in $j$ with zero battery)** &nbsp; &thinsp; &ensp; &emsp;

# 

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

#

## Part 2.

**This LP is both feasible and bounded under reasonable assumptions.**

This LP is feasible assuming that the plane's fixed path is made up of flights that are less than or equal to the maximum flight time.
At all times, the plane's charge must be between $0$ and the maximum charge $t_{max}$.
These constraints are enforced by the inequality constraints (1), (2), and (3).
Bounded charge values result in a bounded objective function.
This LP is therefore both feasible and bounded.

#

## Part 3.

**Feasible and bounded LP problems have optimal solutions that lie on constraint boundaries.**

The objective function for LPs take the form $\min \mathbf{c}^T \mathbf{x}$, where $\mathbf{c}$ is a constant vector and $\mathbf{x}$ is a variable vector.
Each element of $\mathbf{c}$ gets multiplied by a corresponding element of $\mathbf{x}$.
The constraints describe a convex feasible region in which the elements of $\mathbf{x}$ must lie.
If one finds a good solution to this problem that lies inside the feasible region, then scaling this solution until it touches the boundary of the feasible region will make the solution even better.
This is a property of linear objective functions and convex feasible regions.
It follows that feasible and bounded linear programming problems (LPs) have optimal solutions that lie on constraint boundaries.

#

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

#

## Part 5. 

**Triangle inequality says charging for zero time is bad.**

Distances between cities in this problem obey the triangle inequality.
It's always faster to fly directly from one city to another than it is to stop somewhere else first.
If the plane charges for zero time, then it's faster to skip this city and fly directly to the next city instead.
The only reason to stop in a city is to charge the plane.
If for a fixed city sequence, charging for zero time is necessary to minimize charging time, then this city sequence is suboptimal.
We only care about finding optimal city sequences, so we can safely ignore zero charge time transitions.
Charging for zero time is bad because the triangle equality says so.

#

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

#