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
**Minimizing the charging time for a fixed path is a linear programming problem.**


For a fixed path, the plane leaves city 0 with a full battery and visits cities $1, 2, ..., n$ before arriving in city $n + 1$.
For this problem, I use the following notation:
- $x_i^-$: The plane's charge when it arrives in city $i$
- $x_i^+$: The plane's charge when it departs city $i$
- $d_{~i}^{~i+1}$: The charge required to get from city $i$ to $i + 1$
- $R_i$: The charging rate in city $i$

The goal of this problem is to minimize the total time spent charging as shows below:

$~~\min$ $\sum_{i=1}^{n} \frac{1}{R_i} (x_i^+ - x_i^-)$

The plane leaves city $i$ with charge $x_i^+$ and arrives in city $i + 1$ with charge $x_i^-$.
A charge of $d_{~i}^{~i+1}$ is required to get from city $i$ to city $i + 1$.
The plane leaves cities $0$ through $n$, therefore the following equality constraints below apply.

$~~d_{~i}^{~i+1} = x_{i}^+ - x_{i+1}^-$ for all $i = 0, 1, ..., n$

The plane charges at cities $1$ through $n$.
At each city, the plane's charge must satisfy three inequality constraints.
These constraints say that the plane:
1. Can't leave with more than the maximum charge
2. Can't arrive in the next city with negative charge
3. Can't charge for negative time

Mathematically, these constraints become:
1. $~~x_i^+ ≤ 1$ for all $i = 1, 2, ..., n$
2. $~~0 ≤ x_i^+ - d_{~i}^{~i+1}$ for all $i = 1, 2, ..., n$
3. $~~x_i^- ≤ x_i^+$ for all $i = 1, 2, ..., n$

The problem is therefore structured as follows:

**Minimize:**

$~~~~~~\sum_{i=1}^{n} \frac{1}{R_i} (x_i^+ - x_i^-)$

**Subject to:**

$~~~~~~d_{~i}^{~i+1} = x_{i}^+ - x_{i+1}^-$ for all $i = 0, 1, ..., n$

(1) $~~x_i^+ ≤ 1$ for all $i = 1, 2, ..., n$

(2) $~~0 ≤ x_i^+ - d_{~i}^{~i+1}$ for all $i = 1, 2, ..., n$

(3) $~~x_i^- ≤ x_i^+$ for all $i = 1, 2, ..., n$

#

## Part 2.

**This problem is feasible and bounded linear programming problem (LP).**


The problem above is a linear programming problem (LP) because the objective function and constraints are both linear.
The problem is always feasible because the plane's fixed path is made up of flights that are less than or equal to the maximum flight time.
The objective function is always bounded because the plane can only take on charge values between $0$ and $1$.
The problem is therefore a feasible and bounded LP.


## Part 3.

**Feasible and bounded linear programming problems (LPs) have optimal solutions that lie on constraint boundaries.**

The objective functions for LPs contain a constant vector $\bold{c}$ and a variable vector $\bold{x}$.
The goal is to minimize $\bold{c}^T \bold{x}$.
Each element of $\bold{c}$ gets multiplied by a corresponding element of $\bold{x}$.
If for example, an element of $\bold{c}$ is negative, then making the corresponding element of $\bold{x}$ larger will decrease the objective function.
Making the value of this element $\infty$ will make the objective function $-\infty$.
In bounded LPs this is not possible, each element of $\bold{x}$ must be bounded by at least one constraint.
Feasible and bounded linear programming problems (LPs) have optimal solutions that lie on constraint boundaries.


## Part 4.

**Minimizing charging time for a fixed city sequence requires that at each city the plane must do one of the following: 1. leave with a full battery, 2. arrive in the next city with a zero battery, 3. charge for zero time.**

As discussed earlier, the plane charges at cities $1$ through $n$.
At each city, the planes charging is constrained by inequality constraints.
These constraints say that the plane:
1. Can't leave with more than the maximum charge
2. Can't arrive in the next city with negative charge
3. Can't charge for negative time

Because this problem is a feasible and bounded LP, the optimal solution must lie on one of these constraint boundaries.
Lying on a constraint boundary means that one of the inequality constraints is satisfied as an equality constraint.
When each of the three constraints above is satisfied as an equality constraint, the plane:
1. Leaves with a full battery
2. Arrives in the next city with a zero battery
3. Charges for zero time


## Part 5. 

**The optimal city sequence will never charge for zero time.**

A city sequence cannot be optimal if it contains a city where the plane charges for zero time.
The plane could skip this city and fly directly to the next city instead.
This direct flight would be faster because the distances between cities obeys the triangle inequality.
The optimal city sequence will never contain a city where the plane charges for zero time.

The optimal city sequence, still needs to have one equality constraint active at each city.
It follows that the plane must either:
1. Leave with a full battery or
2. Arrive in the next city with a zero battery.

