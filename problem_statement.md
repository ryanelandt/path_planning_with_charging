A company is developing a small electric plane.
This plane flies at about the speed of a car and travels close to the ground.
This plane uses batteries so it has a limited range.
To support cross country travel, the company intends to create a network of charging stations at small airports across the United States.
The plane charges faster at some stations than at others, and doesn't need to fully charge at any one station.
The company wants you to design an algorithm to find the minimum time path between any two of these airports.


## Assumptions:
- The plane has a range of 320km and always begins trips with a full charge.
- The plane travels at 105km/hr at an altitude of 0m.
- The earth is circular and has a radius of 6356.752km. 


## Evaluation criteria:
- Solution feasibility (plane cannot run out of battery at any point)
- Total path time (flying + charging)
- Code quality (readability, maintainability, etc.)
- Computational efficiency (speed, memory usage, etc.)


## Program details
The company has provided you with a header and source file that contains the name, location, and charging rate of each airport in the following format:

    airport name, latitude (degrees), longitude (degrees), charge rate in km/hr

The input to your program will be two strings: the initial airport name and the final airport name.
The output to your program will be a print to std::out as shown below:

    initial airport name,
    second airport name, charge time in hrs,
    third airport name, charge time in hrs,
    ...,
    final airport name

For example, the command

    ./flight_planner Chattanooga_TN Dallas_TX

might output

    Chattanooga_TN, 
    Auburn_AL, 1.525572459816475,
    DeFuniak_Springs_FL, 2.601626016260163,
    Mobile_AL, 0.3503830144012444,
    Slidell_LA, 1.085901368664472,
    Baton_Rouge_LA, 1.84971098265896,
    Alexandria_LA, 0.9852574764805306,
    Lindale_TX, 0.9364816462068926,
    Dallas_TX

The company has also provided you with a checker program in Linux and OSX so you can verify your program.
The company expects that your solution work for all different initial and final airports. 
For example,

    ./checker_linux “
    Chattanooga_TN, 
    Auburn_AL, 1.525572459816475,
    DeFuniak_Springs_FL, 2.601626016260163,
    Mobile_AL, 0.3503830144012444,
    Slidell_LA, 1.085901368664472,
    Baton_Rouge_LA, 1.84971098265896,
    Alexandria_LA, 0.9852574764805306,
    Lindale_TX, 0.9364816462068926,
    Dallas_TX
    ”

will return 
	
    Finding Path Between Chattanooga_TN and Dallas_TX
    Reference result: Success, cost was 24.7704
    Candidate result: Success, cost was 24.4054

The company expects you to be able to improve on the reference solution provided by the checker program.
At a minimum, your solution should match the reference solution.
