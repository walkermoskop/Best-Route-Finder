# Best-Route-Finder

The route.py script finds the best path between two cities. "Best" is determined by the cost function specified when the program is run:

- "time" finds the fastest route.
- "distance" finds the shortest route.
- "segments" finds the route that uses the fewest roads.
- "cycling" finds the safest route, which is based on the assumption that higher speed limits = more danger. To be more precise, this assumes that the probability of an accident is .000001 times the speed limit, meaning that the expected number of accidents between two locations is .000001 x speed limit x distance traveled. 

The route.py script reads in a list of cities and GPS coordinates as well as a list of road segments and the locations they connect. It then uses the [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) to find an ideal route.

The program receives as input two cities -- a start location and end destination (in the format of City,\_State) -- as well as the cost function. For example:

python Chicago,\_Illinois Denver,\_Colorado distance

As output, the program prints the total number of road segments used, the number of miles traveled, the trip duration (in hours) and the full path taken (all ciiies visited and all roads used) to read the end destination.

This script was used as part of an assignment for the Indiana University [Elements of AI graduate course](https://luddy.indiana.edu/academics/courses/class/iub-fall-2020-csci-b551#:~:text=CSCI%2DB%20551%20ELEMENTS%20OF%20ARTIFICIAL%20INTELLIGENCE%20(3%20CR.)&text=Principles%20of%20reactive%2C%20goal%2Dbased,%2C%20reasoning%20under%20uncertainty%2C%20planning.).

The city GPS and road segment text files were provided by Indiana University computer science professor [David Crandall](https://homes.luddy.indiana.edu/djcran/).
