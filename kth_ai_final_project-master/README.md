This is the implementation of the final project for KTH DD2380 course

Server.java
This class represents a dummy server. Its job is to read the map/circuit from a pre-created file and save it into a twodimension integer array internally.

The circuit is defined as the part of the map contained betwen pairs of blue, yellow and a single pair of red cones. The red cones define the starting line, but the starting position of the agent is a bit behind the line, at a position specified during the creation of the map. 

The circuit is circular and closed. 

It also is responsible for providing the agent with part of that map, defined by a specific radius on the Agent's part, whenever the agents asks for it.

Finnaly, the server writes a two dimension integer array to a file, at the agents request.

Agent.java
This class represents our agent. The agent is defined by its current position in the global circuit map, and its previous position. The agent does not know the shape or characteristics of the circuit, other that the complete size of the map.

The agent asks the server to reveal to it a part of the map, in a direction defined by its current and previous positions and a radius. Then, in that newly revealed part, it tries to detect cones and their colour, and saves theri positions in two vectors, which hold all the cones.

After the cone detection, the agent attempts to create waypoints. Waypoints are positions between apirs of two cones of different colors. So we try to match a yellow cone to a close by blue cone, and then mark the middle point of their distance as a waypoint.

After having completed that, the agent sorts all the waypoints it has created according to their distance from the waypoint created between the red cones, which is single and unique("redWaypoint", since there are only two red cones).

Following that the agents tries to move. the movement of the agent is defined by the its current position, the coordinates of the previous waypoint and the next waypoint. We use the two waypoints to define a straight line between the two consecutive waypoints, and have the agent move along this line, one sqaure at a time. 

After the agent has completed a complete navigation of the circuit, so it has sees all the cones, has created all the corresponding waypoints and has moved though them, it considers the pass complete and moves ahead to the enlargement of the cones, and the creation of the walls between them.



Explorer
This class hold some fuctions which the agent uses to understand and move through it environment. A sorting fuction sorts the waypoints according to their Euclidean distance, and the redWaypoint.

The enlargeWaypoints function enlarges the cones that correspond to each waypoint. Each waypoint is created between two cones, that at the beginning have specific and "true" coordinates on the global map. The enlargement takes the form of changing those coordinates. We create the line between the two cones, and move the coordinates of them along this line, for a distance equal to the car width (which is variable, but 1 in our case). In essence, we are making the circuit narrower, to avoid colission of the agent with the cones, and to make sure our course in the circuit does not take us out of its limits.

Finally, the TrumpIt function is very eager to build the walls between the cones. We connect each enlarged cone with its neighor of the same color, and replace every value along this imaginary line on the agent map representation with a specific value, whic denotes a wall (5). In this way, we achieve concrete boundaries within which the agent can move. Everything between the walls is a valid position for the agent to be, and since the circuit is closed we cannot end up outside of those boundaries.



Coordinate
A simple class denoting a coordinate. It holds two integers which represent the (x,y) coordinate pair, and it defines an equality function between coordinates.

Waypoint
This class represents a waypoint. It holds the waypoint coordinates, as well as two references to the cones which define the waypoint. Both the cones and the waypoint are defined as Coordinate.


Utilities
Just an implementation of the Euclidean distance metric between to Coordinate objects.


Constants
This class simply holds some constants, including the values for the colors of the cones, the radius of vision for the agent, and the wall values


After we have completed all the different parts, we print two files, wallsWithPathMap.txt which represents the wall values with the number 5 and the path the agent took with the number 7, and visionWithConesWaypointsMap.txt which represents the complete area the agent saw during its run with 6, the cones with 1 and 2, the waypoints with 8.


To run  
Unpack  
cd implementation  
javac -cp . src/*.java  
java -classpath src CircuitExploration  


To run with different maps  
java -classpath src CircuitExploration [map_name]  

where map_name can be anyone in map_circuit_h.txt, map_circuit_messy.txt, map_circuit.txt
by default the agent runs on map_circuit

To visualize the results, call the visualizer.py script with  
python3 visualizer.py [map_name]  

where [map name] has to be the same as the map you used in the CircuitExploration. If no specific map was used in the CircuitExploration, give the arument map_circuit.txt here. The visualization results are in viz/
