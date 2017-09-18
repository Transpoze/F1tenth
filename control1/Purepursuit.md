
The pure-pursuit algorithm is a geometric control method to maneuver a set of given waypoints in space.The Look ahead distance is the major parameter that decides the performance of the controller.The [paper](www.dtic.mil/get-tr-doc/pdf?AD=ADA599492) gives the theoritical base for the geometric based algorithm.



##The implementation of the Pure pursuit algorithm,with Path planning.

 1. The implementation waits until it receives a path in the form of a waypoint list from the HMI before moving.

 2. Interpolate between 2 waypoints linearly with a fixed density.

 3. The algorithm checks if there is an obstacle in its path,if not the algorithm is in the normal mode and if yes, the algorithm goes to the obstacle mode.

 4. Normal Mode:
	-Finds the closest point in the map and the vehicle tries to get there.
	-Each update from the GPS unit triggers the controller to move to next closest point in the waypoint interpolated list.
 
 5. Obstacle Mode:
	-Each update from the obstacle_avoidance node produces an angle the car needs to move for 1 Look ahead_obstace distance,to avoid the obstacle.(creation of a new waypoint)
	-The update from GPS checks if the new waypoint is reached according to the tolerance set for the new waypoint.

 6. A reference state is computed in the vehicle's frame and a steering command is computed to take the vehicle to the reference state.
 
 7. The refernce state's angle(=`target angle`) is published to the planner and the the velocity request & steering request is sent to the `serial_transmitter.py`.

 8. The algorithm repeats this procedure for each new positional update until it has reached the last waypoint, based upon the radial tolerance set.


