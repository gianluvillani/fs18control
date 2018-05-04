
/*enlargedWayPoints

/ref ws - list of waypoints
/ref enlargedBlue, enlargedYellow - 2 lists/obj for the enlarged waypoint values (we will populate now) - (they might have the form of cones? idk)
/param carRadius - half of the max size of the car, just a parameter (it has to be legal -> carRadius<0.5*min length of the road (we don't care about that))
(if carRadius is zero -> no cones enlargment)

this function returns two points for each WP, they represent the max expansion of the WP according to its relative cones and the carRadius.
we will have 2 point each WP, with same index -> index WP = index enlargedBlue = enlaredYellow
we will use these point as joints for the WALLS for the final circuit.

*/
void enlargeWayPoints(waypoints &ws, Cone &enlargedBlue, Cone &enlargedYellow, int carRadius) {
	
	for (int i = 0; i < ws.size(); i++) { //euclide theorem (maybe)

		double factor = carRadius / dist(ws.blue[i], ws.yellow[i]); // constant factor
		
		enlargedBlue[i].x = ws.blue[i].x + factor*(ws.yellow[i].x - ws.blue[i].x); // X coordinate of the blue cone that defined wp[i] + factor*deltaX
		enlargedBlue[i].y = ws.blue[i].y + factor*(ws.yellow[i].y - ws.blue[i].y);

		enlargedYellow[i].x = ws.yellow[i].x - factor*(ws.yellow[i].x - ws.blue[i].x); //same, just put - to not invert blues and yellows
		enlargedYellow[i].y = ws.yellow[i].y - factor*(ws.yellow[i].y - ws.blue[i].y);

	}
}


/*TRUMP (aka let's build the fucking walls)

/ref enYellow, enBlue - lists that contains the eleent we have just with EnlargeWayPoints
/ref agentmap - actual agent map or similar where to build the walls

from a enlWP to the next one (blue->blue / yellow->yellow) draw straight lines:
first get the line equation constants (kb and ky)
then make a for that runs from an element (X coordinate) to the next one, you can get the y thanks the kb/ky
so you can change the value of the map in those points

finally close the circle going from the last waypoint to the first one

*/

void TRUMP(Cones &enBlue, Cones &enYellow, vector<vector<int>> &agentmap) {
	
	//standard connection
	for (int i = 0; i < enBlue.size()-1; i++) { //enBlue.size() = enYellow.size()
		
		// get kb (infinity/0 check)
		if (enBlue[i+1].x - enBlue[i].x!=0){
			double kb = (enBlue[i+1].y - enBlue[i].y)+1e-8 / (enBlue[i+1].x - enBlue[i].x); //since from a enlWP to the next it's just a line
		} else double kb=1e8 //10^8
		
		// get ky (infinity/0 check)
		if (enYellow[i+1].x - enYellow[i].x!=0){
			double ky = (enYellow[i+1].y - enYellow[i].y)+1e-8 / (enYellow[i+1].x - enYellow[i].x);
		} else double ky=1e8
		
		//work with x
		for (int j = 0; j < enBlue[i + 1].x-enBlue[i].x; j++) {
			agentmap[enBlue[i].x+j][enBlue[i].y+j*kb] = WALL_VALUE; //to be added in constants! (0 walkable, WALL_VALUE = cannot expand here)
		}
		for (int j = 0 ; j <enYellow[i + 1].x-enYellow[i].x; j++) {
			agentmap[enYellow[i].x+j][enYellow[i].y+j*ky] = WALL_VALUE;
		}
		
		//work with y
		for (int j = 0; j < enBlue[i + 1].y-enBlue[i].y; j++) {
			agentmap[enBlue[i].x+j*1/kb][enBlue[i].y+j] = WALL_VALUE; //to be added in constants! (0 walkable, WALL_VALUE = cannot expand here)
		}
		for (int j = 0 ; j <enYellow[i + 1].y-enYellow[i].y; j++) {
			agentmap[enYellow[i].x+j*1/ky][enYellow[i].y+j] = WALL_VALUE;
		}
	}
	
	//last connection to close the end with the 0

	i++; // we want last elements of the lists	
		
	// get kb (no-infinity/0 check)
		if (enBlue[0].x - enBlue[i].x!=0){
			double kb = (enBlue[0].y - enBlue[i].y)+1e-8 / (enBlue[0].x - enBlue[i].x); //since from a enlWP to the next it's just a line
		} else double kb=1e8
		
		// get ky (no-infinity/0 check)
		if (enYellow[0].x - enYellow[i].x!=0){
			double ky = (enYellow[0].y - enYellow[i].y)+1e-8 / (enYellow[0].x - enYellow[i].x);
		} else double ky=1e8
		
		//work with x
		for (int j = 0; j < enBlue[0].x-enBlue[i].x; j++) {
			agentmap[enBlue[i].x+j][enBlue[i].y+j*kb] = WALL_VALUE; //to be added in constants! (0 walkable, WALL_VALUE = cannot expand here)
		}
		for (int j = 0 ; j <enYellow[0].x-enYellow[i].x; j++) {
			agentmap[enYellow[i].x+j][enYellow[i].y+j*ky] = WALL_VALUE;
		}
		
		//work with y
		for (int j = 0; j < enBlue[0].y-enBlue[i].y; j++) {
			agentmap[enBlue[i].x+j*1/kb][enBlue[i].y+j] = WALL_VALUE; //to be added in constants! (0 walkable, WALL_VALUE = cannot expand here)
		}
		for (int j = 0 ; j <enYellow[0].y-enYellow[i].y; j++) {
			agentmap[enYellow[i].x+j*1/ky][enYellow[i].y+j] = WALL_VALUE;
		}
	}

}
