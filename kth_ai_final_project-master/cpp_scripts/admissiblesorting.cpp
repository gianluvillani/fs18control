//functions required for the exploration pathfinding - admissible WP sorting

/*notbackward

/param xp, yp - previous waypoint/position
/param xc, yc - current waypoint/position
/param xn, yn - next waypoint/position

previous waypoint and current one give CURRENT direction in X and Y
current waypoint and next waypoint five NEXT direction in X and Y

if there is no change of direction in BOTH of the dimensions -> we are not going backward -> return 1
otherwise -> we change both directions -> we are going backward -> BAD -> return 0

*/

int notbackward(xp, yp, xc, yc, xn, yn) {

	if ((xc - xp)*(xn - xc) >= 0 || (yc - yp)(yn - yc) >= 0)
		return 1;
	else return 0;
}

/*sortwaypoint

/ref ws - waypoint list (not sorted but element 0)
/param xs, ys - starting postion coordinates

NB: this function require the waypoint list already updated with the startingWP ALREADY in the 0 index.
	(we can do it in the findwaypointfunction)

given the startingWP, the function find the best next element according to direction(1) and distance(2)

(1) admissible direction
(2) the closest

*/

void sortwaypoint(waypoint &ws, int xs, int ys) {
	
	XpreviousWP = xs; //starting position = parent of node 0 -> give first direction
	YpreviousWP = ys;

	for (int i = 0; i < ws.size() - 1; i++) { //go from 0 to N-1
		
		if (i > 0) { //skip if the current WP is the 0 (aka starting WP)
			XpreviousWP = ws[i - 1].x; //otherwise save previous WP coordinates
			YpreviousWP = ws[i - 1].y;
		}

		mindist = 10; //twice the max legal distance (just in case we lost one WP, we have chance to get the following one..)
		mindex = -1;

		for (int j = i + 1; j < ws.size(); j++) { //go from i+1 to N
			// if (direction is admissible) && (new dist is lower than mindist) --> found new best candidate!
			if (notbackward(XpreviousWP, YpreviousWP, ws[i].x, ws[i].y, ws[j].x, ws[j].y) && dist(ws[i], ws[j]) < mindist) {
				mindist = dist(ws[i], ws[j]);
				mindex = j;
			}
		}//end for j

		if (mindex != -1)
			swap(ws[i + 1], ws[mindex]); //swap the selected and put as next WP
		else
			break; //if we have a gap/idk just break and hope that with future vision it will be all OK.			
	}//end for i
}