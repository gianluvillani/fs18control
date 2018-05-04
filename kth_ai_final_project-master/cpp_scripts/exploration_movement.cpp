/*move

/ref wp - waypoint list
/ref ap - agentposition list
/param oldwp_index - it the INDEX of the LAST WAYPOINT! (give -1 if we have no last waypoint, because we just start)
/param xs,ys - actual very starting position of the machine
/ref agentpos_last_index - last index used in the moveagent (with a pushback it would be useless)

this function update the agent position list

it return 1 if the agent perform any sort of movement,
it return 0 if the exploration lap is over.

*/

int move(waypoint &wp, agentposition &ap, int &oldwp_index int xs, int ys, int &agentpos_last_index) {

	//have we finished?
	if (oldwp_index == wp.size())
		return 0; //we finished the exploration lap

	//else..

	 /*check if the oldwp_index is :
		  == -1			this means that we are at the beginning, so as previous WP we consider the starting postion.
		  == wp.size() - 1	this means that we are at the end, so we consider as next WP the first one.*/

	//if we start now
	if (oldwp_index == -1) {
		int Xprevious = xs;
		int Yprevious = ys;
	}
	else { //if not
		int Xprevious = wp[oldwp_index].x;
		int Yprevious = wp[oldwp_index].y;
	}


	if (oldwp_index < wp.size() - 1) {
		int Xnext = wp[oldwp_index + 1].x;
		int Ynext = wp[oldwp_index + 1].y;
	}
	else { //if we have to close the loop
		int Xnext = wp[0].x;
		int Ynext = wp[0].y;
	}


	//once we get previous and next we get the K.we also make boundaries to avoid K = 0 (admissible here, actually) and K = infinity
	if (Xnext - Xprevious != 0) {
		double k = (Ynext - Yprevious) + 1e-8 / (Xnext - Xprevious); //since from a enlWP to the next it's just a line
	}
	else {
		double k = 1e8
	}

	//then we iterate on X calculating the Y = X*K
	//this will create many jumps ad discontinuity for K>1, but we don't care actually.

	for (j = 1; j <= Xnext - Xprevious; j++) {

		ap[++agent_last_index].x = Xprevious + j;
		ap[++agent_last_index].y = Yprevious + j*k; //not realistic route, this will cause jumps..
		
	}

	if (j == 1) { //for vertical or semivertical movements
		ap[++agent_last_index].x = Xnext;
		ap[++agent_last_index].y = Ynext; //not realistic route, this will cause jumps..
	}


	oldwp_index++; //update wp index

	return 1; //exploration movement went fine.
	
}
