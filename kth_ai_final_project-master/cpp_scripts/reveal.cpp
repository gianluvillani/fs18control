//need to include:
#include <vector>
#include <algorithm>

using namespace std;

/*
void reveal

param/ xa, ya:  current coordinates of agent
param/ xp, yp:  previous coordinates of agent
param/ radius:  vision range
ref/ agentmap:  map visible by the agent
ref/ servermap: input map

given agent position, direction and range -> update the agentmap with the servermap
*/

void reveal(int xa, int ya, int xp, int yp, int radius, vector<vector<int>> &agentmap, vector<vector<int>> &servermap) {
	
	int maxi = min(servermap.size(), xa + radius) - xa; //set i iteration max value as acceptable
	int maxj = min(servermap.size(), ya + radius) - ya; //set j iteration max value as acceptable 

	int starti = max(0, xa - radius) - xa; //set i iteration min value as acceptable
	int startj = max(0, ya - radius) - ya; //set j iteration min value as acceptable

	int flagV = 0;
	int flagO = 0;

	if (xp < xa) starti = 0;	//if X parent is lower than X current -> visual range exclude lower values than current X
	else if (xp > xa) maxi = 0; //if X parent is major than X current -> visual range exclude major values that current X
	
	if (yp < ya) startj = 0;	//if Y parent is lower than X current -> visual range exclude lower values than current Y	
	else if (yp > ya) maxj = 0;	//if Y parent is major than X current -> visual range exclude major values that current Y

	if (xp == xa) flagV = 1;		//vertical flag
	else if (yp == ya) flagO = 1;	//orizzontal flag

	for (i = starti; i < maxi; i++) {
		for (j = startj; j < maxj; j++) {
			// if flags are both false    ->  range limitations detemined by iterations
			// if flags are both true     ->  we are in starting starting position, no range limitations at all
			// if just one flag are true  ->  the point has to included in the realtive area
			if (((!flagV || abs(j) > abs(i)) && (!flagO || abs(j) < abs(i))) || (flagV && flagO ))
				// corner cutting for cirular shape of vision
				if (i*i + j*j <= radius*radius) {
					agentmap[xa + i][ya + j] = servermap[xa + i][ya + j];
				}
		}
	}

}