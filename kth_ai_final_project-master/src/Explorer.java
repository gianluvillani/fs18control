import java.util.Vector;

public class Explorer {

    private Vector<Waypoint> waypoints;

    public Explorer(Vector<Waypoint> waypoints)
    {
        this.waypoints = waypoints;

    }

    private boolean notbackward(Waypoint previous, Waypoint current, Waypoint next) {
    // facciamo che 
        int xc = current.getCoordinates().getX();
        int yc = current.getCoordinates().getY();

        int xp = previous.getCoordinates().getX();
        int yp = previous.getCoordinates().getY();

        int xn = next.getCoordinates().getX();
        int yn = next.getCoordinates().getY();

        if ((xc - xp) * (xn - xc) >= 0 || (yc - yp) * (yn - yc) >= 0) {
            return true;
        }
        return false;
    }


    public void sortWaypoints(Waypoint startingPosition) {

        int xs = startingPosition.getCoordinates().getX();  // x_starting
        int ys = startingPosition.getCoordinates().getY();  // y_starting 

//        int XpreviousWP = xs; // starting position = parent of node 0 -> give first direction
//        int YpreviousWP = ys; // 

        Waypoint previousWP = startingPosition;

        for (int i = 0; i < this.waypoints.size() - 1; i++) { //go from 0 to N-1
            //skip if the current WP is the 0 (aka starting WP)
            if (i > 0) { // 
                //otherwise save previous WP coordinates
                previousWP = this.waypoints.get(i - 1);
            }

            //twice the max legal distance (just in case we lost one WP, we have chance to get the following one..)
            int mindist = Constants.TWO_CONSECUTIVE_WAYPOINTS_DISTANCE_MAX;
            int mindex = -1;

            //go from i+1 to N
            for (int j = i + 1; j < this.waypoints.size(); j++) {

                // if (direction is admissible) && (new dist is lower than mindist) --> found new best candidate!
                if (notbackward(previousWP, this.waypoints.get(i),this.waypoints.get(j)) &&
                            Utilities.dist(this.waypoints.get(i).getCoordinates(),
                                            this.waypoints.get(j).getCoordinates()) < mindist
                    ) {

                    mindist = (int)Math.round(
                                    Utilities.dist(
                                            this.waypoints.get(i).getCoordinates(),
                                            this.waypoints.get(j).getCoordinates()
                                            )
                                    );
                    mindex = j;
                }
            }//end for j

            if (mindex != -1) {

                Waypoint prev = this.waypoints.get(mindex);
                this.waypoints.set(mindex,this.waypoints.get(i + 1)); //swap the selected and put as next WP
                this.waypoints.set(i+1, prev);
            } else {
                //if we have a gap/idk just break and hope that with future vision it will be all OK.
                break;
            }

        }//end for i
    }

    /*
    /ref ws - list of waypoints
    /ref enlargedBlue, enlargedYellow - 2 lists/obj for the enlarged waypoint values
                                        (we will populate now) - (they might have the form of cones? idk)
    /param carRadius - half of the max size of the car, just a parameter
                        (it has to be legal -> carRadius<0.5*min length of the road (we don't care about that))
                        (if carRadius is zero -> no cones enlargment)

    this function returns two points for each WP, they represent the max expansion of the WP according to its relative cones and the carRadius.
    we will have 2 point each WP, with same index -> index WP = index enlargedBlue = enlaredYellow
    we will use these point as joints for theConstants.WALLS for the final circuit.
    */
    void enlargeWayPoints(int carRadius) {

        for (int i = 0; i < this.waypoints.size(); i++) { //euclide theorem (maybe)

            // constant factor
            double factor = carRadius / Utilities.dist(this.waypoints.get(i).getFirstCone(),
                                            this.waypoints.get(i).getSecondCone());

            int newFirstConeX;
            int newFirstConeY;

//            enlargedBlue[i].x = ws.this.waypoints.get().getFirstCone()[i].x + factor*(ws.this.waypoints.get().getSecondCone()[i].x - ws.this.waypoints.get().getFirstCone()[i].x);
//            enlargedBlue[i].y = ws.this.waypoints.get().getFirstCone()[i].y + factor*(ws.this.waypoints.get().getSecondCone()[i].y - ws.this.waypoints.get().getFirstCone()[i].y);

            // X coordinate of the this.waypoints.get().getFirstCone() cone that defined wp[i] + factor*deltaX
            newFirstConeX = (int)Math.round(this.waypoints.get(i).getFirstCone().getX() +
                            factor*(this.waypoints.get(i).getSecondCone().getX() -
                                    this.waypoints.get(i).getFirstCone().getX()
                            ));


            newFirstConeY = (int)Math.round(this.waypoints.get(i).getFirstCone().getY() +
                    factor*(this.waypoints.get(i).getSecondCone().getY() -
                            this.waypoints.get(i).getFirstCone().getY()
                    ));

            Coordinate newFirstCone = new Coordinate(newFirstConeX, newFirstConeY);

            int newSecondConeX, newSecondConeY;

//            enlargedYellow[i].x = ws.this.waypoints.get().getSecondCone()[i].x - factor*(ws.this.waypoints.get().getSecondCone()[i].x - ws.this.waypoints.get().getFirstCone()[i].x);
//            enlargedYellow[i].y = ws.this.waypoints.get().getSecondCone()[i].y - factor*(ws.this.waypoints.get().getSecondCone()[i].y - ws.this.waypoints.get().getFirstCone()[i].y);

            //same, just put - to not invert this.waypoints.get().getFirstCone()s and this.waypoints.get().getSecondCone()s
            newSecondConeX = (int)Math.round(this.waypoints.get(i).getSecondCone().getX() -
                                    factor*(this.waypoints.get(i).getSecondCone().getX() -
                                    this.waypoints.get(i).getFirstCone().getX()
                                ));

            newSecondConeY = (int)Math.round(this.waypoints.get(i).getSecondCone().getY() -
                    factor*(this.waypoints.get(i).getSecondCone().getY() -
                            this.waypoints.get(i).getFirstCone().getY()
                    ));

            Coordinate newSecondCone = new Coordinate(newSecondConeX, newSecondConeY);

            this.waypoints.get(i).setFirstCone(newFirstCone);
            this.waypoints.get(i).setSecondCone(newSecondCone);

        }
    }

    /*TRUMP (aka let's build the fucking walls)
        /ref this.waypoints.get().getSecondCone(), this.waypoints.get().getFirstCone() - lists that contains the eleent we have just with EnlargeWayPoints
        /ref agentmap - actual agent map or similar where to build the walls
        from a enlWP to the next one (blue->blue / yellow->yellow) draw straight lines:
        first get the line equation constants (kx and ky)
        then make a for that runs from an element (X coordinate) to the next one, you can get the y thanks the kx/ky
        so you can change the value of the map in those points
        finally close the circle going from the last waypoint to the first one
    */

    public int[][] TrumpIt(int[][] agentMap, Vector<Coordinate> secondColorConeCoordinates, Vector<Coordinate> firstColorConeCoordinates) {

        int[][] walledAgentMap = new int[agentMap.length][agentMap[0].length]; // ? //

        int i;

        //standard connection
        for (i = 0; i < secondColorConeCoordinates.size(); i++) {

            int xc = secondColorConeCoordinates.get(i).getX();
            int yc = secondColorConeCoordinates.get(i).getY();

            int xn, yn;

            if(i == this.waypoints.size()-1)
            {
                xn = secondColorConeCoordinates.get(0).getX();
                yn = secondColorConeCoordinates.get(0).getY();

            } else {
                xn = secondColorConeCoordinates.get(i+1).getX();
                yn = secondColorConeCoordinates.get(i+1).getY();
            }

            //init kx, ky, sum for the couple of gates
            double kx = (yn - yc + Math.pow(10, -10)) / (xn - xc + Math.pow(10, -10));
            double ky = 1/kx;
            double sum = 0;

            if(Math.abs(kx) <= Math.abs(ky)) //then x independent
            {

                while(xn - xc > 0) //if positive dir
                {
                    xc++;
                    sum += kx;
                    yc += (int)Math.round(sum); //balanced apporach with credit and debit
                    if(sum >=  0.5) sum-=1;
                    if(sum <= -0.5) sum+=1;

                    walledAgentMap[xc][yc] = Constants.WALL;

                }

                
                kx = -kx; //change sign of k
                while(xn - xc < 0) //if negative dir
                {
                    xc--;
                    sum += kx;
                    yc += (int)Math.round(sum);
                    if(sum >=  0.5) sum-=1;
                    if(sum <= -0.5) sum+=1;

                    walledAgentMap[xc][yc] = Constants.WALL;
                }

            } else { // then y independent

                while(yn - yc > 0) //if positive dir
                {
                    yc++;
                    sum += ky;
                    xc += (int)Math.round(sum);
                    if(sum >=  0.5) sum-=1;
                    if(sum <= -0.5) sum+=1;

                    walledAgentMap[xc][yc] = Constants.WALL;
                }


                ky = -ky;
                while (yn - yc < 0){ //if negative dir
                    yc--;
                    sum += ky;
                    xc += (int)Math.round(sum);
                    if(sum >=  0.5) sum-=1;
                    if(sum <= -0.5) sum+=1;

                    walledAgentMap[xc][yc] = Constants.WALL;
                }
            }
            
        }


        //standard connection
        for (i = 0; i < firstColorConeCoordinates.size(); i++) {

            int xc = firstColorConeCoordinates.get(i).getX();
            int yc = firstColorConeCoordinates.get(i).getY();

            int xn, yn;

            if (i == this.waypoints.size() - 1) {
                xn = firstColorConeCoordinates.get(0).getX();
                yn = firstColorConeCoordinates.get(0).getY();

            } else {
                xn = firstColorConeCoordinates.get(i + 1).getX();
                yn = firstColorConeCoordinates.get(i + 1).getY();
            }

            //init kx, ky, sum for the couple of gates
            double kx = (yn - yc + Math.pow(10, -10)) / (xn - xc + Math.pow(10, -10));
            double ky = 1 / kx;
            double sum = 0;

            if (Math.abs(kx) <= Math.abs(ky)) //if x independent
            {

                while (xn - xc > 0) //if positive dir
                {
                    xc++;
                    sum += kx;
                    yc += (int) Math.round(sum); //balanced apporach with credit and debit
                    if (sum >= 0.5) sum -= 1;
                    if (sum <= -0.5) sum += 1;

                    walledAgentMap[xc][yc] = Constants.WALL;

                }


                kx = -kx; //change sign of k
                while (xn - xc < 0) //if negative dir
                {
                    xc--;
                    sum += kx;
                    yc += (int) Math.round(sum);
                    if (sum >= 0.5) sum -= 1;
                    if (sum <= -0.5) sum += 1;

                    walledAgentMap[xc][yc] = Constants.WALL;
                }

            } else { //if y independent

                while (yn - yc > 0) //if positive dir
                {
                    yc++;
                    sum += ky;
                    xc += (int) Math.round(sum);
                    if (sum >= 0.5) sum -= 1;
                    if (sum <= -0.5) sum += 1;

                    walledAgentMap[xc][yc] = Constants.WALL;
                }


                ky = -ky;
                while (yn - yc < 0) { //if negative dir
                    yc--;
                    sum += ky;
                    xc += (int) Math.round(sum);
                    if (sum >= 0.5) sum -= 1;
                    if (sum <= -0.5) sum += 1;

                    walledAgentMap[xc][yc] = Constants.WALL;
                }
            }
        }

        return walledAgentMap;


    }



    public Vector<Waypoint> getWaypoints() {
        return waypoints;
    }

//    public void setWaypoints(Vector<Waypoint> waypoints) {this.waypoints = waypoints;}

    public void setWaypoints(Vector<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }


}
