import java.util.Vector;

public class Agent {

    private Server server;

    private Coordinate currentPosition;
    private Coordinate previousPosition;
    
    private int radius;

    private Vector<Coordinate> blueCones; // needed
    private Vector<Coordinate> yellowCones; // needed
    private Vector<Coordinate> redCones; // needed - Start cones

    private Vector<Waypoint> waypoints; // needed, maybe
    private Vector<Coordinate> agentPath; // sure needed
    
    private int previousWaypointIndex; // a che punto sono

    private double sum; // ? vedremo
    

//      This holds the map representation that is available to the agent,
//      in a mixed state. Part are elaborated and parts are not.
    private int[][] internalMapRepresentation;   // occupancy grid in pur case
                                                 // ok

//    This holds a copy of the internal map of the agent, but with only the elaborated parts of it.
//    We use it to decide whether a cone we have identified has been seen before.
    private int[][] recognisedConesRepresentation;  //  

    public Agent(Coordinate startingPosition) // Constructor. Receives initial position
    {
        this.currentPosition = startingPosition; 
    }

    public Agent(Coordinate startingPosition, Server server)
    {
        this.currentPosition = startingPosition;
        this.previousPosition = this.currentPosition;
        this.server = server;

        this.radius = Constants.VISION_RADIUS;

        this.blueCones = new Vector<>();
        this.yellowCones = new Vector<>();
        this.redCones = new Vector<>();

        this.waypoints = new Vector<>();
        this.previousWaypointIndex = -1;

        this.agentPath = new Vector<>();


        int xMapLimit = server.getxMapLimit(); // getlimits of the map
        int yMapLimit = server.getyMapLimit();

        this.initializeInternalMapRepresentation(xMapLimit, yMapLimit);
    }

    private void initializeInternalMapRepresentation(int xMapLimit, int yMapLimit) {

        this.internalMapRepresentation = new int[xMapLimit][yMapLimit];
        this.recognisedConesRepresentation = new int[xMapLimit][yMapLimit];

        //initialize with -1
    }


    /*
     *  Updates the internal incomplete map representation to include the new FOV
     */
    private void updateInternalMapRepresentation()
    {
//        System.out.println("Server revealing more map");
        int[][] revealedMap;

        revealedMap = this.server.reveal(Constants.VISION_RADIUS, this.currentPosition,
                this.previousPosition, this.internalMapRepresentation);

        this.internalMapRepresentation = revealedMap;

        int startX = Math.max(0, this.currentPosition.getX()-this.radius); // be careful with the boundary of the matrix
        int endX = Math.min(this.internalMapRepresentation[0].length, this.currentPosition.getX()+this.radius);

        int startY = Math.max(0, this.currentPosition.getY() - this.radius); //same
        int endY = Math.min(this.internalMapRepresentation.length, this.currentPosition.getY() + this.radius); //same



        for (int y = startY; y < endY; y++)
        {
            for(int x = startX; x < endX; x++)
            {

//                this.markOnMap(this.server.visionMap, new Coordinate(x,y), 6);

                if (detectCone(x, y))
                {
//                    System.out.println(String.format("\tAgent position (%S, %S)", this.currentPosition.getX(),
//                            this.currentPosition.getY()));
//                    System.out.println("\tCone found. In ("+x+","+y+")");
//                    System.out.println(String.format("\tLength of blue cones: %S", this.blueCones.size()));
//                    System.out.println(String.format("\tLength of yellow cones: %S", this.yellowCones.size()));
//                    System.out.println(String.format("\tLength of red cones: %S", this.redCones.size()));
//                    System.out.println("-----------");
                }


            }
        }

//        this.server.writeMapToFile(visionMap, "visionMap.txt");

    }


    /*
     * Goes over the FOV and checks each position to spot blue and yellow cones
     */
    private boolean detectCone(int x, int y)
    {
        int conesAddedCounter = 0;

        if(isBlueCone(x,y))
        {
            addCone(Constants.BLUE_CONE, new Coordinate(x,y));
            conesAddedCounter++;
        }
	else if (isYellowCone(x, y))
        {
            addCone(Constants.YELLOW_CONE, new Coordinate(x,y));

            conesAddedCounter++;
        }
	else if (isRedCone(x,y))
        {
            addCone(Constants.RED_CONE, new Coordinate(x,y));
            conesAddedCounter++;
        }
        return conesAddedCounter > 0;
    }

    private void addCone(int coneKind, Coordinate coneCoordinate)
    {
        if(coneKind == Constants.BLUE_CONE)
        {
            if(!this.blueCones.contains(coneCoordinate)) {
//                System.out.println("Inserting BLUE cone");
                this.blueCones.add(coneCoordinate);
            }

        } else if (coneKind == Constants.YELLOW_CONE) {
            if(!this.yellowCones.contains(coneCoordinate)) {
//                System.out.println("Inserting YELLOW cone");
                this.yellowCones.add(coneCoordinate);
            }

        } else {
            if(!this.redCones.contains(coneCoordinate)) {
//                System.out.println("Inserting RED cone");
                this.redCones.add(coneCoordinate);
            }
        }

    }

    /*
     *  Checks if the value at the specified position is BLUE_CONE
     */
    private boolean isBlueCone(int x, int y)
    {
        return this.internalMapRepresentation[x][y] == Constants.BLUE_CONE;
    }

    /*
     *  Checks if the value at the specified position is YELLOW_CONE
     */
    private boolean isYellowCone(int x, int y)
    {
        return this.internalMapRepresentation[x][y] == Constants.YELLOW_CONE;
    }

    /*
 *  Checks if the value at the specified position is RED_CONE
 */
    private boolean isRedCone(int x, int y)
    {
        return this.internalMapRepresentation[x][y] == Constants.RED_CONE;
    }


    private Vector<Waypoint> createWaypoints(Vector<Coordinate> shortest, Vector<Coordinate> longest) {
        //take as first reference the shortest list of cones and as second the longest


        Vector<Waypoint> waypoints = new Vector<>();
        //
        // 
        //  Check this.
        //
        //
        int k = 0;
        for (int i = 0; i < longest.size(); i++) { // COMINCIA DALLA LISTA PIU' LUNGA. PER OGNI CONO AL SUO INTERNO
                                                   // SCORRI TUTTI QUELLI NELLA LISTA PIU' CORTA.
                                                   // PER OGNI CONO NELLA LISTA PIU' LUNGA AGGIUNGI UN WAYPOINT.
                                                   // 
            double mindist = maxdist; //initialize min dist
            int mindex = -1; //initialize mindex as illegal value
            
            for (int j = 0; j < shortest.size(); j++) { // SCORRI LA LISTA CORTA

                //check if dist is legal and check if new dist is better
                if (mindist > Utilities.dist(longest.get(i), shortest.get(j))) //if (mindist > dist) should be enough (since mindist initialized as maxdist)
                        //&& Utilities.dist(longest.get(i), shortest.get(j)) < maxdist)
                {

                    //update min
                    mindist = Utilities.dist(longest.get(i), shortest.get(j));
                    mindex = j;
                }
            }
            if (mindex != -1) {
                k++; //waypoint[0] reserved for start cones
                
                int newWaypointX = (int)Math.round(0.5*(longest.get(i).getX() + shortest.get(mindex).getX()));
                int newWaypointY = (int)Math.round(0.5*(longest.get(i).getY() + shortest.get(mindex).getY()));

                Coordinate waypointCoordinate = new Coordinate(newWaypointX, newWaypointY);
                Coordinate firstCone = longest.get(i);
                Coordinate secondCone = shortest.get(mindex);

                Waypoint newWaypoint = new Waypoint(waypointCoordinate, firstCone, secondCone); // AGGIUNGI I POVERI GENITORI

                waypoints.add(newWaypoint);

            }
        }

        return waypoints;

    }


    private Waypoint createSingleWaypoint(Vector<Coordinate> redCones) // PRIMO WAYPOINT
    {
        Coordinate firstRedCone = redCones.get(0);
        Coordinate secondRedCone = redCones.get(1);


        int newWaypointX = (int)Math.round(0.5*(firstRedCone.getX() + secondRedCone.getX()));
        int newWaypointY = (int)Math.round(0.5*(firstRedCone.getY() + secondRedCone.getY()));

        Coordinate waypointCoordinate = new Coordinate(newWaypointX, newWaypointY);

        Waypoint redWaypoint = new Waypoint(waypointCoordinate, firstRedCone, secondRedCone);

        return redWaypoint;

    }

    public void performMoveAction() // MAYBE NOT NEEDED AT ALL
    {
        System.out.println(String.format("Current position : (%S, %S)", this.currentPosition.getX(), this.currentPosition.getY()));
        this.updateInternalMapRepresentation();


        Waypoint startingPosition = new Waypoint(new Coordinate(server.getStartingX(), server.getStartingY())); // ADD WAYPOINT - CURRENT position
        Waypoint redWaypoint = createSingleWaypoint(this.redCones);     
        this.waypoints.add(0, redWaypoint);


        if(this.blueCones.size() < this.yellowCones.size()){
            this.waypoints.addAll(this.createWaypoints(blueCones, yellowCones));
        } else {
            this.waypoints.addAll(this.createWaypoints(yellowCones, blueCones));
        }

        
        Explorer explorer = new Explorer(this.waypoints); // Explorer


        explorer.sortWaypoints(startingPosition);   // Sort waypoints from given position startingPosition
        Vector<Waypoint> sortedWaypoints = explorer.getWaypoints();

        boolean passedTheStartingLine = false;

        // a memory of the agent's movement
        this.sum = 0;

        while((this.currentPosition != startingPosition.getCoordinates()) || (passedTheStartingLine == false)) // 
        {

            if(currentPosition == startingPosition.getCoordinates()) // this should be different of course.
                                                                     // you will never get into the starting position
            {
                passedTheStartingLine = true;
            }
        
            boolean moved = updatePosition(sortedWaypoints, startingPosition);
            this.agentPath.add(this.currentPosition);

            if(!moved)
            {
                break;
            }
            this.updateInternalMapRepresentation();

            this.waypoints = new Vector<>();
            this.waypoints.add(0, redWaypoint);


            if(this.blueCones.size() < this.yellowCones.size()){
                this.waypoints.addAll(this.createWaypoints(blueCones, yellowCones));
            } else {
                this.waypoints.addAll(this.createWaypoints(yellowCones, blueCones));
            }

            explorer.setWaypoints(this.waypoints);
            explorer.sortWaypoints(startingPosition);
            sortedWaypoints = explorer.getWaypoints();

        }

        explorer.enlargeWayPoints(Constants.CAR_RADIUS);


        Vector<Coordinate> firstColorConesWaypoints = new Vector<>();
        for (Waypoint wp : explorer.getWaypoints())
        {
            firstColorConesWaypoints.add(wp.getFirstCone());
        }

        Vector<Coordinate> secondColorConesWaypoints = new Vector<>();
        for (Waypoint wp : explorer.getWaypoints())
        {
            secondColorConesWaypoints.add(wp.getSecondCone());
        }
        int[][] walledMap = explorer.TrumpIt(this.internalMapRepresentation, firstColorConesWaypoints, secondColorConesWaypoints);



        for(Waypoint wp : explorer.getWaypoints())
        {
            int x = wp.getCoordinates().getX();
            int y = wp.getCoordinates().getY();

            this.markOnMap(this.server.visionMap, new Coordinate(x,y), Constants.WAYPOINT);
            this.markOnMap(this.server.visionMap, wp.getFirstCone(), Constants.GATE_INDICATOR);
            this.markOnMap(this.server.visionMap, wp.getSecondCone(), Constants.GATE_INDICATOR);
        }

        for(Coordinate cone : this.blueCones)
        {
            this.markOnMap(this.server.visionMap, cone, Constants.BLUE_CONE);
        }


        for(Coordinate cone : this.yellowCones)
        {
            this.markOnMap(this.server.visionMap, cone, Constants.YELLOW_CONE);
        }


        for(Coordinate po : this.agentPath)
        {
            int x = po.getX();
            int y = po.getY();
            this.markOnMap(walledMap, po, Constants.PATH);
        }

        this.server.writeMapToFile(walledMap, "wallsWithPathMap.txt");
        this.server.writeMapToFile(this.server.visionMap, "visionWithConesWaypointsMap.txt");



    }

    private boolean updatePosition(Vector<Waypoint> orderedWaypoints, Waypoint startingPosition)
    {
        System.out.println(String.format("Current position : (%S, %S)", this.currentPosition.getX(), this.currentPosition.getY()));

        if (this.previousWaypointIndex == -1 && this.currentPosition == startingPosition.getCoordinates()) {
            this.previousPosition = startingPosition.getCoordinates();
            this.currentPosition = startingPosition.getCoordinates();

        }

        if(this.previousWaypointIndex == orderedWaypoints.size() - 1) //
        {
            return false;
        }

        int yn = orderedWaypoints.get(this.previousWaypointIndex+1).getCoordinates().getY();
        int xn = orderedWaypoints.get(this.previousWaypointIndex+1).getCoordinates().getX();
        System.out.println("xn :"+xn+ "   yn : "+yn);

        int yc = this.currentPosition.getY();
        int xc = this.currentPosition.getX();


        int yp = this.previousPosition.getY();
        int xp = this.previousPosition.getX();

        double kx = (yn - yc + Math.pow(10, -10)) / (xn - xc + Math.pow(10, -10));
        double ky = 1/kx;


        if(Math.abs(kx) <= Math.abs(ky))
        {
            if(xn - xp >= 0)
            {
                xc++;
                this.sum += kx;

            } else {
                kx = -kx; //change sign of k
                xc--;
                this.sum += kx;

            }

            yc += (int) Math.round(this.sum);
            if (this.sum >= 0.5) this.sum -= 1;
            if (this.sum <= -0.5) this.sum += 1;

        } else {
            if(yn - yp >= 0)
            {
                yc++;
                this.sum += ky;

            } else {
                ky = -ky;
                yc--;
                this.sum += ky;
            }

            xc += (int) Math.round(this.sum);
            if (this.sum >= 0.5) this.sum -= 1;
            if (this.sum <= -0.5) this.sum += 1;

        }

        this.previousPosition = this.currentPosition;
        Coordinate newPositionCoordinates = new Coordinate(xc, yc);
        this.currentPosition = newPositionCoordinates;
        System.out.println();

        if(this.currentPosition.equals(orderedWaypoints.get(previousWaypointIndex+1).getCoordinates()))
        {
            this.previousWaypointIndex++;
        }


        return true;
    }

    private void markOnMap(int[][] map, Coordinate coordinate, int mark)
    {
        map[coordinate.getX()][coordinate.getY()] = mark;
    }

}
