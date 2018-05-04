public class Waypoint {

    private Coordinate coordinates;

    private Coordinate firstCone;
    private Coordinate secondCone;
    

    public Waypoint(Coordinate waypointCoordinates, Coordinate firstCone, Coordinate secondCone) {
        
        this.coordinates = waypointCoordinates;
        this.firstCone = firstCone;
        this.secondCone = secondCone;
    }

    public Waypoint(Coordinate waypointCoordinates) {

        this.coordinates = waypointCoordinates;
        this.firstCone = null;
        this.secondCone = null;
    }


    public Coordinate getCoordinates() {
        return coordinates;
    }

    public void setCoordinates(Coordinate coordinates) {
        this.coordinates = coordinates;
    }

    public Coordinate getFirstCone() {
        return firstCone;
    }

    public void setFirstCone(Coordinate firstCone) {
        this.firstCone = firstCone;
    }

    public Coordinate getSecondCone() {
        return secondCone;
    }

    public void setSecondCone(Coordinate secondCone) {
        this.secondCone = secondCone;
    }
    
    
        
 

}
