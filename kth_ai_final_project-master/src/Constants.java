public class Constants {

    static final int VISION_RADIUS = 20;
    static final int CAR_RADIUS = 1;

    static final int BLUE_CONE = 1;
    static final int YELLOW_CONE = 2;
    static final int RED_CONE = 3;

    static final int STARTING_POSITION = 4;

    static final int WALL = 5;
    static final int VISION = 6;

    static final int WAYPOINT = 8;
    static final int GATE_INDICATOR = 9;

    static final int TWO_CONSECUTIVE_WAYPOINTS_DISTANCE_MAX = 100;

    static final int PATH = 7;


    //since the max distance between two cones is 5m and the max width of the cicruit
    //is 3.5m, the max distance between the two cones to connect is the sqrt(3.5m^2+0.5*5m^2)
    static final double TWO_NEIGHBORING_CONES_DISTANCE_MAX = 100;

}
