public class Utilities {

    public static double dist(Coordinate a, Coordinate b)
    {
        int ax = a.getX();
        int ay = a.getY();

        int bx = b.getX();
        int by = b.getY();

        return Math.sqrt(Math.pow((ax-bx), 2) + Math.pow((ay-by),2));
    }
}
