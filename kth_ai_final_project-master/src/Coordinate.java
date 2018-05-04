public class Coordinate {



    private int X;
    private int Y;

    public Coordinate(int x, int y)
    {
        this.X = x;
        this.Y = y;
    }


    public int getX() {
        return X;
    }

    public void setX(int x) {
        X = x;
    }

    public int getY() {
        return this.Y;
    }

    public void setY(int y) {
        this.Y = y;
    }

//    public boolean equals(Coordinate o) {
//        if (this == o) return true;
//        if (o == null || getClass() != o.getClass()) return false;
//
//        return this.getX() == o.getX() && this.getY() == o.getY();
//    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Coordinate that = (Coordinate) o;

        if (getX() != that.getX()) return false;
        return getY() == that.getY();
    }

    @Override
    public int hashCode() {
        int result = getX();
        result = 31 * result + getY();
        return result;
    }

    public double dist(Coordinate o)
    {
        if (this == o) return 0;
        if (o == null || getClass() != o.getClass()) throw new IllegalArgumentException("The passed Coordinate is null, or not a Coordinate");
     
        int ax = this.getX();
        int ay = this.getY();

        int ox = o.getX();
        int oy = o.getY();

        return Math.sqrt(Math.pow((ax-ox), 2) + Math.pow((ay-oy),2));
    }


    public String print()
    {
        return String.format("(%S, %S)", this.getX(), this.getY());
    }

}
