import java.io.*;
import java.util.Scanner;


public class Server {
    private String filename;

    private int[][] serverMap;


    private int xMapLimit;
    private int yMapLimit;

    public int[][] visionMap;

    private int startingX;
    private int startingY;

    public Server(String filename)
    {
        this.filename = filename;
        this.serverMap = new int[100][100];
        this.visionMap = new int[100][100];

        this.readMapFromFile();

        this.xMapLimit = this.serverMap[0].length;
        this.yMapLimit = this.serverMap.length;
    }


    public int[][] reveal(int radius, Coordinate currentPosition, Coordinate previousPosition, int[][] agentMap) {
        System.out.println("Revealing");
        int xa = currentPosition.getX();
        int ya = currentPosition.getY();

        int xp = previousPosition.getX();
        int yp = previousPosition.getY();

        int maxi = Integer.min(this.xMapLimit, xa + radius) - xa; //set i iteration max value as acceptable
        int maxj = Integer.min(this.yMapLimit, ya + radius) - ya; //set j iteration max value as acceptable

        int starti = Integer.max(0, xa - radius) - xa; //set i iteration min value as acceptable
        int startj = Integer.max(0, ya - radius) - ya; //set j iteration min value as acceptable

        boolean flagV = false;
        boolean flagO = false;

        if (xp < xa) starti = 0;	//if X parent is lower than X current -> visual range exclude lower values than current X
        else if (xp > xa) maxi = 0; //if X parent is major than X current -> visual range exclude major values that current X

        if (yp < ya) startj = 0;	//if Y parent is lower than X current -> visual range exclude lower values than current Y
        else if (yp > ya) maxj = 0;	//if Y parent is major than X current -> visual range exclude major values that current Y

        if (xp == xa) flagV = true;		//vertical flag
        if (yp == ya) flagO = true; //horizontal flag


        for (int i = starti; i < maxi; i++) {
            for (int j = startj; j < maxj; j++) {
                // if flags are both false    ->  range limitations detemined by iterations
                // if flags are both true     ->  we are in starting starting position, no range limitations at all
                // if just one flag are true  ->  the point has to included in the realtive area
                if (((!flagV || Math.abs(j) > Math.abs(i)) && (!flagO || Math.abs(j) < Math.abs(i))) || (flagV && flagO ))
                    // corner cutting for cirular shape of vision
                    if (i*i + j*j <= radius*radius) {
                        agentMap[xa + i][ya + j] = this.serverMap[xa + i][ya + j];

                        visionMap[xa + i][ya + j] = Constants.VISION;

                    }
            }
        }

        return agentMap;
    }

    private void readMapFromFile()
    {
        Scanner sc = null;
        try {
            sc = new Scanner(new FileReader(this.filename));

            String line;

//            for(int row = this.serverMap.length-1; row >= 0; row--)
            for(int row = 0; row < this.serverMap.length; row++)
            {
                line = sc.nextLine();
                String[] split = line.split("");

                int[] split_line = toIntArray(split);

                for(int column = 0; column < this.serverMap[0].length; column++)
                {
                    if(split_line[column] == Constants.STARTING_POSITION) {
                        this.startingX = row;
                        this.startingY = column;
                    }

                    this.serverMap[row][column] = split_line[column];
                }
            }

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }


    public void writeMapToFile(int[][] map, String filename)
    {
        BufferedWriter outputWriter = null;
        try {
            outputWriter = new BufferedWriter(new FileWriter(filename));

            for (int row = 0; row < map.length; row++) {
                for(int column = 0; column < map[0].length; column++)
                {
                    outputWriter.write(Integer.toString(map[row][column]));
                }

                outputWriter.newLine();
            }
            outputWriter.flush();
            outputWriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private int[] toIntArray(String[] stringArray)
    {
        int[] intArray = new int[stringArray.length];

        for (int i = 0; i < stringArray.length; i++)
        {
            intArray[i] = Integer.parseInt(stringArray[i]);
        }

        return intArray;
//        return Arrays.stream(stringArray).mapToInt(Integer::parseInt).toArray();
    }

    public int getxMapLimit() {
        return xMapLimit;
    }

    public int getyMapLimit() {
        return yMapLimit;
    }

    public int getStartingX() {
        return startingX;
    }

    public void setStartingX(int startingX) {
        this.startingX = startingX;
    }

    public int getStartingY() {
        return startingY;
    }

    public void setStartingY(int startingY) {
        this.startingY = startingY;
    }


}
