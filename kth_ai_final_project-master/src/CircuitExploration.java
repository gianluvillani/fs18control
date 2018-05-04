public class CircuitExploration {

    public static void main(String[] args)
    {
        String filename;
        if(args.length >= 1)
        {
            filename = args[1];
        } else {
            filename = "map_circuit.txt";
        }

        Server server = new Server(filename);

        Coordinate startingPosition = new Coordinate(server.getStartingX(), server.getStartingY());

        Agent agent = new Agent(startingPosition, server);

        agent.performMoveAction();

    }


}
