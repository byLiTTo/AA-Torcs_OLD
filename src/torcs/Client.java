package torcs;

import torcs.Controller.Stage;

import java.util.StringTokenizer;

/**
 * The Client class is responsible for connecting to the TORCS server, receiving game state information,
 * and controlling the car based on the provided controller implementation.
 * <p> </p>
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Daniele Loiacono</p>
 * <p>Date: N/A</p>
 * <p>Time: N/A</p>
 */
public class Client {

    private static int UDP_TIMEOUT = 10000;
    private static int port;
    private static String host;
    private static String clientId;
    private static boolean verbose;
    private static int maxEpisodes;
    private static int maxSteps;
    private static Stage stage;
    private static String trackName;

    /**
     * The main entry point of the client.
     *
     * @param args The command-line arguments.
     */
    public static void main(String[] args) {
        parseParameters(args);
        SocketHandler mySocket = new SocketHandler(host, port, verbose);
        String inMsg;

        Controller driver = load(args[0]);
        driver.setStage(stage);
        driver.setTrackName(trackName);

        /* Build init string */
        float[] angles = driver.initAngles();
        String initStr = clientId + "(init";
        for (int i = 0; i < angles.length; i++) {
            initStr = initStr + " " + angles[i];
        }
        initStr = initStr + ")";

        long curEpisode = 0;
        boolean shutdownOccurred = false;
        do {

            /*
             * Client identification
             */

            do {
                mySocket.send(initStr);
                inMsg = mySocket.receive(UDP_TIMEOUT);
            } while (inMsg == null || inMsg.indexOf("***identified***") < 0);

            /*
             * Start to drive
             */
            long currStep = 0;
            while (true) {
                /*
                 * Receives from TORCS the game state
                 */
                inMsg = mySocket.receive(UDP_TIMEOUT);

                if (inMsg != null) {

                    /*
                     * Check if race is ended (shutdown)
                     */
                    if (inMsg.indexOf("***shutdown***") >= 0) {
                        shutdownOccurred = true;
                        System.out.println("Server shutdown!");
                        break;
                    }

                    /*
                     * Check if race is restarted
                     */
                    if (inMsg.indexOf("***restart***") >= 0) {
                        driver.reset();
                        if (verbose)
                            System.out.println("Server restarting!");
                        break;
                    }

                    Action action = new Action();
                    if (currStep < maxSteps || maxSteps == 0)
                        action = driver.control(new MessageBasedSensorModel(inMsg));
                    else
                        action.restartRace = true;

                    currStep++;
                    mySocket.send(action.toString());
                } else
                    System.out.println("Server did not respond within the timeout");
            }

        } while (++curEpisode < maxEpisodes && !shutdownOccurred);

        /*
         * Shutdown the controller
         */
        driver.shutdown();
        mySocket.close();
        System.out.println("Client shutdown.");
        System.out.println("Bye, bye!");

    }

    /**
     * Parses the command-line parameters and sets the corresponding values.
     *
     * @param args The command-line arguments.
     */
    private static void parseParameters(String[] args) {
        /*
         * Set default values for the options
         */
        port = 3001;
        host = "localhost";
        clientId = "championship2011";
        verbose = false;
        maxEpisodes = 1;
        maxSteps = 0;
        stage = Stage.UNKNOWN;
        trackName = "unknown";

        for (int i = 1; i

                < args.length; i++) {
            StringTokenizer st = new StringTokenizer(args[i], ":");
            String entity = st.nextToken();
            String value = st.nextToken();
            if (entity.equals("port")) {
                port = Integer.parseInt(value);
            }
            if (entity.equals("host")) {
                host = value;
            }
            if (entity.equals("id")) {
                clientId = value;
            }
            if (entity.equals("verbose")) {
                if (value.equals("on"))
                    verbose = true;
                else if (value.equals(false))
                    verbose = false;
                else {
                    System.out.println(entity + ":" + value + " is not a valid option");
                    System.exit(0);
                }
            }
            if (entity.equals("id")) {
                clientId = value;
            }
            if (entity.equals("stage")) {
                stage = Stage.fromInt(Integer.parseInt(value));
            }
            if (entity.equals("trackName")) {
                trackName = value;
            }
            if (entity.equals("maxEpisodes")) {
                maxEpisodes = Integer.parseInt(value);
                if (maxEpisodes <= 0) {
                    System.out.println(entity + ":" + value + " is not a valid option");
                    System.exit(0);
                }
            }
            if (entity.equals("maxSteps")) {
                maxSteps = Integer.parseInt(value);
                if (maxSteps < 0) {
                    System.out.println(entity + ":" + value + " is not a valid option");
                    System.exit(0);
                }
            }
        }
    }

    /**
     * Loads an instance of the specified controller class.
     *
     * @param name The name of the controller class.
     *
     * @return The loaded controller instance.
     */
    private static Controller load(String name) {
        Controller controller = null;
        try {
            controller = (Controller) (Object) Class.forName(name).newInstance();
        } catch (ClassNotFoundException e) {
            System.out.println(name + " is not a class name");
            System.exit(0);
        } catch (InstantiationException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return controller;
    }
}
