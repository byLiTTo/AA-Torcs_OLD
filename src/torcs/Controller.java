package torcs;

/**
 * The Controller class is an abstract class that serves as the base for implementing TORCS controllers.
 * It provides common methods and properties for controlling the car's behavior in different stages of the race.
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: N/A</p>
 * <p>Time: N/A</p>
 */
public abstract class Controller {

    private Stage stage;
    private String trackName;

    /**
     * Initializes an array of angles for the car's orientation.
     *
     * @return An array of angles for the car's orientation.
     */
    public float[] initAngles() {
        float[] angles = new float[19];
        for (int i = 0; i < 19; ++i)
            angles[i] = -90 + i * 10;
        return angles;
    }

    /**
     * Retrieves the current stage of the race.
     *
     * @return The current stage of the race.
     */
    public Stage getStage() {
        return stage;
    }

    /**
     * Sets the current stage of the race.
     *
     * @param stage The current stage of the race.
     */
    public void setStage(Stage stage) {
        this.stage = stage;
    }

    /**
     * Retrieves the name of the track.
     *
     * @return The name of the track.
     */
    public String getTrackName() {
        return trackName;
    }

    /**
     * Sets the name of the track.
     *
     * @param trackName The name of the track.
     */
    public void setTrackName(String trackName) {
        this.trackName = trackName;
    }

    /**
     * Controls the car's behavior based on the sensor readings.
     *
     * @param sensors The sensor readings from the car.
     *
     * @return The action to be performed by the car.
     */
    public abstract Action control(SensorModel sensors);

    /**
     * Resets the controller's state at the beginning of each new trial.
     */
    public abstract void reset();

    /**
     * Performs any necessary cleanup or shutdown operations.
     */
    public abstract void shutdown();

    /**
     * The Stage enum represents the different stages of the race.
     */
    public enum Stage {

        WARMUP, QUALIFYING, RACE, UNKNOWN;

        /**
         * Converts an integer value to the corresponding Stage enum value.
         *
         * @param value The integer value representing the stage.
         *
         * @return The Stage enum value.
         */
        static Stage fromInt(int value) {
            switch (value) {
                case 0:
                    return WARMUP;
                case 1:
                    return QUALIFYING;
                case 2:
                    return RACE;
                default:
                    return UNKNOWN;
            }
        }
    }

}