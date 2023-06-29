package torcs;

/**
 * The SensorModel interface provides methods to retrieve various sensor readings related to the car's state and the track.
 * These methods are useful for obtaining information during a race in the TORCS (The Open Racing Car Simulator) environment.
 *
 * <p>Implement this interface to create a custom sensor model that can be used for processing the sensor data.</p>
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: Mar 4, 2008</p>
 * <p>Time: 12:18:47 PM</p>
 */
public interface SensorModel {

    /**
     * Retrieves the current speed of the car.
     *
     * @return The car's speed in units per second.
     */
    public double getSpeed();

    /**
     * Retrieves the angle between the car's orientation and the track's axis.
     *
     * @return The angle to the track's axis in radians.
     */
    public double getAngleToTrackAxis();

    /**
     * Retrieves an array of sensor readings representing the track's edge distances.
     *
     * @return An array of track edge sensor readings.
     */
    public double[] getTrackEdgeSensors();

    /**
     * Retrieves an array of sensor readings representing the focus on specific objects in the environment.
     *
     * @return An array of focus sensor readings.
     */
    public double[] getFocusSensors();

    /**
     * Retrieves the car's position on the track.
     *
     * @return The car's track position as a value between -1.0 (left) and 1.0 (right).
     */
    public double getTrackPosition();

    /**
     * Retrieves the current gear of the car.
     *
     * @return The car's gear as an integer value.
     */
    public int getGear();

    /**
     * Retrieves an array of sensor readings representing other cars on the track.
     *
     * @return An array of opponent sensor readings.
     */
    public double[] getOpponentSensors();

    /**
     * Retrieves the current race position of the car.
     *
     * @return The car's race position as an integer value.
     */
    public int getRacePosition();

    /**
     * Retrieves the lateral speed of the car.
     *
     * @return The car's lateral speed in units per second.
     */
    public double getLateralSpeed();

    /**
     * Retrieves the current lap time of the car.
     *
     * @return The car's current lap time in seconds.
     */
    public double getCurrentLapTime();

    /**
     * Retrieves the damage level of the car.
     *
     * @return The car's damage level as a value between 0.0 (no damage) and 1.0 (severe damage).
     */
    public double getDamage();

    /**
     * Retrieves the distance from the start line to the car's current position.
     *
     * @return The car's distance from the start line in meters.
     */
    public double getDistanceFromStartLine();

    /**
     * Retrieves the total distance raced by the car.
     *
     * @return The car's total distance raced in meters.
     */
    public double getDistanceRaced();

    /**
     * Retrieves the current fuel level of the car.
     *
     * @return The car's fuel level as a percentage.
     */
    public double getFuelLevel();

    /**
     * Retrieves the time taken to complete the last lap.
     *
     * @return The car's last lap time in seconds.
     */
    public double getLastLapTime();

    /**
     * Retrieves the current RPM (Revolutions Per
     * <p>
     * Minute) of the car's engine.
     *
     * @return The car's current RPM.
     */
    public double getRPM();

    /**
     * Retrieves an array of sensor readings representing the wheel spin velocities.
     *
     * @return An array of wheel spin velocity sensor readings.
     */
    public double[] getWheelSpinVelocity();

    /**
     * Retrieves the speed of the car along the z-axis (vertical axis).
     *
     * @return The car's z-axis speed in units per second.
     */
    public double getZSpeed();

    /**
     * Retrieves the z-coordinate of the car's position.
     *
     * @return The car's z-coordinate.
     */
    public double getZ();

    /**
     * Retrieves any message associated with the car's state.
     *
     * @return The car's message as a string.
     */
    public String getMessage();
}