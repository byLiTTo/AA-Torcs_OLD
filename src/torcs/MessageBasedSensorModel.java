package torcs;

/**
 * The MessageBasedSensorModel class implements the SensorModel interface by utilizing the readings parsed from a message.
 * It provides methods to retrieve various sensor readings based on the parsed message.
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: Mar 4, 2008</p>
 * <p>Time: 3:44:29 PM</p>
 */
public class MessageBasedSensorModel implements SensorModel {

    private MessageParser message; // The parsed message containing the sensor readings

    /**
     * Constructs a new MessageBasedSensorModel with the specified MessageParser object.
     *
     * @param message The MessageParser object containing the parsed message.
     */
    public MessageBasedSensorModel(MessageParser message) {
        this.message = message;
    }

    /**
     * Constructs a new MessageBasedSensorModel with the specified message string.
     * The message string will be parsed to extract the sensor readings.
     *
     * @param strMessage The message string to be parsed.
     */
    public MessageBasedSensorModel(String strMessage) {
        this.message = new MessageParser(strMessage);
    }

    /**
     * Retrieves the current speed of the car.
     *
     * @return The car's speed in units per second.
     */
    public double getSpeed() {
        return (Double) message.getReading("speedX");
    }

    /**
     * Retrieves the angle between the car's orientation and the track's axis.
     *
     * @return The angle to the track's axis in radians.
     */
    public double getAngleToTrackAxis() {
        return (Double) message.getReading("angle");
    }

    /**
     * Retrieves an array of sensor readings representing the track's edge distances.
     *
     * @return An array of track edge sensor readings.
     */
    public double[] getTrackEdgeSensors() {
        return (double[]) message.getReading("track");
    }

    /**
     * Retrieves an array of sensor readings representing the focus on specific objects in the environment.
     *
     * @return An array of focus sensor readings.
     */
    public double[] getFocusSensors() {
        return (double[]) message.getReading("focus");
    }

    /**
     * Retrieves the current gear of the car.
     *
     * @return The car's gear as an integer value.
     */
    public int getGear() {
        return (int) (double) (Double) message.getReading("gear");
    }

    /**
     * Retrieves an array of sensor readings representing other cars on the track.
     *
     * @return An array of opponent sensor readings.
     */
    public double[] getOpponentSensors() {
        return (double[]) message.getReading("opponents");
    }

    /**
     * Retrieves the current race position of the car.
     *
     * @return The car's race position as an integer value.
     */
    public int getRacePosition() {
        return (int) (double) (Double) message.getReading("racePos");
    }

    /**
     * Retrieves the lateral speed of the car.
     *
     * @return The car's lateral speed in units per second.
     */
    public double getLateralSpeed() {
        return (Double) message.getReading("speedY");
    }

    /**
     * Retrieves the current lap time of the car.
     *
     * @return The car's current lap time in seconds.
     */
    public double getCurrentLapTime() {
        return (Double) message.getReading("curLapTime");
    }

    /**
     * Retrieves the damage level of the car.
     *
     * @return The car's damage level
     * <p>
     * as a value between 0.0 (no damage) and 1.0 (severe damage).
     */
    public double getDamage() {
        return (Double) message.getReading("damage");
    }

    /**
     * Retrieves the distance from the start line to the car's current position.
     *
     * @return The car's distance from the start line in meters.
     */
    public double getDistanceFromStartLine() {
        return (Double) message.getReading("distFromStart");
    }

    /**
     * Retrieves the total distance raced by the car.
     *
     * @return The car's total distance raced in meters.
     */
    public double getDistanceRaced() {
        return (Double) message.getReading("distRaced");
    }

    /**
     * Retrieves the current fuel level of the car.
     *
     * @return The car's fuel level as a percentage.
     */
    public double getFuelLevel() {
        return (Double) message.getReading("fuel");
    }

    /**
     * Retrieves the time taken to complete the last lap.
     *
     * @return The car's last lap time in seconds.
     */
    public double getLastLapTime() {
        return (Double) message.getReading("lastLapTime");
    }

    /**
     * Retrieves the current RPM (Revolutions Per Minute) of the car's engine.
     *
     * @return The car's current RPM.
     */
    public double getRPM() {
        return (Double) message.getReading("rpm");
    }

    /**
     * Retrieves the car's position on the track.
     *
     * @return The car's track position as a value between -1.0 (left) and 1.0 (right).
     */
    public double getTrackPosition() {
        return (Double) message.getReading("trackPos");
    }

    /**
     * Retrieves an array of sensor readings representing the wheel spin velocities.
     *
     * @return An array of wheel spin velocity sensor readings.
     */
    public double[] getWheelSpinVelocity() {
        return (double[]) message.getReading("wheelSpinVel");
    }

    /**
     * Retrieves the original message that was parsed.
     *
     * @return The original message as a string.
     */
    public String getMessage() {
        return message.getMessage();
    }

    /**
     * Retrieves the z-coordinate of the car's position.
     *
     * @return The car's z-coordinate.
     */
    public double getZ() {
        return (Double) message.getReading("z");
    }

    /**
     * Retrieves the speed of the car along the z-axis (vertical axis).
     *
     * @return The car's z-axis speed in units per second.
     */
    public double getZSpeed() {
        return (Double) message.getReading("speedZ");
    }
}