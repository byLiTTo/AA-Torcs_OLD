package torcs;

/**
 * The Action class represents the actions to be taken by the car in the TORCS environment.
 * It encapsulates the acceleration, brake, clutch, gear, steering, restartRace, and focus values.
 * <p> </p>
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: Mar 4, 2008</p>
 * <p>Time: 3:35:31 PM</p>
 */
public class Action {

    public double accelerate = 0; // 0..1
    public double brake = 0; // 0..1
    public double clutch = 0; // 0..1
    public int gear = 0; // -1..6
    public double steering = 0;  // -1..1
    public boolean restartRace = false;
    public int focus = 360; // Desired focus angle in degrees [-90; 90], set to 360 if no focus reading is desired!

    /**
     * Returns the string representation of the Action object.
     *
     * @return The string representation of the Action object.
     */
    public String toString() {
        limitValues();
        return "(accel " + accelerate + ") " +
                "(brake " + brake + ") " +
                "(clutch " + clutch + ") " +
                "(gear " + gear + ") " +
                "(steer " + steering + ") " +
                "(meta " + (restartRace ? 1 : 0) + ") " +
                "(focus " + focus + ")";
    }

    /**
     * Limits the values of the action variables to their valid ranges.
     */
    public void limitValues() {
        accelerate = Math.max(0, Math.min(1, accelerate));
        brake = Math.max(0, Math.min(1, brake));
        clutch = Math.max(0, Math.min(1, clutch));
        steering = Math.max(-1, Math.min(1, steering));
        gear = Math.max(-1, Math.min(6, gear));
    }
}
