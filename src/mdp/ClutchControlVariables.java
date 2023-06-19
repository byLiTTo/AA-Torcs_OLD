package mdp;

public class ClutchControlVariables {

    public enum Actions {
        KEEP_STEERING_WHEEL_STRAIGHT,
        TURN_STEERING_WHEEL,
//        TURN_STEERING_WHEEL_SHARPLY
    }

    public enum States {
        OFF_TRACK,
        IN_STRAIGHT_LINE,
        IN_CURVE_LOW_SPEED,
        IN_CURVE_HIGH_SPEED,
        IN_CURVE_ENOUGH_SPEED,
        IN_CURVE_BLOCKING
    }
}
