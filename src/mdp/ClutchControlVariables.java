package mdp;

import torcs.SensorModel;

public class ClutchControlVariables {

    final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};

    private int getGear(SensorModel sensors) {
        int gear = sensors.getGear();
        double rpm = sensors.getRPM();

        // if gear is 0 (N) or -1 (R) just return 1
        if (gear < 1)
            return 1;
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return gear + 1;
        else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1])
                return gear - 1;
            else // otherwhise keep current gear
                return gear;
    }

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
