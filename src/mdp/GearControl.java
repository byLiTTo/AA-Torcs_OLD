package mdp;

import torcs.SensorModel;

public class GearControl {

    // Gear Variables --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private static final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    private static final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static States evaluateGearState(SensorModel current) {
        int gear = current.getGear();
        double rpm = current.getRPM();

        // if gear is 0 (N) or -1 (R) just return 1
        if (gear < 1)
            return States.NEUTRAL_REVERSE;
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return States.ENOUGH_RPM_UP;
        else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1])
                return States.LOW_RPM;
            else // otherwhise keep current gear
                return States.REVOLUTIONIZING_ENGINE;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static int gearAction2Double(SensorModel current, Actions action) {
        switch (action) {
            case ACTIVE_LIMITER -> {
                return 1;
            }
            case GEAR_UP -> {
                return current.getGear() + 1;
            }
            case GEAR_DOWN -> {
                return current.getGear() - 1;
            }
            case KEEP_GEAR -> {
                return current.getGear();
            }
        }

        return 0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double calculateReward(SensorModel previous, SensorModel current) {
        int gear = previous.getGear();
        double rpm = previous.getRPM();

        // if gear is 0 (N) or -1 (R) just return 1
        if (gear < 1)
            return current.getGear() == 1 ? 100.0 : -100.0;
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return gear < current.getGear() ? 100.0 : -100.0;
        else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1])
                return gear > current.getGear() ? 100.0 : -100.0;
            else // otherwhise keep current gear
                return gear == current.getGear() ? 100.0 : -100.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        ACTIVE_LIMITER,
        GEAR_UP,
        GEAR_DOWN,
        KEEP_GEAR
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        NEUTRAL_REVERSE,
        ENOUGH_RPM_UP,
        LOW_RPM,
        REVOLUTIONIZING_ENGINE
    }
}
