package mdp;

import torcs.SensorModel;

/**
 * The GearControl class handles the control and evaluation of gears in a TORCS racing simulation.
 */
public class GearControl {

    // Gear Variables
    private static final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    private static final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};

    /**
     * Evaluates the current gear state based on the sensor model.
     *
     * @param current The current sensor model of the car.
     *
     * @return The state of the gear as an enum value.
     */
    public static States evaluateGearState(SensorModel current) {
        int gear = current.getGear();
        double rpm = current.getRPM();

        if (gear < 1)
            return States.NEUTRAL_REVERSE;
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return States.ENOUGH_RPM_UP;
        else if (gear > 1 && rpm <= gearDown[gear - 1])
            return States.LOW_RPM;
        else
            return States.REVOLUTIONIZING_ENGINE;
    }

    /**
     * Converts the gear action to a double value based on the current sensor model and action.
     *
     * @param current The current sensor model of the car.
     * @param action  The gear control action.
     *
     * @return The double value representing the gear action.
     */
    public static int gearAction2Double(SensorModel current, Actions action) {
        switch (action) {
            case ACTIVE_LIMITER:
                return 1;
            case GEAR_UP:
                return current.getGear() + 1;
            case GEAR_DOWN:
                return current.getGear() - 1;
            case KEEP_GEAR:
                return current.getGear();
        }

        return 0;
    }

    /**
     * Calculates the reward based on the previous and current sensor models.
     *
     * @param previous The previous sensor model of the car.
     * @param current  The current sensor model of the car.
     *
     * @return The reward value.
     */
    public static double calculateReward(SensorModel previous, SensorModel current) {
        int gear = previous.getGear();
        double rpm = previous.getRPM();

        if (gear < 1)
            return current.getGear() == 1 ? 100.0 : -100.0;
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return gear < current.getGear() ? 100.0 : -100.0;
        else if (gear > 1 && rpm <= gearDown[gear - 1])
            return gear > current.getGear() ? 100.0 : -100.0;
        else
            return gear == current.getGear() ? 100.0 : -100.0;
    }

    /**
     * The gear control actions.
     */
    public enum Actions {
        ACTIVE_LIMITER,
        GEAR_UP,
        GEAR_DOWN,
        KEEP_GEAR
    }

    /**
     * The gear states.
     */
    public enum States {
        NEUTRAL_REVERSE,
        ENOUGH_RPM_UP,
        LOW_RPM,
        REVOLUTIONIZING_ENGINE
    }
}