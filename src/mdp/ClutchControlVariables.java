package mdp;

import torcs.SensorModel;

public class ClutchControlVariables {

    static final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    static final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Clutching Constants */
    static final float clutchMax = (float) 0.5;
    static final float clutchDelta = (float) 0.05;
    static final float clutchRange = (float) 0.82;
    static final float clutchDeltaTime = (float) 0.02;
    static final float clutchDeltaRaced = 10;
    static final float clutchDec = (float) 0.01;
    static final float clutchMaxModifier = (float) 1.3;
    static final float clutchMaxTime = (float) 1.5;

    public static ClutchControlVariables.States evaluateClutchState(SensorModel sensorModel) {
        int gear = sensorModel.getGear();
        double rpm = sensorModel.getRPM();
        if (gear < 1) {
            return States.STARTING_GRID;
        }
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1]) {
            return States.ENOUGH_RPM_UP;
        } else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1]) {
                return States.LOWER_RPM_DOWN;
            } else // otherwhise keep current gear
            {
                return States.REVOLUTIONIZING_ENGINE;
            }
    }

    public static Float[] clutchAction2Double(SensorModel sensorModel, ClutchControlVariables.Actions actionPerformed,
            float clutch) {
        int gear = sensorModel.getGear();
        Float[] gear_clucth = new Float[2];
        gear_clucth[1] = clutching(sensorModel, clutch);
        if (actionPerformed == Actions.RACE_START) {
            gear_clucth[0] = 1f;
        }// check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (actionPerformed == Actions.UP_GEAR) {
            gear_clucth[0] = gear + 1f;
        } else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (actionPerformed == Actions.DOWN_GEAR) {
                gear_clucth[0] = gear - 1f;
            } else // otherwhise keep current gear
            {
                gear_clucth[0] = (float) gear;
            }

        return gear_clucth;
    }

    private static float clutching(SensorModel sensors, float clutch) {

        float maxClutch = clutchMax;

        // Check if the current situation is the race start
        if (sensors.getCurrentLapTime() < clutchDeltaTime
                && sensors.getDistanceRaced() < clutchDeltaRaced) {
            clutch = maxClutch;
        }

        // Adjust the current value of the clutch
        if (clutch > 0) {
            double delta = clutchDelta;
            if (sensors.getGear() < 2) {
                // Apply a stronger clutch output when the gear is one and the race is just started
                delta /= 2;
                maxClutch *= clutchMaxModifier;
                if (sensors.getCurrentLapTime() < clutchMaxTime) {
                    clutch = maxClutch;
                }
            }

            // check clutch is not bigger than maximum values
            clutch = Math.min(maxClutch, clutch);

            // if clutch is not at max value decrease it quite quickly
            if (clutch != maxClutch) {
                clutch -= delta;
                clutch = Math.max((float) 0.0, clutch);
            }
            // if clutch is at max value decrease it very slowly
            else {
                clutch -= clutchDec;
            }
        }
        return clutch;
    }

    private int getGear(SensorModel sensors) {
        int gear = sensors.getGear();
        double rpm = sensors.getRPM();

        // if gear is 0 (N) or -1 (R) just return 1
        if (gear < 1) {
            return 1;
        }
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1]) {
            return gear + 1;
        } else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1]) {
                return gear - 1;
            } else // otherwhise keep current gear
            {
                return gear;
            }
    }

    public enum Actions {
        RACE_START,
        KEEP_GEAR,
        UP_GEAR,
        DOWN_GEAR
    }

    public enum States {
        STARTING_GRID,
        ENOUGH_RPM_UP,
        REVOLUTIONIZING_ENGINE,
        LOWER_RPM_DOWN
    }
}
