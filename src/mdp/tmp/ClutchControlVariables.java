package mdp.tmp;

import torcs.Controller;
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
            return States.FIRST_GEAR;
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

    public static GearClutch clutchAction2Double(SensorModel sensorModel, ClutchControlVariables.Actions actionPerformed,
                                                 float clutch, Controller.Stage stage) {
        int gear = sensorModel.getGear();
        float c = clutching(sensorModel, clutch, stage);
        if (actionPerformed == Actions.IDLE_SPEED) {

            return new GearClutch(1, c);
        }
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (actionPerformed == Actions.UP_GEAR) {
            return new GearClutch(gear + 1, c);
        } else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (actionPerformed == Actions.DOWN_GEAR) {
                return new GearClutch(gear - 1, c);
            } else // otherwhise keep current gear
            {
                return new GearClutch(gear, c);
            }
    }

    public static double calculateReward(SensorModel lastSensorModel, SensorModel currentSensorModel) {
        if (lastSensorModel == null) return 0.0;
        else {
            int lastGear = lastSensorModel.getGear();
            int currentGear = currentSensorModel.getGear();
            double rpm = lastSensorModel.getRPM();
            if (lastGear == 1 && currentGear == 0) return -100.0;
            // if gear is 0 (N) or -1 (R) just return 1
            if (lastGear < 1) {
                if (currentGear == 1) return 100.0;
                else return -10.0;
            }
            // check if the RPM value of car is greater than the one suggested
            // to shift up the gear from the current one
            if (lastGear < 6 && rpm >= gearUp[lastGear - 1]) {
                if (lastGear < currentGear) return 100.0;
                else return -10.0;
            } else
                // check if the RPM value of car is lower than the one suggested
                // to shift down the gear from the current one
                if (lastGear > 1 && rpm <= gearDown[lastGear - 1]) {
                    if (lastGear > currentGear) return 100.0;
                    else return -10.0;
                } else // otherwhise keep current gear
                {
                    if (lastGear == currentGear) return 100.0;
                    else return -10.0;
                }
        }
    }

    private static float clutching(SensorModel sensors, float clutch, Controller.Stage stage) {

        float maxClutch = clutchMax;

        // Check if the current situation is the race start
        if (sensors.getCurrentLapTime() < clutchDeltaTime
                && stage == Controller.Stage.RACE
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
        IDLE_SPEED,
        KEEP_GEAR,
        UP_GEAR,
        DOWN_GEAR
    }

    public enum States {
        FIRST_GEAR,
        ENOUGH_RPM_UP,
        REVOLUTIONIZING_ENGINE,
        LOWER_RPM_DOWN
    }
}
