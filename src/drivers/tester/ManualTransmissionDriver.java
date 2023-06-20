package drivers.tester;

//   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

import static torcs.Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM;
import static torcs.Constants.ControlSystems.CLUTCH_CONTROL_SYSTEM;
import static torcs.Constants.ControlSystems.STEERING_CONTROL_SYSTEM;
import static torcs.Constants.SEPARATOR;

import mdp.AccelControlVariables;
import mdp.ClutchControlVariables;
import mdp.ClutchControlVariables.Actions;
import mdp.ClutchControlVariables.States;
import mdp.QLearning;
import mdp.SteerControlVariables;
import torcs.Action;
import torcs.Controller;
import torcs.SensorModel;

//   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

/**
 * The TurnerDriver class represents a controller for a racing car in the Car Racing Competition 2011.
 * It utilizes a Q-learning algorithm for steering control and makes decisions based on sensor inputs.
 */
public class ManualTransmissionDriver extends Controller {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Gear Changing Constants */
    final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Stuck constants */
    final int stuckTime = 25;
    final float stuckAngle = (float) 0.523598775; //PI/6

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Accel and Brake Constants */
    final float maxSpeedDist = 270;
    final float maxSpeed = 350;
    final float sin5 = (float) 0.08716;
    final float cos5 = (float) 0.99619;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Steering constants */
    final float steerLock = (float) 0.785398;
    final float steerSensitivityOffset = (float) 80.0;
    final float wheelSensitivityCoeff = 1;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* ABS Filter Constants */
    final float wheelRadius[] = {(float) 0.3179, (float) 0.3179, (float) 0.3276, (float) 0.3276};
    final float absSlip = (float) 2.0;
    final float absRange = (float) 3.0;
    final float absMinSpeed = (float) 3.0;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Clutching Constants */
    final float clutchMax = (float) 0.5;
    final float clutchDelta = (float) 0.05;
    final float clutchRange = (float) 0.82;
    final float clutchDeltaTime = (float) 0.02;
    final float clutchDeltaRaced = 10;
    final float clutchDec = (float) 0.01;
    final float clutchMaxModifier = (float) 1.3;
    final float clutchMaxTime = (float) 1.5;
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* System Control Variables */
    private final QLearning steerControlSystem;
    private final QLearning accelControlSystem;
    private final QLearning clutchControlSystem;
    private final double trackLenght = 2057.56;
    private int stuck = 0;
    // current clutch
    private float clutch = 0;
    private SteerControlVariables.States lastSteerState;
    private SteerControlVariables.States currentSteerState;
    private SteerControlVariables.Actions steerAction;
    private double steerReward;
    private AccelControlVariables.States lastAccelState;
    private AccelControlVariables.States currentAccelState;
    private AccelControlVariables.Actions accelAction;
    private double accelReward;
    private ClutchControlVariables.States lastClutchState;
    private ClutchControlVariables.States currentClutchState;
    private ClutchControlVariables.Actions clutchAction;
    private double clutchReward;
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Common Variables */
    private SensorModel lastSensorModel;
    private SensorModel currentSensorModel;
    private int maxEpochs = 4;
    private int epochs;
    private int laps;
    private double distanceRaced;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Constructs a new instance of the TurnerDriver.
     */
    public ManualTransmissionDriver() {
        this.steerControlSystem = new QLearning(STEERING_CONTROL_SYSTEM);
        this.currentSteerState = SteerControlVariables.States.STARTING_GRID;
        this.steerAction = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;

        this.accelControlSystem = new QLearning(ACCELERATION_CONTROL_SYSTEM);
        this.currentAccelState = AccelControlVariables.States.OFF_TRACK;
        this.accelAction = AccelControlVariables.Actions.ACTIVE_LIMITER;

        this.clutchControlSystem = new QLearning(CLUTCH_CONTROL_SYSTEM);
        this.currentClutchState = States.STARTING_GRID;
        this.clutchAction = Actions.RACE_START;

        this.epochs = 0;
        this.laps = 0;
        this.distanceRaced = 0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Controls the car based on the sensor inputs.
     *
     * @param sensorModel The sensor model containing information about the car's surroundings.
     * @return The action to be performed by the car (acceleration, brake, gear, and steering).
     */
    public Action control(SensorModel sensorModel) {
        System.out.println("EPOCH: " + (this.epochs + 1) + "/" + this.maxEpochs);
        System.out.println("Complete Laps: " + laps);
        System.out.println("State: " + this.currentSteerState.name());
        System.out.println("Action: " + this.steerAction.name());
        System.out.println();

        this.distanceRaced = sensorModel.getDistanceRaced();

//        if (this.distanceRaced < (this.trackLenght - 0.1)) {

        // check if car is currently stuck
        if (Math.abs(sensorModel.getAngleToTrackAxis()) > stuckAngle) {
            // update stuck counter
            stuck++;
        } else {
            // if not stuck reset stuck counter
            stuck = 0;
        }

        // after car is stuck for a while apply recovering policy
        if (stuck > stuckTime) {
            /* set gear and sterring command assuming car is
             * pointing in a direction out of track */

            // to bring car parallel to track axis
            float steer = (float) (-sensorModel.getAngleToTrackAxis() / steerLock);
            int gear = -1; // gear R

            // if car is pointing in the correct direction revert gear and steer
            if (sensorModel.getAngleToTrackAxis() * sensorModel.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }
            clutch = clutching(sensorModel, clutch);
            // build a CarControl variable and return it
            Action action = new Action();
            action.gear = gear;
            action.steering = steer;
            action.accelerate = 1.0;
            action.brake = 0;
            action.clutch = clutch;
            return action;
        } else // car is not stuck
        {

            // build a CarControl variable and return it
            Action action = new Action();
            /* Update variables for steer control system ------------------------------------------------------------ */
            this.currentSteerState = SteerControlVariables.evaluateSteerState(sensorModel);
            this.steerAction = (SteerControlVariables.Actions) this.steerControlSystem.nextOnlyBestAction(
                    this.currentSteerState);
            action.steering = SteerControlVariables.steerAction2Double(sensorModel, this.steerAction);
            /* Update variables for accel control system ------------------------------------------------------------ */
            this.currentAccelState = AccelControlVariables.evaluateAccelState(sensorModel);
            this.accelAction = (AccelControlVariables.Actions) this.accelControlSystem.nextOnlyBestAction(
                    this.currentAccelState);
            Double[] accel_brake = AccelControlVariables.accelAction2Double(sensorModel, this.accelAction);
            action.accelerate = accel_brake[0];
            action.brake = accel_brake[1];
            /* Update variables for clutch control system ----------------------------------------------------------- */
            this.currentClutchState = ClutchControlVariables.evaluateClutchState(sensorModel);
            this.clutchAction = (ClutchControlVariables.Actions) this.clutchControlSystem.nextOnlyBestAction(
                    this.currentClutchState);
            Float[] gear_clutch = ClutchControlVariables.clutchAction2Double(sensorModel, this.clutchAction, clutch);
            float gear = gear_clutch[0];
            action.gear = (int) gear;
            clutch = gear_clutch[1];
            action.clutch = clutch;
            /* ------------------------------------------------------------------------------------------------------ */
            return action;
        }
//        } else {
//            this.laps++;
//            Action action = new Action();
//            action.restartRace = true;
//            return action;
//        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Resets the state of the driver.
     */
    @Override
    public void reset() {
        this.lastSteerState = SteerControlVariables.States.STARTING_GRID;
        this.currentSteerState = lastSteerState;
        this.steerAction = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;
        this.epochs++;
        String newResults = this.generateStatistics();
        this.steerControlSystem.saveStatistics(newResults);
        System.out.println("Restarting the race!");

    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Shuts down the driver and saves the Q-learning results.
     */
    @Override
    public void shutdown() {
        this.epochs++;
        String newResults = this.generateStatistics();
        this.steerControlSystem.saveStatistics(newResults);
        System.out.println("Bye bye!");
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Initializes the angles used for sensor readings.
     *
     * @return An array of angles used for sensor readings.
     */
    @Override
    public float[] initAngles() {

        float[] angles = new float[19];

        /* set angles as {-90,-75,-60,-45,-30,-20,-15,-10,-5,0,5,10,15,20,30,45,60,75,90} */
        for (int i = 0; i < 5; i++) {
            angles[i] = -90 + i * 15;
            angles[18 - i] = 90 - i * 15;
        }

        for (int i = 5; i < 9; i++) {
            angles[i] = -20 + (i - 5) * 5;
            angles[18 - i] = 20 - (i - 5) * 5;
        }
        angles[9] = 0;
        return angles;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Adjusts the clutch value based on the current sensor readings and racing conditions.
     * The clutch value determines the engagement of the clutch mechanism in the car.
     *
     * @param sensors The sensor model containing information about the car's state.
     * @param clutch  The current clutch value.
     * @return The updated clutch value based on the racing conditions.
     */
    private float clutching(SensorModel sensors, float clutch) {

        float maxClutch = clutchMax;

        // Check if the current situation is the race start
        if (sensors.getCurrentLapTime() < clutchDeltaTime
                && getStage() == Stage.RACE
                && sensors.getDistanceRaced() < clutchDeltaRaced)
            clutch = maxClutch;

        // Adjust the current value of the clutch
        if (clutch > 0) {
            double delta = clutchDelta;
            if (sensors.getGear() < 2) {
                // Apply a stronger clutch output when the gear is one and the race is just started
                delta /= 2;
                maxClutch *= clutchMaxModifier;
                if (sensors.getCurrentLapTime() < clutchMaxTime)
                    clutch = maxClutch;
            }

            // check clutch is not bigger than maximum values
            clutch = Math.min(maxClutch, clutch);

            // if clutch is not at max value decrease it quite quickly
            if (clutch != maxClutch) {
                clutch -= delta;
                clutch = Math.max((float) 0.0, clutch);
            }
            // if clutch is at max value decrease it very slowly
            else
                clutch -= clutchDec;
        }
        return clutch;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->


    /**
     * Computes the appropriate gear based on the current sensor readings.
     *
     * @param sensors The sensor model containing information about the car's state.
     * @return The selected gear value.
     */
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

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.maxEpochs + SEPARATOR
                + this.epochs + SEPARATOR
                + this.distanceRaced + SEPARATOR
                + this.laps;
    }
}