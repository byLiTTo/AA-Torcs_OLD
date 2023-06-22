package drivers.trainer;

//   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

import mdp.AccelControlVariables;
import mdp.QLearning;
import mdp.SteerControlVariables;
import torcs.Action;
import torcs.Controller;
import torcs.SensorModel;

import static mdp.AccelControlVariables.evaluateAccelState;
import static mdp.SteerControlVariables.evaluateSteerState;
import static mdp.SteerControlVariables.steerAction2Double;
import static torcs.Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM;
import static torcs.Constants.ControlSystems.STEERING_CONTROL_SYSTEM;
import static torcs.Constants.SEPARATOR;


//   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

/**
 * The TurnerDriver class represents a controller for a racing car in the Car Racing Competition 2011.
 * It utilizes a Q-learning algorithm for steering control and makes decisions based on sensor inputs.
 */
public class AutomaticTransmissionDriver extends Controller {

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
    final float maxSpeedDist = 70;
    final float maxSpeed = 150;
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
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Common Variables */
    private SensorModel lastSensorModel;
    private SensorModel currentSensorModel;
    private int maxEpochs = 20;
    private int rangeEpochs = 15;

    private int epochs;
    private int laps;
    private double distanceRaced;
    private long currentTime;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Constructs a new instance of the TurnerDriver.
     */
    public AutomaticTransmissionDriver() {
        this.steerControlSystem = new QLearning(STEERING_CONTROL_SYSTEM, rangeEpochs);
        this.currentSteerState = SteerControlVariables.States.CENTER_AXIS;
        this.steerAction = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;

        this.accelControlSystem = new QLearning(ACCELERATION_CONTROL_SYSTEM, rangeEpochs);
        this.currentAccelState = AccelControlVariables.States.IN_STRAIGHT_LINE;
        this.accelAction = AccelControlVariables.Actions.PRESS_FULL_THROTTLE;

        this.epochs = 0;
        this.laps = 0;
        this.distanceRaced = 0;
        this.currentTime = System.currentTimeMillis();
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Controls the car based on the sensor inputs.
     *
     * @param sensorModel The sensor model containing information about the car's surroundings.
     * @return The action to be performed by the car (acceleration, brake, gear, and steering).
     */
    public Action control(SensorModel sensorModel) {

        int currentLaps = (int) (sensorModel.getDistanceRaced() / this.trackLenght);
        if (currentLaps < this.laps) {
            if (Math.abs(sensorModel.getTrackPosition()) >= 1) {
//                this.steerControlSystem.lastUpdate(this.actionPerformed, -1000.0);
                Action action = new Action();
                action.restartRace = true;
                return action;
            } else {
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
                    // compute gear
                    int gear = getGear(sensorModel);

                    clutch = clutching(sensorModel, clutch);

                    // build a CarControl variable and return it
                    Action action = new Action();
                    action.gear = gear;

                    long timeTranscurred = (System.currentTimeMillis() - this.currentTime);
                    if (timeTranscurred > 1) {
                        this.currentTime = System.currentTimeMillis();
                        System.out.println("EPOCH: " + (this.epochs + 1) + "/" + this.maxEpochs);
                        System.out.println("Complete Laps: " + laps);
                        System.out.println("State: " + this.currentAccelState.name());
                        System.out.println("Action: " + this.accelAction.name());
                        System.out.println();
                        /* Update variables for steer control system -------------------------------------------------- */
                        this.lastSensorModel = this.currentSensorModel;
                        this.currentSensorModel = sensorModel;
                        this.lastSteerState = this.currentSteerState;
                        this.currentSteerState = SteerControlVariables.evaluateSteerState(sensorModel);
                        this.steerReward = SteerControlVariables.calculateReward(this.lastSensorModel, sensorModel);
                        this.steerAction = (SteerControlVariables.Actions) this.steerControlSystem.Update(
                                this.lastSteerState,
                                this.currentSteerState,
                                this.steerAction,
                                this.steerReward
                        );
                        action.steering = SteerControlVariables.steerAction2Double(sensorModel, this.steerAction);
                        /* Update variables for accel control system -------------------------------------------------- */
                        this.lastSensorModel = this.currentSensorModel;
                        this.currentSensorModel = sensorModel;
                        this.lastAccelState = this.currentAccelState;
                        this.currentAccelState = AccelControlVariables.evaluateAccelState(sensorModel);
                        this.accelReward = AccelControlVariables.calculateReward(this.lastSensorModel, this.currentSensorModel);
                        this.accelAction = (AccelControlVariables.Actions) this.accelControlSystem.Update(
                                this.lastAccelState,
                                this.currentAccelState,
                                this.accelAction,
                                this.accelReward
                        );
                        Double[] accel_brake = AccelControlVariables.accelAction2Double(this.currentSensorModel, this.accelAction);
                        action.accelerate = accel_brake[0];
                        action.brake = accel_brake[1];
                        /* -------------------------------------------------------------------------------------------- */
                    } else {
                        /* Update variables for steer control system -------------------------------------------------- */
                        this.currentSteerState = evaluateSteerState(sensorModel);
                        this.steerAction = (SteerControlVariables.Actions) this.steerControlSystem.nextOnlyBestAction(this.currentSteerState);
                        action.steering = steerAction2Double(sensorModel, this.steerAction);
                        /* Update variables for accel control system -------------------------------------------------- */
                        this.currentAccelState = evaluateAccelState(sensorModel);
                        this.accelAction = (AccelControlVariables.Actions) this.accelControlSystem.nextOnlyBestAction(this.currentAccelState);
                        Double[] accel_brake = AccelControlVariables.accelAction2Double(sensorModel, this.accelAction);
                        action.accelerate = accel_brake[0];
                        action.brake = accel_brake[1];
                        /* -------------------------------------------------------------------------------------------- */
                    }
                    action.clutch = clutch;
                    return action;
                }
            }
        } else {
            this.laps++;
            Action action = new Action();
            action.restartRace = true;
            return action;
        }

    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Resets the state of the driver.
     */
    @Override
    public void reset() {
        this.lastSteerState = SteerControlVariables.States.CENTER_AXIS;
        this.currentSteerState = lastSteerState;
        this.steerAction = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;
        this.lastAccelState = AccelControlVariables.States.IN_STRAIGHT_LINE;
        this.currentAccelState = this.lastAccelState;
        this.accelAction = AccelControlVariables.Actions.PRESS_FULL_THROTTLE;
        this.epochs++;
        String newResults = this.generateStatistics();
        this.steerControlSystem.saveTable();
        this.accelControlSystem.saveQTableAndStatistics(newResults);
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
        this.steerControlSystem.saveTable();
        this.accelControlSystem.saveQTableAndStatistics(newResults);
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
