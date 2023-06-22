package drivers;

//   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

import mdp.AccelControlVariables;
import mdp.QLearning;
import mdp.SteerControlVariables;
import torcs.*;

import static torcs.Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM;
import static torcs.Constants.ControlSystems.STEERING_CONTROL_SYSTEM;
import static torcs.Constants.SEPARATOR;

//   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

/**
 * The TurnerDriver class represents a controller for a racing car in the Car Racing Competition 2011.
 * It utilizes a Q-learning algorithm for steering control and makes decisions based on sensor inputs.
 */
public class AutomaticTransmissionDriver extends Controller {

    private final QLearning steerControlSystem;
    private final QLearning accelControlSystem;
    private int stuck = 0;
    // current clutch
    private float clutch = 0;
    private double lastSteer = 0;
    private SensorModel lastSensorModel;
    private SensorModel currentSensorModel;
    private SteerControlVariables.States lastSteerState;
    private SteerControlVariables.States currentSteerState;
    private SteerControlVariables.Actions steerActionPerformed;
    private double lastSteerReward;
    private AccelControlVariables.States lastAccelState;
    private AccelControlVariables.States currentAccelState;
    private AccelControlVariables.Actions accelActionPerformed;
    private double lastAccelReward;
    private int epochs;
    private int tics = 0;
    private int laps;
    private long currentTime;
    private double lastDistanceToStart;
    private double currentDistanceToStart;
    private int completeLaps;
    private double distanceRaced;
    private Double[] accel_brake;
    private boolean timeout;
    private boolean outTrack;

    public AutomaticTransmissionDriver() {
        this.steerControlSystem = new QLearning(STEERING_CONTROL_SYSTEM, Constants.RANGE_EPOCHS);
        this.lastSteerState = SteerControlVariables.States.CENTER_AXIS;
        this.currentSteerState = lastSteerState;
        this.steerActionPerformed = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;
        this.lastSteerReward = 0;

        this.accelControlSystem = new QLearning(ACCELERATION_CONTROL_SYSTEM, Constants.RANGE_EPOCHS);
        this.lastAccelState = AccelControlVariables.States.IN_STRAIGHT_LINE;
        this.currentAccelState = lastAccelState;
        this.accelActionPerformed = AccelControlVariables.Actions.PRESS_FULL_THROTTLE;
        this.lastAccelReward = 0;

        this.lastSensorModel = null;
        this.currentSensorModel = null;
        this.epochs = 0;
        this.laps = -1;
        this.completeLaps = 0;
        this.distanceRaced = 0;
        this.currentTime = System.currentTimeMillis();

        this.accel_brake = new Double[2];
        this.accel_brake[0] = 0.0;
        this.accel_brake[1] = 0.0;

        timeout = false;
        outTrack = false;
    }

    public Action control(SensorModel sensorModel) {
        if (tics == 0) {
            this.lastDistanceToStart = sensorModel.getDistanceFromStartLine();
            this.currentDistanceToStart = this.lastDistanceToStart;
        } else {
            this.lastDistanceToStart = this.currentDistanceToStart;
            this.currentDistanceToStart = sensorModel.getDistanceFromStartLine();
        }
        this.tics++;

        if (sensorModel.getCurrentLapTime() > 240.0) {
            timeout = true;
            Action action = new Action();
            action.restartRace = true;
            return action;
        }
//        System.out.println("Laps: " + this.laps + "/1");
//        System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
//        System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
//        System.out.println();

        this.distanceRaced = sensorModel.getDistanceRaced();
        sensorModel.getDistanceFromStartLine();
        if (this.lastDistanceToStart > 1 && this.currentDistanceToStart < 1) {
            this.laps++;
            if (laps >= 1) {
                Action action = new Action();
                action.restartRace = true;
                return action;
            }
        }
        if (Math.abs(sensorModel.getTrackPosition()) >= 1) {
            outTrack = true;
            Action action = new Action();
            action.restartRace = true;
            return action;
        } else {
            // check if car is currently stuck
            if (Math.abs(sensorModel.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
                // update stuck counter
                stuck++;
            } else {
                // if not stuck reset stuck counter
                stuck = 0;
            }

            // after car is stuck for a while apply recovering policy
            if (stuck > DrivingInstructor.stuckTime) {
                /* set gear and sterring command assuming car is
                 * pointing in a direction out of track */

                // to bring car parallel to track axis
                float steer = (float) (-sensorModel.getAngleToTrackAxis() / DrivingInstructor.steerLock);
                int gear = -1; // gear R

                // if car is pointing in the correct direction revert gear and steer
                if (sensorModel.getAngleToTrackAxis() * sensorModel.getTrackPosition() > 0) {
                    gear = 1;
                    steer = -steer;
                }
                clutch = DrivingInstructor.clutching(sensorModel, clutch, getStage());
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
                // compute accel/brake command
                float accel_and_brake = DrivingInstructor.getAccel(sensorModel);
                // compute gear
                int gear = DrivingInstructor.getGear(sensorModel);

                // set accel and brake from the joint accel/brake command
                float accel, brake;
                if (accel_and_brake > 0) {
                    accel = accel_and_brake;
                    brake = 0;
                } else {
                    accel = 0;
                    // apply ABS to brake
                    brake = DrivingInstructor.filterABS(sensorModel, -accel_and_brake);
                }

                clutch = DrivingInstructor.clutching(sensorModel, clutch, getStage());

                // build a CarControl variable and return it
                Action action = new Action();
                long timeTranscurred = (System.currentTimeMillis() - this.currentTime);
//                if (timeTranscurred > 0.0000000000000005) {
                if (timeTranscurred > 500) {
                    this.currentTime = System.currentTimeMillis();
                    this.lastSensorModel = this.currentSensorModel;
                    this.currentSensorModel = sensorModel;
                    /* Update variables for steer control system -------------------------------------------------- */
                    this.currentSteerState = SteerControlVariables.evaluateSteerState(sensorModel);
                    this.steerActionPerformed = (SteerControlVariables.Actions) this.steerControlSystem.nextOnlyBestAction(currentSteerState);
                    this.lastSteer = SteerControlVariables.steerAction2Double(sensorModel, this.steerActionPerformed);
                    action.steering = this.lastSteer;
                    /* Update variables for accel control system -------------------------------------------------- */
                    this.lastAccelState = this.currentAccelState;
                    this.currentAccelState = AccelControlVariables.evaluateAccelState(sensorModel);
                    this.lastAccelReward = AccelControlVariables.calculateReward(this.lastSensorModel, this.currentSensorModel);
                    this.accelActionPerformed = (AccelControlVariables.Actions) this.accelControlSystem.nextOnlyBestAction(this.currentAccelState);
                    if (this.lastSensorModel != null) {
                        System.out.println("State: " + this.currentAccelState.name());
                        System.out.println("Action: " + this.accelActionPerformed.name());
                        double diff = this.currentSensorModel.getSpeed() - this.lastSensorModel.getSpeed();
                        System.out.println("Inc: " + (this.currentSensorModel.getSpeed() > this.lastSensorModel.getSpeed()) + " value: " + diff);
                        System.out.println("accel: " + accel_brake[0]);
                        System.out.println("brake: " + accel_brake[1]);
                        System.out.println();
                    }
                    accel_brake = AccelControlVariables.accelAction2Double(this.currentSensorModel, this.accelActionPerformed);
                    action.accelerate = accel_brake[0];
                    action.brake = accel_brake[1];
                    /* -------------------------------------------------------------------------------------------- */
                } else {
                    /* Update variables for steer control system -------------------------------------------------- */
                    this.currentSteerState = SteerControlVariables.evaluateSteerState(sensorModel);
                    this.steerActionPerformed = (SteerControlVariables.Actions) this.steerControlSystem.nextOnlyBestAction(currentSteerState);
                    this.lastSteer = SteerControlVariables.steerAction2Double(sensorModel, this.steerActionPerformed);
                    action.steering = this.lastSteer;
                    /* Update variables for accel control system -------------------------------------------------- */
                    this.lastAccelState = this.currentAccelState;
                    this.currentAccelState = AccelControlVariables.evaluateAccelState(sensorModel);
                    this.lastAccelReward = AccelControlVariables.calculateReward(this.lastSensorModel, this.currentSensorModel);
                    this.accelActionPerformed = (AccelControlVariables.Actions) this.accelControlSystem.nextOnlyBestAction(this.currentAccelState);
                    accel_brake = AccelControlVariables.accelAction2Double(this.currentSensorModel, this.accelActionPerformed);
                    action.accelerate = accel_brake[0];
                    action.brake = accel_brake[1];
                    /* -------------------------------------------------------------------------------------------- */
                }
                action.gear = gear;
                action.clutch = clutch;
                return action;
            }
        }
    }

    @Override
    public void reset() {
        if (timeout) {
            System.out.println("Time-out");
            timeout = false;
        } else if (outTrack) {
            System.out.println("Out of track");
            outTrack = false;
        } else {
            System.out.println("Complete lap");
            this.completeLaps++;
        }

        String newResults = this.generateStatistics();
        this.accelControlSystem.saveQTableAndStatistics(newResults);
        System.out.println("Restarting the race!");

        this.lastSteerState = SteerControlVariables.States.CENTER_AXIS;
        this.currentSteerState = lastSteerState;
        this.steerActionPerformed = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;
        this.lastSteerReward = 0;

        this.lastAccelState = AccelControlVariables.States.IN_STRAIGHT_LINE;
        this.currentAccelState = lastAccelState;
        this.accelActionPerformed = AccelControlVariables.Actions.PRESS_FULL_THROTTLE;
        this.lastAccelReward = 0;

        this.epochs++;
        this.laps = -1;
        this.tics = 0;
    }

    @Override
    public void shutdown() {
        String newResults = this.generateStatistics();
        this.accelControlSystem.saveQTableAndStatistics(newResults);
        System.out.println("Bye bye!");

        this.epochs++;
    }

    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.epochs + SEPARATOR
                + (int) (this.distanceRaced) + SEPARATOR
                + this.completeLaps + SEPARATOR
                + this.tics + SEPARATOR
                + Constants.MAX_EPOCHS;
    }
}
