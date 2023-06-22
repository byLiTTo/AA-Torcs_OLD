package drivers;

import mdp.QLearning;
import mdp.SteerControlVariables;
import torcs.*;

import static mdp.SteerControlVariables.*;
import static torcs.Constants.ControlSystems.STEERING_CONTROL_SYSTEM;
import static torcs.Constants.SEPARATOR;

public class TurnerTrainer extends Controller {
    private final QLearning steerControlSystem;
    private int stuck = 0;
    // current clutch
    private float clutch = 0;
    private double lastSteer = 0;
    private SensorModel lastSensorModel;
    private SensorModel currentSensorModel;
    private States lastState;
    private States currentState;
    private Actions actionPerformed;
    private int epochs;
    private double lastReward;
    private int tics = 0;
    private int laps;
    private long currentTime;
    private double lastDistanceToStart;
    private double currentDistanceToStart;
    private int completeLaps;
    private double distanceRaced;

    public TurnerTrainer() {
        this.steerControlSystem = new QLearning(STEERING_CONTROL_SYSTEM, Constants.RANGE_EPOCHS);
        this.lastState = States.CENTER_AXIS;
        this.currentState = lastState;
        this.actionPerformed = Actions.KEEP_STEERING_WHEEL_STRAIGHT;
        this.lastSensorModel = null;
        this.currentSensorModel = null;
        this.epochs = 0;
        this.laps = -1;
        this.lastReward = 0;
        this.completeLaps = 0;
        this.distanceRaced = 0;
        this.currentTime = System.currentTimeMillis();
    }

    public Action control(SensorModel sensorModel) {
        if (tics == 0) {
            this.lastDistanceToStart = sensorModel.getDistanceFromStartLine();
            this.currentDistanceToStart = this.lastDistanceToStart;
        } else {
            this.lastDistanceToStart = this.currentDistanceToStart;
            this.currentDistanceToStart = sensorModel.getDistanceFromStartLine();
        }
        System.out.println("Laps: " + this.laps + "/1");
        System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
        System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
        System.out.println();
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
            Action action = new Action();
            action.restartRace = true;
            this.tics++;
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
                this.tics++;
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
                if (timeTranscurred > 40) {
                    /* Update variables for steer control system -------------------------------------------------- */
                    this.lastSensorModel = this.currentSensorModel;
                    this.currentSensorModel = sensorModel;
                    this.lastState = this.currentState;
                    this.currentState = evaluateSteerState(sensorModel);
                    this.lastReward = calculateReward(this.lastSensorModel, sensorModel, this.lastSteer);
                    this.actionPerformed = (Actions) (this.steerControlSystem.Update(
                            this.lastState,
                            this.currentState,
                            this.actionPerformed,
                            this.lastReward
                    ));
                    this.lastSteer = steerAction2Double(sensorModel, this.actionPerformed);
                    action.steering = this.lastSteer;
                    /* ------------------------------------------------------------------------------------------- */
                } else {
                    /* Update variables for steer control system -------------------------------------------------- */
                    this.currentState = SteerControlVariables.evaluateSteerState(sensorModel);
                    this.actionPerformed = (SteerControlVariables.Actions) this.steerControlSystem.nextOnlyBestAction(currentState);
                    this.lastSteer = steerAction2Double(sensorModel, this.actionPerformed);
                    action.steering = this.lastSteer;
                    /* ------------------------------------------------------------------------------------------- */
                }
                action.gear = gear;
                action.accelerate = accel;
                action.brake = brake;
                action.clutch = clutch;
                this.tics++;
                return action;
            }
        }
    }

    @Override
    public void reset() {
        this.completeLaps++;

        String newResults = this.generateStatistics();
        this.steerControlSystem.saveQTableAndStatistics(newResults);
        System.out.println("Restarting the race!");

        this.lastState = States.CENTER_AXIS;
        this.currentState = lastState;
        this.actionPerformed = Actions.KEEP_STEERING_WHEEL_STRAIGHT;
        this.epochs++;
        this.lastReward = 0;
        this.laps = -1;
    }

    @Override
    public void shutdown() {
        String newResults = this.generateStatistics();
        this.steerControlSystem.saveQTableAndStatistics(newResults);
        System.out.println("Bye bye!");

        this.epochs++;
    }

    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.epochs + SEPARATOR
                + (int)(this.distanceRaced) + SEPARATOR
                + this.completeLaps + SEPARATOR
                + Constants.MAX_EPOCHS;
    }
}
