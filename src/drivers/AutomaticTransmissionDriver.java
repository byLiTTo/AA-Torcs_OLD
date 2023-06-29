package drivers;

import mdp.AccelControl;
import mdp.QLearning;
import mdp.SteerControl;
import torcs.*;

import static torcs.Constants.SEPARATOR;

/**
 * An automatic transmission driver that uses Q-learning to control steering and acceleration.
 */
public class AutomaticTransmissionDriver extends Controller {

    // QLearning to Steer Control Variables
    private QLearning steerControlSystem;
    private SteerControl.States currentSteerState;
    private SteerControl.Actions actionSteer;

    // QLearning to Accel Control Variables
    private QLearning accelControlSystem;
    private AccelControl.States currentAccelState;
    private AccelControl.Actions actionAccel;

    // Time, Laps and Statistics Variables
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private SensorModel previousSensors;
    private SensorModel currentSensors;

    // Cache variables
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;

    /**
     * Initializes a new instance of the AutomaticTransmissionDriver class.
     */
    public AutomaticTransmissionDriver() {
        steerControlSystem = new QLearning(Constants.ControlSystems.STEERING_CONTROL_SYSTEM);
        currentSteerState = SteerControl.States.NORMAL_SPEED;
        actionSteer = SteerControl.Actions.TURN_STEERING_WHEEL;

        accelControlSystem = new QLearning(Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM);
        currentAccelState = AccelControl.States.STRAIGHT_LINE;
        actionAccel = AccelControl.Actions.FULL_THROTTLE;

        tics = 0;
        epochs = 0;
        laps = -1;
        completeLaps = 0;
        distanceRaced = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
    }

    /**
     * Controls the car based on the sensor inputs.
     *
     * @param sensors the sensor data received from the car
     *
     * @return the action to be performed by the car
     */
    @Override
    public Action control(SensorModel sensors) {
        if (this.tics == 0) {
            this.previosDistanceFromStartLine = sensors.getDistanceFromStartLine();
            this.currentDistanceFromStartLine = this.previosDistanceFromStartLine;

            this.previousSensors = sensors;
            this.currentSensors = this.previousSensors;

            this.tics++;
        } else {
            this.previosDistanceFromStartLine = this.currentDistanceFromStartLine;
            this.currentDistanceFromStartLine = sensors.getDistanceFromStartLine();

            this.previousSensors = this.currentSensors;
            this.currentSensors = sensors;

            this.tics++;

            System.out.println("Laps: " + this.laps + "/1");
            System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
            System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
            System.out.println();
        }

        // Update raced distance
        this.distanceRaced = this.currentSensors.getDistanceRaced();

        // Update complete laps
        if (this.previosDistanceFromStartLine > 1 && this.currentDistanceFromStartLine < 1) {
            this.laps++;

            // Car start back the goal, so ignore first update
            // If the car complete the number of laps, restart the race
            if (this.laps >= 1) {
                this.completeLap = true;

                Action action = new Action();
                action.restartRace = true;
                return action;
            }
        }

        // If the car is off track, restart the race
        if (Math.abs(this.currentSensors.getTrackPosition()) >= 1) {
            this.offTrack = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // check if car is currently stuck
        if (Math.abs(this.currentSensors.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
            this.stuck++;
        } else {
            this.stuck = 0;
        }

        // After car is stuck for a while apply recovering policy
        if (this.stuck > DrivingInstructor.stuckTime) {
            // Set gear and steering command assuming car is pointing in a direction out of track

            // To bring car parallel to track axis
            float steer = (float) (-this.currentSensors.getAngleToTrackAxis() / DrivingInstructor.steerLock);
            int gear = -1; // gear R

            // If car is pointing in the correct direction revert gear and steer
            if (this.currentSensors.getAngleToTrackAxis() * this.currentSensors.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }

            this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());

            // Build a CarControl variable and return it
            Action action = new Action();
            action.gear = gear;
            action.steering = steer;
            action.accelerate = 1.0;
            action.brake = 0;
            action.clutch = clutch;

            return action;
        }

        // If the car is not stuck
        Action action = new Action();

        // Calculate gear value
        action.gear = DrivingInstructor.getGear(this.currentSensors);

        // Calculate steer value
        this.currentSteerState = SteerControl.evaluateSteerState(this.currentSensors);
        this.actionSteer = (SteerControl.Actions) this.steerControlSystem.nextOnlyBestAction(this.currentSteerState);
        double steer = SteerControl.steerAction2Double(this.currentSensors, this.actionSteer);

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        action.steering = steer;

        // Calculate accel/brake
        this.currentAccelState = AccelControl.evaluateAccelState(this.currentSensors);
        this.actionAccel = (AccelControl.Actions) this.accelControlSystem.nextOnlyBestAction(this.currentAccelState);
        Double[] accel_and_brake = AccelControl.accelAction2Double(this.currentSensors, this.actionAccel);
        action.accelerate = accel_and_brake[0];
        action.brake = accel_and_brake[1];

        // Calculate clutch
        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
        action.clutch = this.clutch;

        return action;
    }

    /**
     * Resets the controller's state.
     */
    @Override
    public void reset() {
        currentSteerState = SteerControl.States.NORMAL_SPEED;
        actionSteer = SteerControl.Actions.TURN_STEERING_WHEEL;

        currentAccelState = AccelControl.States.STRAIGHT_LINE;
        actionAccel = AccelControl.Actions.FULL_THROTTLE;

        if (this.completeLap) {
            this.completeLaps++;
            System.out.println("Complete lap!");
        }
        if (this.offTrack) {
            System.out.println("Out of track!");
        }

        String newResults = this.generateStatistics();
        this.accelControlSystem.saveStatistics(newResults);

        tics = 0;
        epochs++;
        laps = -1;
        completeLaps = 0;
        distanceRaced = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;

        System.out.println();
        System.out.println("*** Restarting the race ***");
        System.out.println();
    }

    /**
     * Shuts down the controller.
     */
    @Override
    public void shutdown() {
        System.out.println();
        System.out.println("*** Finish the test ***");
        System.out.println();
    }

    /**
     * Generates a string containing the statistics for the current race.
     *
     * @return the statistics string
     */
    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.epochs + SEPARATOR
                + this.tics + SEPARATOR
                + (int) (this.distanceRaced) + SEPARATOR
                + this.completeLaps + SEPARATOR
                + Constants.MAX_EPOCHS;
    }
}
