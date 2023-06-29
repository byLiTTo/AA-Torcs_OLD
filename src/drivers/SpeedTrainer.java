package drivers;

import mdp.AccelControl;
import mdp.QLearning;
import torcs.*;

import static torcs.Constants.SEPARATOR;

/**
 * A driver implementation for training the speed control using Q-learning.
 */
public class SpeedTrainer extends Controller {
    // QLearning to Steer Control Variables
    private QLearning accelControlSystem;
    private AccelControl.States previousAccelState;
    private AccelControl.States currentAccelState;
    private AccelControl.Actions actionAccel;
    private double accelReward;
    private Double[] accel_and_brake;

    // Time, Laps and Statistics Variables
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private double highSpeed;
    private SensorModel previousSensors;
    private SensorModel currentSensors;

    // Cache variables
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;
    private boolean timeOut;
    private double previousAccel;

    /**
     * Initializes the SpeedTrainer controller.
     */
    public SpeedTrainer() {
        accelControlSystem = new QLearning(Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM, Constants.RANGE_EPOCHS);
        previousAccelState = AccelControl.States.STRAIGHT_LINE;
        currentAccelState = AccelControl.States.STRAIGHT_LINE;
        actionAccel = AccelControl.Actions.FULL_THROTTLE;
        accelReward = 0;
        accel_and_brake = new Double[2];

        tics = 0;
        epochs = 0;
        laps = -1;
        completeLaps = 0;
        distanceRaced = 0;
        highSpeed = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
        timeOut = false;
        previousAccel = 0.0;
    }

    /**
     * The main control method that is called at each time step.
     *
     * @param sensors The sensor data received from the simulator.
     *
     * @return The action to be performed by the car.
     */
    @Override
    public Action control(SensorModel sensors) {
        if (this.tics == 0) {
            this.previosDistanceFromStartLine = sensors.getDistanceFromStartLine();
            this.currentDistanceFromStartLine = this.previosDistanceFromStartLine;

            this.previousSensors = sensors;
            this.currentSensors = this.previousSensors;

            accel_and_brake = AccelControl.accelAction2Double(this.currentSensors, this.actionAccel);

            this.tics++;
        } else {
            this.previosDistanceFromStartLine = this.currentDistanceFromStartLine;
            this.currentDistanceFromStartLine = sensors.getDistanceFromStartLine();

            this.tics++;

            System.out.println("Tics: " + this.tics);
            System.out.println("Laps: " + this.laps + "/1");
            System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
            System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
            System.out.println();
        }

        // Check if time-out
        if (sensors.getCurrentLapTime() > 240.0) {
            this.timeOut = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // Update raced distance
        this.distanceRaced = sensors.getDistanceRaced();

        // Update high speed
        if (sensors.getSpeed() > this.highSpeed) {
            this.highSpeed = sensors.getSpeed();
        }

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
        if (Math.abs(sensors.getTrackPosition()) >= 1) {
            this.offTrack = true;

            this.accelControlSystem.lastUpdate(this.actionAccel, -1000.0);

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // check if car is currently stuck
        if (Math.abs(sensors.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
            this.stuck++;
        } else {
            this.stuck = 0;
        }

        // After car is stuck for a while apply recovering policy
        if (this.stuck > DrivingInstructor.stuckTime) {
            // Set gear and steering command assuming car is pointing in a direction out of track

            // To bring car parallel to track axis
            float steer = (float) (-sensors.getAngleToTrackAxis() / DrivingInstructor.steerLock);
            int gear = -1; // gear R

            // If car is pointing in the correct direction revert gear and steer
            if (sensors.getAngleToTrackAxis() * sensors.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }

            this.clutch = (double) DrivingInstructor.clutching(sensors, (float) this.clutch, getStage());

            // Build a CarControl variable and return it
            Action action = new Action();
            action.gear = gear;
            action.steering = steer;
            action.accelerate = 1.0;
            action.brake = 0;
            action.clutch = clutch;

            return action;
        }


        // If the car is not stuck .....................................................................................
        Action action = new Action();

        // Calculate gear value .......................................................................................
        action.gear = DrivingInstructor.getGear(sensors);

        // Calculate steer value .......................................................................................
        float steer = DrivingInstructor.getSteer(sensors);

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        action.steering = steer;

        // Calculate accel/brake .......................................................................................
        if (this.tics % 5 == 0) {
            this.previousAccel = this.currentSensors.getSpeed() - this.previousSensors.getSpeed();
            this.previousSensors = this.currentSensors;
            this.currentSensors = sensors;
            this.previousAccelState = this.currentAccelState;
            this.currentAccelState = AccelControl.evaluateAccelState(this.currentSensors);
            this.accelReward = AccelControl.calculateReward(
                    this.previousSensors,
                    this.currentSensors,
                    this.previousAccel,
                    (this.currentSensors.getSpeed() - this.previousSensors.getSpeed())
            );
            this.actionAccel = (AccelControl.Actions) this.accelControlSystem.update(
                    this.previousAccelState,
                    this.currentAccelState,
                    this.actionAccel,
                    this.accelReward
            );
            this.accel_and_brake = AccelControl.accelAction2Double(this.currentSensors, this.actionAccel);
            action.accelerate = this.accel_and_brake[0];
            action.brake = this.accel_and_brake[1];
        } else {
            action.accelerate = this.accel_and_brake[0];
            action.brake = this.accel_and_brake[1];
        }

        // Calculate clutch ............................................................................................
        this.clutch = DrivingInstructor.clutching(sensors, (float) this.clutch, getStage());
        action.clutch = this.clutch;

        return action;
    }

    /**
     * Resets the controller to its initial state.
     */
    @Override
    public void reset() {
        previousAccelState = AccelControl.States.STRAIGHT_LINE;
        currentAccelState = AccelControl.States.STRAIGHT_LINE;
        actionAccel = AccelControl.Actions.FULL_THROTTLE;
        accelReward = 0;

        if (this.timeOut) {
            System.out.println("Time out!!!");
        }

        if (this.completeLap) {
            this.completeLaps++;
            System.out.println("Complete lap!");
        }
        if (this.offTrack) {
            System.out.println("Out of track!");
        }

        String newResults = this.generateStatistics();
        this.accelControlSystem.saveQTableAndStatistics(newResults);
        this.accelControlSystem.decreaseEpsilon();

        tics = 0;
        epochs++;
        laps = -1;
        distanceRaced = 0;
        highSpeed = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
        timeOut = false;

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
     * Generates statistics about the race.
     *
     * @return The statistics as a string.
     */
    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.epochs + SEPARATOR
                + this.tics + SEPARATOR
                + (int) (this.distanceRaced) + SEPARATOR
                + (int) (this.highSpeed) + SEPARATOR
                + this.completeLaps + SEPARATOR
                + Constants.MAX_EPOCHS;
    }
}