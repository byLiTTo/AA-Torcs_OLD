package drivers;

import mdp.GearControl;
import mdp.QLearning;
import torcs.*;

import static torcs.Constants.SEPARATOR;

/**
 * This class represents a Gear Trainer driver that uses Q-learning to control the gear of a car in TORCS.
 * It extends the {@link Controller} class.
 */
public class GearTrainer extends Controller {
    private final QLearning gearControlSystem;
    private GearControl.States previousGearState;
    private GearControl.States currentGearState;
    private GearControl.Actions actionGear;
    private double gearReward;
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
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;
    private boolean timeOut;

    /**
     * Initializes a new instance of the GearTrainer class.
     * It creates a QLearning object for gear control and sets initial states and actions.
     */
    public GearTrainer() {
        gearControlSystem = new QLearning(Constants.ControlSystems.GEAR_CONTROL_SYSTEM, Constants.RANGE_EPOCHS);
        previousGearState = GearControl.States.NEUTRAL_REVERSE;
        currentGearState = GearControl.States.NEUTRAL_REVERSE;
        actionGear = GearControl.Actions.ACTIVE_LIMITER;
        gearReward = 0;

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
    }

    /**
     * Controls the car's actions based on the sensor data received.
     *
     * @param sensors the sensor data received from the car
     *
     * @return an Action object representing the car's actions
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

            System.out.println("Tics: " + this.tics);
            System.out.println("Laps: " + this.laps + "/1");
            System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
            System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
            System.out.println();
        }

        // Check if time-out
        if (this.currentSensors.getLastLapTime() > 240.0) {
            this.timeOut = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // Update raced distance
        this.distanceRaced = this.currentSensors.getDistanceRaced();

        // Update high speed
        if (this.currentSensors.getSpeed

                () > this.highSpeed) {
            this.highSpeed = this.currentSensors.getSpeed();
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

            this.clutch = (double) DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());

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

        // Calculate gear value ........................................................................................
        if (this.tics % 5 == 0) {
            this.previousGearState = this.currentGearState;
            this.currentGearState = GearControl.evaluateGearState(this.currentSensors);
            this.gearReward = GearControl.calculateReward(this.previousSensors, this.currentSensors);
            this.actionGear = (GearControl.Actions) this.gearControlSystem.update(
                    this.previousGearState,
                    this.currentGearState,
                    this.actionGear,
                    this.gearReward
            );
            action.gear = GearControl.gearAction2Double(this.currentSensors, this.actionGear);
        } else {
            action.gear = GearControl.gearAction2Double(this.currentSensors, GearControl.Actions.KEEP_GEAR);
        }

        // Calculate steer value ........................................................................................
        float steer = DrivingInstructor.getSteer(this.currentSensors);

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        action.steering = steer;

        // Calculate accel/brake ........................................................................................
        float accel_and_brake = DrivingInstructor.getAccel(this.currentSensors);

        // Set accel and brake from the joint accel/brake command
        float accel, brake;
        if (accel_and_brake > 0) {
            accel = accel_and_brake;
            brake = 0;
        } else {
            accel = 0;
            // apply ABS to brake
            brake = DrivingInstructor.filterABS(this.currentSensors, -accel_and_brake);
        }
        action.accelerate = accel;
        action.brake = brake;

        // Calculate clutch ............................................................................................
        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
        action.clutch = this.clutch;

        return action;
    }

    /**
     * Resets the driver's state and variables.
     */
    @Override
    public void reset() {
        previousGearState = GearControl.States.NEUTRAL_REVERSE;
        currentGearState = GearControl.States.NEUTRAL_REVERSE;
        actionGear = GearControl.Actions.ACTIVE_LIMITER;
        gearReward = 0;

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
        this.gearControlSystem.saveQTableAndStatistics(newResults);
        this.gearControlSystem.decreaseEpsilon();

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
     * Performs shutdown operations when the race is finished.
     */
    @Override
    public void shutdown() {
        System.out.println();
        System.out.println("*** Finish the test ***");
        System.out.println();
    }

    /**
     * Generates statistics about the race and returns them as a formatted string.
     *
     * @return a string containing the statistics of the race
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