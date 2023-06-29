package drivers;

import mdp.AccelControl;
import mdp.GearControl;
import mdp.QLearning;
import mdp.SteerControl;
import torcs.*;

import static torcs.Constants.SEPARATOR;

/**
 * A manual transmission driver that uses Q-learning to control the gear, steering, and acceleration of the car.
 */
public class ManualTransmissionDriver extends Controller {

    private final QLearning gearControlSystem;
    private final QLearning steerControlSystem;
    private final QLearning accelControlSystem;
    private GearControl.States currentGearState;
    private GearControl.Actions actionGear;
    private SteerControl.States currentSteerState;
    private SteerControl.Actions actionSteer;
    private AccelControl.States currentAccelState;
    private AccelControl.Actions actionAccel;
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private SensorModel previousSensors;
    private SensorModel currentSensors;
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;

    /**
     * Initializes a new instance of the ManualTransmissionDriver class.
     */
    public ManualTransmissionDriver() {
        gearControlSystem = new QLearning(Constants.ControlSystems.GEAR_CONTROL_SYSTEM);
        currentGearState = GearControl.States.NEUTRAL_REVERSE;
        actionGear = GearControl.Actions.ACTIVE_LIMITER;

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
     * @param sensors the sensor data
     *
     * @return the action to perform
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

        this.distanceRaced = this.currentSensors.getDistanceRaced();

        if (this.previosDistanceFromStartLine > 1 && this.currentDistanceFromStartLine < 1) {
            this.laps++;

            if (this.laps >= 1) {
                this.completeLap = true;

                Action action = new Action();
                action.restartRace = true;
                return action;
            }
        }

        if (Math.abs(this.currentSensors.getTrackPosition()) >= 1) {
            this.offTrack = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        if (Math.abs(this.currentSensors.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
            this.stuck++;
        } else {
            this.stuck = 0;
        }

        if (this.stuck > DrivingInstructor.stuckTime) {
            float steer = (float) (-this.currentSensors.getAngleToTrackAxis() / DrivingInstructor.steerLock);
            int gear = -1;

            if (this.currentSensors.getAngleToTrackAxis() * this.currentSensors.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }

            this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());

            Action action = new Action();
            action.gear = gear;
            action.steering = steer;
            action.accelerate = 1.0;
            action.brake = 0;
            action.clutch = clutch;

            return action;
        }

        Action action = new Action();

        this.currentGearState = GearControl.evaluateGearState(this.currentSensors);
        this.actionGear = (GearControl.Actions) this.gearControlSystem.nextOnlyBestAction(this.currentGearState);
        action.gear = GearControl.gearAction2Double(this.currentSensors, this.actionGear);

        this.currentSteerState = SteerControl.evaluateSteerState(this.currentSensors);
        this.actionSteer = (SteerControl.Actions) this.steerControlSystem.nextOnlyBestAction(this.currentSteerState);
        double steer = SteerControl.steerAction2Double(this.currentSensors, this.actionSteer);

        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        action.steering = steer;

        this.currentAccelState = AccelControl.evaluateAccelState(this.currentSensors);
        this.actionAccel = (AccelControl.Actions) this.accelControlSystem.nextOnlyBestAction(this.currentAccelState);
        Double[] accel_and_brake = AccelControl.accelAction2Double(this.currentSensors, this.actionAccel);
        action.accelerate = accel_and_brake[0];
        action.brake = accel_and_brake[1];

        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
        action.clutch = this.clutch;

        return action;
    }

    /**
     * Resets the driver to its initial state.
     */
    @Override
    public void reset() {
        currentGearState = GearControl.States.NEUTRAL_REVERSE;
        actionGear = GearControl.Actions.ACTIVE_LIMITER;

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
        this.gearControlSystem.saveStatistics(newResults);

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
     * Performs any necessary cleanup operations before shutting down the driver.
     */
    @Override
    public void shutdown() {
        System.out.println();
        System.out.println("*** Finish the test ***");
        System.out.println();
    }

    /**
     * Generates statistics about the driver's performance.
     *
     * @return the statistics as a string
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
