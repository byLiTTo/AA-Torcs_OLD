package drivers;

import mdp.AccelControl;
import mdp.QLearning;
import torcs.*;

import static torcs.Constants.SEPARATOR;

public class SpeedDriver extends Controller {
    // QLearning to Accel Control Variables  --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private QLearning accelControlSystem;
    private AccelControl.States previousAccelState;
    private AccelControl.States currentAccelState;
    private AccelControl.Actions actionAccel;
    private double accelReward;
    private Double[] accel_and_brake;
    // Time, Laps and Statistics Variables   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private SensorModel previousSensors;
    private SensorModel currentSensors;
    private SensorModel previousAux;
    private SensorModel currentAux;
    // Cache variables   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;
    private double accel;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public SpeedDriver() {
        accelControlSystem = new QLearning(Constants.ControlSystems.ACCELERATION_CONTROL_SYSTEM);
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

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
        accel = 0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    @Override
    public Action control(SensorModel sensors) {
        if (this.tics == 0) {
            this.previosDistanceFromStartLine = sensors.getDistanceFromStartLine();
            this.currentDistanceFromStartLine = this.previosDistanceFromStartLine;

            this.previousSensors = sensors;
            this.currentSensors = this.previousSensors;

            this.previousAux = sensors;
            this.currentAux = this.previousAux;

            accel_and_brake = AccelControl.accelAction2Double(this.currentSensors, this.actionAccel);

            this.tics++;
        } else {
            this.previosDistanceFromStartLine = this.currentDistanceFromStartLine;
            this.currentDistanceFromStartLine = sensors.getDistanceFromStartLine();

            this.previousSensors = this.currentSensors;
            this.currentSensors = sensors;

            this.tics++;

            System.out.println("Tics: " + this.tics);
//            System.out.println("Laps: " + this.laps + "/1");
//            System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
//            System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
//            System.out.println();
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

        // If the car is not stuck .....................................................................................
        Action action = new Action();

        // Calculate gear value ........................................................................................
        action.gear = DrivingInstructor.getGear(this.currentSensors);

        // Calculate steer value .......................................................................................
        float steer = DrivingInstructor.getSteer(this.currentSensors);

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        action.steering = steer;

        // Calculate accel/brake .......................................................................................
        if (this.tics % 5 == 0) {
            this.accel = Constants.round(this.currentAux.getSpeed() - this.previousAux.getSpeed(), 3);
            this.previousAux = this.currentAux;
            this.currentAux = sensors;
            this.previousAccelState = this.currentAccelState;
            this.currentAccelState = AccelControl.evaluateAccelState(this.currentAux);
            AccelControl.Actions aux = this.actionAccel;
            this.actionAccel = (AccelControl.Actions) this.accelControlSystem.nextOnlyBestAction(this.currentAccelState);
            this.accel_and_brake = AccelControl.accelAction2Double(this.currentAux, this.actionAccel);
            action.accelerate = accel_and_brake[0];
            action.brake = accel_and_brake[1];

            System.out.println("Time: " + this.currentSensors.getCurrentLapTime());
            System.out.println("State P: " + this.previousAccelState);
            System.out.println("State C: " + this.currentAccelState);
            System.out.println("Action: " + aux);
            System.out.println("vel P: " + this.previousAux.getSpeed());
            System.out.println("vel C: " + this.currentAux.getSpeed());
            System.out.println("vel int P: " + (int) this.previousAux.getSpeed());
            System.out.println("vel int C: " + (int) this.currentAux.getSpeed());
            System.out.println("accel P: " + this.accel);
            System.out.println("accel C: " + Constants.round(this.currentAux.getSpeed() - this.previousAux.getSpeed(), 3));
            System.out.println("Distance P: " + previousAux.getDistanceRaced());
            System.out.println("Distance C: " + currentAux.getDistanceRaced());
            System.out.println();
        } else {
            action.accelerate = accel_and_brake[0];
            action.brake = accel_and_brake[1];
        }

        // Calculate clutch ............................................................................................
        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
        action.clutch = this.clutch;

        return action;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    @Override
    public void reset() {
        previousAccelState = AccelControl.States.STRAIGHT_LINE;
        currentAccelState = AccelControl.States.STRAIGHT_LINE;
        actionAccel = AccelControl.Actions.FULL_THROTTLE;
        accelReward = 0;

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

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    @Override
    public void shutdown() {
        System.out.println();
        System.out.println("*** Finish the test ***");
        System.out.println();
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.epochs + SEPARATOR
                + this.tics + SEPARATOR
                + (int) (this.distanceRaced) + SEPARATOR
                + this.completeLaps + SEPARATOR
                + Constants.MAX_EPOCHS;
    }
}
