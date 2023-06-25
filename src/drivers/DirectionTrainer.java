//package drivers;
//
//import mdp.GearControl;
//import mdp.QLearning;
//import mdp.SteerControl;
//import torcs.*;
//
//import static torcs.Constants.SEPARATOR;
//
//public class DirectionTrainer extends Controller {
//    // QLearning to Steer Control Variables   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    private QLearning steerControlSystem;
//    private SteerControl.States previousSteerState;
//    private SteerControl.States currentSteerState;
//    private SteerControl.Actions actionSteer;
//    private double steerReward;
//    // Time, Laps and Statistics Variables   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    private int tics;
//    private int epochs;
//    private int laps;
//    private double previosDistanceFromStartLine;
//    private double currentDistanceFromStartLine;
//    private int completeLaps;
//    private double distanceRaced;
//    private double highSpeed;
//    private SensorModel previousSensors;
//    private SensorModel currentSensors;
//    // Cache variables   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    private int stuck;
//    private double clutch;
//    private boolean completeLap;
//    private boolean offTrack;
//    private boolean timeOut;
//    private long currentTimeMillis;
//
//    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    public DirectionTrainer() {
//        steerControlSystem = new QLearning(Constants.ControlSystems.STEERING_CONTROL_SYSTEM, Constants.RANGE_EPOCHS);
//        previousSteerState = SteerControl.States.NORMAL_SPEED;
//        currentSteerState = SteerControl.States.NORMAL_SPEED;
//        actionSteer = SteerControl.Actions.TURN_STEERING_WHEEL;
//        steerReward = 0;
//
//        tics = 0;
//        epochs = 0;
//        laps = -1;
//        completeLaps = 0;
//        distanceRaced = 0;
//        highSpeed = 0;
//
//        stuck = 0;
//        clutch = 0;
//        completeLap = false;
//        offTrack = false;
//        timeOut = false;
//        currentTimeMillis = System.currentTimeMillis();
//    }
//
//    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    @Override
//    public Action control(SensorModel sensors) {
//        if (this.tics == 0) {
//            this.previosDistanceFromStartLine = sensors.getDistanceFromStartLine();
//            this.currentDistanceFromStartLine = this.previosDistanceFromStartLine;
//
//            this.previousSensors = sensors;
//            this.currentSensors = this.previousSensors;
//
//            this.tics++;
//        } else {
//            this.previosDistanceFromStartLine = this.currentDistanceFromStartLine;
//            this.currentDistanceFromStartLine = sensors.getDistanceFromStartLine();
//
//            this.previousSensors = this.currentSensors;
//            this.currentSensors = sensors;
//
//            this.tics++;
//
//            System.out.println("Tics: " + this.tics);
//            System.out.println("Laps: " + this.laps + "/1");
//            System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
//            System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
//            System.out.println();
//        }
//
//        // Check if time-out
//        if (this.currentSensors.getLastLapTime() > 240.0) {
//            this.timeOut = true;
//
//            Action action = new Action();
//            action.restartRace = true;
//            return action;
//        }
//
//        // Update raced distance
//        this.distanceRaced = this.currentSensors.getDistanceRaced();
//
//        // Update high speed
//        if (this.currentSensors.getSpeed() > this.highSpeed) {
//            this.highSpeed = this.currentSensors.getSpeed();
//        }
//
//        // Update complete laps
//        if (this.previosDistanceFromStartLine > 1 && this.currentDistanceFromStartLine < 1) {
//            this.laps++;
//
//            // Car start back the goal, so ignore first update
//            // If the car complete the number of laps, restart the race
//            if (this.laps >= 1) {
//                this.completeLap = true;
//
//                Action action = new Action();
//                action.restartRace = true;
//                return action;
//            }
//        }
//
//        // If the car is off track, restart the race
//        if (Math.abs(this.currentSensors.getTrackPosition()) >= 1) {
//            this.offTrack = true;
//
//            Action action = new Action();
//            action.restartRace = true;
//            return action;
//        }
//
//        // check if car is currently stuck
//        if (Math.abs(this.currentSensors.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
//            this.stuck++;
//        } else {
//            this.stuck = 0;
//        }
//
//        // After car is stuck for a while apply recovering policy
//        if (this.stuck > DrivingInstructor.stuckTime) {
//            // Set gear and steering command assuming car is pointing in a direction out of track
//
//            // To bring car parallel to track axis
//            float steer = (float) (-this.currentSensors.getAngleToTrackAxis() / DrivingInstructor.steerLock);
//            int gear = -1; // gear R
//
//            // If car is pointing in the correct direction revert gear and steer
//            if (this.currentSensors.getAngleToTrackAxis() * this.currentSensors.getTrackPosition() > 0) {
//                gear = 1;
//                steer = -steer;
//            }
//
//            this.clutch = (double) DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
//
//            // Build a CarControl variable and return it
//            Action action = new Action();
//            action.gear = gear;
//            action.steering = steer;
//            action.accelerate = 1.0;
//            action.brake = 0;
//            action.clutch = clutch;
//
//            return action;
//        }
//
//
//        // If the car is not stuck .....................................................................................
//        Action action = new Action();
//
//        // Calculate gear value ................ .......................................................................
//        action.gear = DrivingInstructor.getGear(this.currentSensors);
//
//        // Calculate steer value .......................................................................................
//        double steer;
//
//        long timeTranscurred = (System.currentTimeMillis() - this.currentTimeMillis);
//        if (timeTranscurred > Constants.GRAPHIC_MODE_TIME) {
//            this.currentTimeMillis = System.currentTimeMillis();
//            this.previousSteerState = this.currentSteerState;
//            this.currentSteerState = SteerControl.evaluateSteerState(this.currentSensors);
//            this.steerReward = SteerControl.calculateReward(this.previousSensors, this.currentSensors);
//            this.actionSteer = (SteerControl.Actions) this.steerControlSystem.Update(
//                    this.previousSteerState,
//                    this.currentSteerState,
//                    this.actionSteer,
//                    this.steerReward
//            );
//            steer = SteerControl.steerAction2Double(this.currentSensors, this.actionSteer);
//        } else {
//            steer = SteerControl.steerAction2Double(this.currentSensors, this.actionSteer);
//        }
//
//        // normalize steering
//        if (steer < -1) steer = -1;
//        if (steer > 1) steer = 1;
//        action.steering = steer;
//
//        // Calculate accel/brake .......................................................................................
//        float accel_and_brake = DrivingInstructor.getAccel(this.currentSensors);
//
//        // Set accel and brake from the joint accel/brake command
//        float accel, brake;
//        if (accel_and_brake > 0) {
//            accel = accel_and_brake;
//            brake = 0;
//        } else {
//            accel = 0;
//            // apply ABS to brake
//            brake = DrivingInstructor.filterABS(this.currentSensors, -accel_and_brake);
//        }
//        action.accelerate = accel;
//        action.brake = brake;
//
//        // Calculate clutch ............................................................................................
//        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
//        action.clutch = this.clutch;
//
//        return action;
//    }
//
//    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    @Override
//    public void reset() {
//        previousSteerState = SteerControl.States.NORMAL_SPEED;
//        currentSteerState = SteerControl.States.NORMAL_SPEED;
//        actionSteer = SteerControl.Actions.TURN_STEERING_WHEEL;
//        steerReward = 0;
//
//        if (this.timeOut) {
//            System.out.println("Time out!!!");
//        }
//
//        if (this.completeLap) {
//            this.completeLaps++;
//            System.out.println("Complete lap!");
//        }
//        if (this.offTrack) {
//            System.out.println("Out of track!");
//        }
//
//        String newResults = this.generateStatistics();
//        this.steerControlSystem.saveQTableAndStatistics(newResults);
//        this.steerControlSystem.decreaseEpsilon();
//
//        tics = 0;
//        epochs++;
//        laps = -1;
//        distanceRaced = 0;
//        highSpeed = 0;
//
//        stuck = 0;
//        clutch = 0;
//        completeLap = false;
//        offTrack = false;
//        timeOut = false;
//
//        System.out.println();
//        System.out.println("*** Restarting the race ***");
//        System.out.println();
//    }
//
//    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    @Override
//    public void shutdown() {
//        System.out.println();
//        System.out.println("*** Finish the test ***");
//        System.out.println();
//    }
//
//    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
//    private String generateStatistics() {
//        return getTrackName() + SEPARATOR + this.epochs + SEPARATOR + this.tics + SEPARATOR + (int) (this.distanceRaced) + SEPARATOR + (int) (this.highSpeed) + SEPARATOR + this.completeLaps + SEPARATOR + Constants.MAX_EPOCHS;
//    }
//}
