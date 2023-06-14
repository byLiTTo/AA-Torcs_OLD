package mdp;

import champ2011client.SensorModel;

public class SteerControlVariables {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static final String SEPARATOR = ",";
    public static final double INITIAL_VALUE = 0.85;
    public static final double LEARNING_RATE = 0.2;
    public static final double DISCOUNT_FACTOR = 0.8;
    public static final int TRAIN_EPOCHS = 250;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Steering constants*/
    static final float steerLock = (float) 0.785398;
    static final float steerSensitivityOffset = (float) 80.0;
    static final float wheelSensitivityCoeff = 1;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static SteerControlVariables.States evaluateSteerState(SensorModel sensorModel) {
        if (sensorModel.getAngleToTrackAxis() < 0) {
            if (sensorModel.getSpeed() > steerSensitivityOffset)
                return States.LEFT_ROAD_AXIS_WHEEL_BLOCKING;
            else
                return States.LEFT_ROAD_AXIS;
        } else {
            if (sensorModel.getSpeed() > steerSensitivityOffset)
                return States.RIGHT_ROAD_AXIS_WHEEL_BLOCKING;
            else
                return States.RIGHT_ROAD_AXIS;
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double steerAction2Double(SensorModel sensorModel, SteerControlVariables.Actions actionPerformed) {

        if (actionPerformed != SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT) {
            // steering angle is compute by correcting the actual car angle w.r.t. to track
            // axis [sensors.getAngle()] and to adjust car position w.r.t to middle of track [sensors.getTrackPos()*0.5]
            float targetAngle = (float) (sensorModel.getAngleToTrackAxis() - sensorModel.getTrackPosition() * 0.5);
            // at high speed reduce the steering command to avoid loosing the control
            if (actionPerformed == Actions.TURN_STEERING_WHEEL_SHARPLY)
                return (float) (targetAngle / (steerLock
                        * (sensorModel.getSpeed() - steerSensitivityOffset)
                        * wheelSensitivityCoeff));
            else
                return (targetAngle) / steerLock;
        }

        return 0.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        KEEP_STEERING_WHEEL_STRAIGHT,
        TURN_STEERING_WHEEL,
        TURN_STEERING_WHEEL_SHARPLY
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        STARTING_GRID,
        LEFT_ROAD_AXIS,
        LEFT_ROAD_AXIS_WHEEL_BLOCKING,
        RIGHT_ROAD_AXIS,
        RIGHT_ROAD_AXIS_WHEEL_BLOCKING
    }

}
