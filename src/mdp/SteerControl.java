package mdp;

import torcs.SensorModel;

public class SteerControl {

    private static final float steerSensitivityOffset = (float) 80.0;
    private static final float steerLock = (float) 0.785398;
    private static final float wheelSensitivityCoeff = 1;


    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static States evaluateSteerState(SensorModel current) {
        if (current.getSpeed() > steerSensitivityOffset) {
            return States.HIGH_SPEED;
        } else {
            return States.NORMAL_SPEED;
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double steerAction2Double(SensorModel current, Actions action) {

        // steering angle is compute by correcting the actual car angle w.r.t. to track
        // axis [sensors.getAngle()] and to adjust car position w.r.t to middle of track [sensors.getTrackPos()*0.5]
        float targetAngle = (float) (current.getAngleToTrackAxis() - current.getTrackPosition() * 0.5);

        switch (action) {
            case TURN_STEERING_WHEEL -> {
                return (targetAngle) / steerLock;
            }
            case TURN_STEERING_WHEEL_SHARPLY -> {
                return (float) (targetAngle / (steerLock * (current.getSpeed() - steerSensitivityOffset)
                        * wheelSensitivityCoeff));
            }
        }

        return -1.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double calculateReward(SensorModel previous, SensorModel current) {
        return 0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        TURN_STEERING_WHEEL, TURN_STEERING_WHEEL_SHARPLY
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        NORMAL_SPEED, HIGH_SPEED,
    }
}
