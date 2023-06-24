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
                return (targetAngle / (steerLock * (current.getSpeed() - steerSensitivityOffset)
                        * wheelSensitivityCoeff));
            }
        }

        return -1.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double calculateReward(SensorModel previous, SensorModel current) {
        double reward = 0.0;

        // Reward for staying on the track
        if (Math.abs(current.getTrackPosition()) <= 1.0) {
            reward += 10.0;
        } else {
            reward -= 10.0;
        }

        // Reward for maintaining a good speed
        if (current.getSpeed() > 50.0) {
            reward += 10.0;
        } else {
            reward -= 10.0;
        }

        // Reward for good track angle (the vehicle is aligned with the track axis)
        if (Math.abs(current.getAngleToTrackAxis()) < 0.1) {
            reward += 10.0;
        } else {
            reward -= 100.0;
        }

        // Reward for maintaining a stable acceleration
        double acceleration = current.getSpeed() - previous.getSpeed();
        if (Math.abs(acceleration) < 5.0) {
            reward += 10.0;
        } else {
            reward -= 10.0;
        }

        return reward;
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
