package mdp;

import torcs.SensorModel;

/**
 * The SteerControl class provides methods for steering control in the TORCS environment.
 * It includes methods for evaluating the steer state, converting steer actions to double values,
 * calculating the reward, and defining the steer actions and states.
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: Mar 4, 2008</p>
 * <p>Time: 3:35:31 PM</p>
 */
public class SteerControl {

    private static final float steerSensitivityOffset = (float) 80.0;
    private static final float steerLock = (float) 0.785398;
    private static final float wheelSensitivityCoeff = 1;

    /**
     * Evaluates the steer state based on the current sensor model.
     *
     * @param current The current sensor model.
     *
     * @return The steer state.
     */
    public static States evaluateSteerState(SensorModel current) {
        if (current.getSpeed() > steerSensitivityOffset) {
            return States.HIGH_SPEED;
        } else {
            return States.NORMAL_SPEED;
        }
    }

    /**
     * Converts a steer action to a double value based on the current sensor model.
     *
     * @param current The current sensor model.
     * @param action  The steer action to convert.
     *
     * @return The double value representing the steer action.
     */
    public static double steerAction2Double(SensorModel current, Actions action) {
        // steering angle is compute by correcting the actual car angle w.r.t. to track
        // axis [sensors.getAngle()] and to adjust car position w.r.t to middle of track [sensors.getTrackPos()*0.5]
        float targetAngle = (float) (current.getAngleToTrackAxis() - current.getTrackPosition() * 0.5);

        switch (action) {
            case TURN_STEERING_WHEEL:
                return (targetAngle) / steerLock;
            case TURN_STEERING_WHEEL_SHARPLY:
                return (targetAngle / (steerLock * (current.getSpeed() - steerSensitivityOffset) * wheelSensitivityCoeff));
        }

        return -1.0;
    }

    /**
     * Calculates the reward based on the previous and current sensor models.
     *
     * @param previous The previous sensor model.
     * @param current  The current sensor model.
     *
     * @return The reward value.
     */
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

    /**
     * The available steer actions.
     */
    public enum Actions {
        TURN_STEERING_WHEEL, TURN_STEERING_WHEEL_SHARPLY
    }

    /**
     * The steer states.
     */
    public enum States {
        NORMAL_SPEED, HIGH_SPEED,
    }
}
