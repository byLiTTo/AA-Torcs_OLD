package mdp;

import torcs.SensorModel;

public class SteerControlVariables {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Steering constants*/
    static final float steerLock = (float) 0.785398;
    static final float steerSensitivityOffset = (float) 80.0;
    static final float wheelSensitivityCoeff = 1;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static SteerControlVariables.States evaluateSteerState(SensorModel sensorModel) {
        if (sensorModel.getAngleToTrackAxis() != 0) {
//            if (sensorModel.getSpeed() > steerSensitivityOffset) return States.OFF_CENTER_AXIS_HIGH_SPEED;
//            else
            return States.OFF_CENTER_AXIS;
        } else {
            return States.CENTER_AXIS;
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double steerAction2Double(SensorModel sensorModel, SteerControlVariables.Actions actionPerformed) {
        if (actionPerformed == SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT) {
            return 0.0;
        } else {
            double steer;
            // steering angle is compute by correcting the actual car angle w.r.t. to track
            // axis [sensors.getAngle()] and to adjust car position w.r.t to middle of track [sensors.getTrackPos()*0.5]
            float targetAngle = (float) (sensorModel.getAngleToTrackAxis() - sensorModel.getTrackPosition() * 0.5);
            // at high speed reduce the steering command to avoid loosing the control
//            if (actionPerformed == Actions.TURN_STEERING_WHEEL_SHARPLY) {
            if (sensorModel.getSpeed() > steerSensitivityOffset) {
                steer = (float) (targetAngle / (steerLock * (sensorModel.getSpeed() - steerSensitivityOffset) * wheelSensitivityCoeff));
            } else {
                steer = (targetAngle) / steerLock;
            }

            if (steer < -1) steer = -1;
            if (steer > 1) steer = 1;
            return steer;
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    /**
     * Calculates the reward based on the current and previous sensor readings.
     * The reward is used in reinforcement learning algorithms to provide feedback on the car's performance.
     *
     * @param lastSensorModel    The sensor model representing the previous state of the car.
     * @param currentSensorModel The sensor model representing the current state of the car.
     * @return The calculated reward value.
     */
    public static double calculateReward(SensorModel lastSensorModel, SensorModel currentSensorModel, double steer) {
        if (lastSensorModel == null) {
            return 0.0;
        } else {
            double lastAngle = Math.abs(lastSensorModel.getAngleToTrackAxis());
            double currentAngle = Math.abs(currentSensorModel.getAngleToTrackAxis());
            if (Math.abs(currentSensorModel.getTrackPosition()) >= 1) {
                return -1000.0;
            } else {
                if (steer != 0) {
                    return 100.0;
                } else {
                    return -100.0;
                }
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        KEEP_STEERING_WHEEL_STRAIGHT,
        TURN_STEERING_WHEEL,
//        TURN_STEERING_WHEEL_SHARPLY
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        CENTER_AXIS,
        //        LEFT_ROAD_AXIS,
//        LEFT_ROAD_AXIS_WHEEL_BLOCKING,
//        RIGHT_ROAD_AXIS,
//        RIGHT_ROAD_AXIS_WHEEL_BLOCKING
        OFF_CENTER_AXIS,
//        OFF_CENTER_AXIS_HIGH_SPEED
    }

}
