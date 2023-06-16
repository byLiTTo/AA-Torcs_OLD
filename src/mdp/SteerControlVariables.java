package mdp;

import champ2011client.SensorModel;

public class SteerControlVariables {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static final String SEPARATOR = ",";
    public static final double INITIAL_EPSILON = 0.85;
    public static final double LEARNING_RATE = 0.7;
    public static final double DISCOUNT_FACTOR = 0.95;
    public static final String STEER_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/steerQTable.csv";
    public static final String STEER_STATISTICS_TRAIN_PATH = System.getProperty("user.dir") + "/mdp/resources/steerStatisticsTrain.csv";
    public static final String STEER_STATISTICS_TEST_PATH = System.getProperty("user.dir") + "/mdp/resources/steerStatisticsTest.csv";

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Steering constants*/
    static final float steerLock = (float) 0.785398;
    static final float steerSensitivityOffset = (float) 80.0;
    static final float wheelSensitivityCoeff = 1;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static SteerControlVariables.States evaluateSteerState(SensorModel sensorModel) {
        if (sensorModel.getAngleToTrackAxis() < 0) {
//            if (sensorModel.getSpeed() > steerSensitivityOffset)
//                return States.LEFT_ROAD_AXIS_WHEEL_BLOCKING;
//            else
            return States.LEFT_ROAD_AXIS;
        } else {
//            if (sensorModel.getSpeed() > steerSensitivityOffset)
//                return States.RIGHT_ROAD_AXIS_WHEEL_BLOCKING;
//            else
            return States.RIGHT_ROAD_AXIS;
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double steerAction2Double(SensorModel sensorModel, SteerControlVariables.Actions actionPerformed) {
        double steer = 0.0;
        if (actionPerformed != SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT) {
            // steering angle is compute by correcting the actual car angle w.r.t. to track
            // axis [sensors.getAngle()] and to adjust car position w.r.t to middle of track [sensors.getTrackPos()*0.5]
            float targetAngle = (float) (sensorModel.getAngleToTrackAxis() - sensorModel.getTrackPosition() * 0.5);
            // at high speed reduce the steering command to avoid loosing the control
//            if (actionPerformed == Actions.TURN_STEERING_WHEEL_SHARPLY)
            if (sensorModel.getSpeed() > steerSensitivityOffset)
                steer = (float) (targetAngle / (steerLock * (sensorModel.getSpeed() - steerSensitivityOffset) * wheelSensitivityCoeff));
            else steer = (targetAngle) / steerLock;
        }


        if (steer < -1) steer = -1;
        if (steer > 1) steer = 1;
        return steer;
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
    public static double calculateReward(SensorModel lastSensorModel, SensorModel currentSensorModel) {
        double reward = 0.0;
        if (lastSensorModel == null) {
            return reward;
        } else {
            double lastAngle = Math.abs(lastSensorModel.getAngleToTrackAxis());
            double currentAngle = Math.abs(currentSensorModel.getAngleToTrackAxis());
            if (Math.abs(currentSensorModel.getTrackPosition()) >= 1) {
                reward = -1000.0;
            } else {
                if (lastAngle > currentAngle) {
                    reward = 100;
                } else {
                    reward = -10;
                }
            }
        }


//        return (reward - 0) / (Double.MAX_VALUE - 0);
        return reward;

//        if (lastSensorModel == null) {
//            return 0.0;
//        } else {
//            if (Math.abs(currentSensorModel.getTrackPosition()) >= 1) {
//                return -1000;
//            } else {
//                if (lastSensorModel.getAngleToTrackAxis() > currentSensorModel.getAngleToTrackAxis())
//                    return 100;
//                else return -10;
//            }
//        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        KEEP_STEERING_WHEEL_STRAIGHT, TURN_STEERING_WHEEL,
//        TURN_STEERING_WHEEL_SHARPLY
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        STARTING_GRID, LEFT_ROAD_AXIS, //        LEFT_ROAD_AXIS_WHEEL_BLOCKING,
        RIGHT_ROAD_AXIS,
//        RIGHT_ROAD_AXIS_WHEEL_BLOCKING
    }

}
