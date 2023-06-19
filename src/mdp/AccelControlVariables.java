package mdp;

import torcs.SensorModel;

public class AccelControlVariables {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Accel and Brake Constants */
    static final float maxSpeedDist = 170;
    static final float maxSpeed = 250;
    static final float sin5 = (float) 0.08716;
    static final float cos5 = (float) 0.99619;
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* ABS Filter Constants */
    static final float wheelRadius[] = {(float) 0.3179, (float) 0.3179, (float) 0.3276, (float) 0.3276};
    static final float absSlip = (float) 2.0;
    static final float absRange = (float) 3.0;
    static final float absMinSpeed = (float) 3.0;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static AccelControlVariables.States evaluateAccelState(SensorModel sensorModel) {
        if (Math.abs(sensorModel.getTrackPosition()) >= 1)
            return States.OFF_TRACK;
        else {
            float rxSensor = (float) sensorModel.getTrackEdgeSensors()[10];
            // reading of sensor parallel to car axis
            float sensorsensor = (float) sensorModel.getTrackEdgeSensors()[9];
            // reading of sensor at -5 degree w.r.t. car axis
            float sxSensor = (float) sensorModel.getTrackEdgeSensors()[8];

            float targetSpeed;
            if (sensorsensor > maxSpeedDist || (sensorsensor >= rxSensor && sensorsensor >= sxSensor))
                return States.IN_STRAIGHT_LINE;
            else {
                // approaching a turn on right
                if (rxSensor > sxSensor) {
                    // computing approximately the "angle" of turn
                    float h = sensorsensor * sin5;
                    float b = rxSensor - sensorsensor * cos5;
                    float sinAngle = b * b / (h * h + b * b);
                    // estimate the target speed depending on turn and on how close it is
                    targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
                }
                // approaching a turn on left
                else {
                    // computing approximately the "angle" of turn
                    float h = sensorsensor * sin5;
                    float b = sxSensor - sensorsensor * cos5;
                    float sinAngle = b * b / (h * h + b * b);
                    // estimate the target speed depending on turn and on how close it is
                    targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
                }
                targetSpeed = (float) (2 / (1 + Math.exp(sensorModel.getSpeed() - targetSpeed)) - 1);
                if (targetSpeed > 0)
                    return States.IN_CURVE_LOW_SPEED;
                else {
                    targetSpeed = -targetSpeed;
                    // convert speed to m/s
                    float speed = (float) (sensorModel.getSpeed() / 3.6);
                    // when speed lower than min speed for abs do nothing
                    if (speed < absMinSpeed)
                        return States.IN_CURVE_HIGH_SPEED;
                    else {
                        // compute the speed of wheels in m/s
                        float slip = 0.0f;
                        for (int i = 0; i < 4; i++) {
                            slip += sensorModel.getWheelSpinVelocity()[i] * wheelRadius[i];
                        }
                        // slip is the difference between actual speed of car and average speed of wheels
                        slip = speed - slip / 4.0f;
                        // when slip too high apply ABS
                        if (slip > absSlip) {
                            targetSpeed = targetSpeed - (slip - absSlip) / absRange;
                        }
                        // check brake is not negative, otherwise set it to zero
                        if (targetSpeed < 0)
                            return States.IN_CURVE_ENOUGH_SPEED;
                        else
                            return States.IN_CURVE_BLOCKING;
                    }
                }
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static Double[] accelAction2Double(SensorModel sensorModel, AccelControlVariables.Actions actionPerformed) {
        Double[] accel_brake = new Double[2];
        if (actionPerformed != Actions.ACTIVE_LIMITER) {
            float accel_and_brake = getAccel(sensorModel);
            // set accel and brake from the joint accel/brake command
            if (actionPerformed == Actions.PRESS_FULL_THROTTLE || actionPerformed == Actions.PRESS_ACCELERATOR) {
                accel_brake[0] = (double) accel_and_brake;
                accel_brake[1] = 0.0;
            } else {
                accel_brake[0] = 0.0;
                // apply ABS to brake
                accel_brake[1] = (double) filterABS(sensorModel, -accel_and_brake);
            }
        } else {
            accel_brake[0] = 0.3;
            accel_brake[1] = 0.0;
        }
        return accel_brake;
    }

    private static float getAccel(SensorModel sensors) {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor = (float) sensors.getTrackEdgeSensors()[10];
        // reading of sensor parallel to car axis
        float sensorsensor = (float) sensors.getTrackEdgeSensors()[9];
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor = (float) sensors.getTrackEdgeSensors()[8];

        float targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (sensorsensor > maxSpeedDist || (sensorsensor >= rxSensor && sensorsensor >= sxSensor))
            targetSpeed = maxSpeed;
        else {
            // approaching a turn on right
            if (rxSensor > sxSensor) {
                // computing approximately the "angle" of turn
                float h = sensorsensor * sin5;
                float b = rxSensor - sensorsensor * cos5;
                float sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }
            // approaching a turn on left
            else {
                // computing approximately the "angle" of turn
                float h = sensorsensor * sin5;
                float b = sxSensor - sensorsensor * cos5;
                float sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }

        }

        // accel/brake command is exponentially scaled w.r.t. the difference between target speed and current one
        return (float) (2 / (1 + Math.exp(sensors.getSpeed() - targetSpeed)) - 1);

    }

    private static float filterABS(SensorModel sensors, float brake) {
        // convert speed to m/s
        float speed = (float) (sensors.getSpeed() / 3.6);
        // when spedd lower than min speed for abs do nothing
        if (speed < absMinSpeed)
            return brake;

        // compute the speed of wheels in m/s
        float slip = 0.0f;
        for (int i = 0; i < 4; i++) {
            slip += sensors.getWheelSpinVelocity()[i] * wheelRadius[i];
        }
        // slip is the difference between actual speed of car and average speed of wheels
        slip = speed - slip / 4.0f;
        // when slip too high apply ABS
        if (slip > absSlip) {
            brake = brake - (slip - absSlip) / absRange;
        }

        // check brake is not negative, otherwise set it to zero
        if (brake < 0)
            return 0;
        else
            return brake;
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
        if (lastSensorModel == null) {
            return 0.0;
        } else {
            if (Math.abs(lastSensorModel.getTrackPosition()) >= 1) {
                if (lastSensorModel.getSpeed() == currentSensorModel.getSpeed()) return 100.0;
                else return -10.0;
            } else {
                // reading of sensor at +5 degree w.r.t. car axis
                float rxSensor = (float) lastSensorModel.getTrackEdgeSensors()[10];
                // reading of sensor parallel to car axis
                float sensorsensor = (float) lastSensorModel.getTrackEdgeSensors()[9];
                // reading of sensor at -5 degree w.r.t. car axis
                float sxSensor = (float) lastSensorModel.getTrackEdgeSensors()[8];

                float targetSpeed;
                if (sensorsensor > maxSpeedDist || (sensorsensor >= rxSensor && sensorsensor >= sxSensor)) {
                    if (lastSensorModel.getSpeed() < currentSensorModel.getSpeed()) return 100.0;
                    else return -10.0;
                } else {
                    // approaching a turn on right
                    if (rxSensor > sxSensor) {
                        // computing approximately the "angle" of turn
                        float h = sensorsensor * sin5;
                        float b = rxSensor - sensorsensor * cos5;
                        float sinAngle = b * b / (h * h + b * b);
                        // estimate the target speed depending on turn and on how close it is
                        targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
                    }
                    // approaching a turn on left
                    else {
                        // computing approximately the "angle" of turn
                        float h = sensorsensor * sin5;
                        float b = sxSensor - sensorsensor * cos5;
                        float sinAngle = b * b / (h * h + b * b);
                        // estimate the target speed depending on turn and on how close it is
                        targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
                    }
                    targetSpeed = (float) (2 / (1 + Math.exp(lastSensorModel.getSpeed() - targetSpeed)) - 1);
                    if (targetSpeed > 0) {
                        if (lastSensorModel.getSpeed() < currentSensorModel.getSpeed()) return 100.0;
                        else return -10.0;
                    } else {
                        if (lastSensorModel.getSpeed() > currentSensorModel.getSpeed()) return 100.0;
                        else return -10.0;
                    }
                }
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        ACTIVE_LIMITER,
        PRESS_FULL_THROTTLE,
        PRESS_ACCELERATOR,
        PRESS_BRAKE,
        PULL_HAND_BRAKE,
        LET_IT_ROLL
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        OFF_TRACK,
        IN_STRAIGHT_LINE,
        IN_CURVE_LOW_SPEED,
        IN_CURVE_HIGH_SPEED,
        IN_CURVE_ENOUGH_SPEED,
        IN_CURVE_BLOCKING
    }

}
