package mdp.tmp;

import torcs.SensorModel;

public class AccelControlVariables {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Accel and Brake Constants */
    static final float maxSpeedDist = 70;
    static final float maxSpeed = 150;
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
        // checks if car is out of track
        if (sensorModel.getTrackPosition() < 1 && sensorModel.getTrackPosition() > -1) {
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
        } else {
            return States.OFF_TRACK;
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static Double[] accelAction2Double(SensorModel sensorModel, AccelControlVariables.Actions actionPerformed) {
        Double[] accel_brake = new Double[2];
        if (actionPerformed == Actions.PRESS_FULL_THROTTLE) {
            accel_brake[0] = Math.abs((2 / (1 + Math.exp(sensorModel.getSpeed() - maxSpeed)) - 1));
            accel_brake[1] = 0.0;
            return accel_brake;
        } else if (actionPerformed == Actions.PRESS_ACCELERATOR) {
            // reading of sensor at +5 degree w.r.t. car axis
            float rxSensor = (float) sensorModel.getTrackEdgeSensors()[10];
            // reading of sensor parallel to car axis
            float sensorsensor = (float) sensorModel.getTrackEdgeSensors()[9];
            // reading of sensor at -5 degree w.r.t. car axis
            float sxSensor = (float) sensorModel.getTrackEdgeSensors()[8];

            float targetSpeed;

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
            accel_brake[0] = Math.abs((2 / (1 + Math.exp(sensorModel.getSpeed() - targetSpeed)) - 1));
            accel_brake[1] = 0.0;
            return accel_brake;
        } else if (actionPerformed == Actions.PRESS_BRAKE) {
            // reading of sensor at +5 degree w.r.t. car axis
            float rxSensor = (float) sensorModel.getTrackEdgeSensors()[10];
            // reading of sensor parallel to car axis
            float sensorsensor = (float) sensorModel.getTrackEdgeSensors()[9];
            // reading of sensor at -5 degree w.r.t. car axis
            float sxSensor = (float) sensorModel.getTrackEdgeSensors()[8];

            float targetSpeed;

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
            accel_brake[0] = 0.0;
            accel_brake[1] = Math.abs((2 / (1 + Math.exp(sensorModel.getSpeed() - targetSpeed)) - 1));
            return accel_brake;
        } else if (actionPerformed == Actions.PULL_HAND_BRAKE) {
            // reading of sensor at +5 degree w.r.t. car axis
            float rxSensor = (float) sensorModel.getTrackEdgeSensors()[10];
            // reading of sensor parallel to car axis
            float sensorsensor = (float) sensorModel.getTrackEdgeSensors()[9];
            // reading of sensor at -5 degree w.r.t. car axis
            float sxSensor = (float) sensorModel.getTrackEdgeSensors()[8];

            float targetSpeed;

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

            double brake = Math.abs((2 / (1 + Math.exp(sensorModel.getSpeed() - targetSpeed)) - 1));
            // convert speed to m/s
            float speed = (float) (sensorModel.getSpeed() / 3.6);
            // compute the speed of wheels in m/s
            float slip = 0.0f;
            for (int i = 0; i < 4; i++) {
                slip += sensorModel.getWheelSpinVelocity()[i] * wheelRadius[i];
            }
            // slip is the difference between actual speed of car and average speed of wheels
            slip = speed - slip / 4.0f;
            // when slip too high applu ABS
            if (slip > absSlip) {
                brake = brake - (slip - absSlip) / absRange;
            }
            accel_brake[0] = 0.0;
            accel_brake[1] = brake;
            return accel_brake;
        } else {
            accel_brake[0] = 0.0;
            accel_brake[1] = 0.0;
            return accel_brake;
        }
//        Double[] accel_brake = new Double[2];
//        if (actionPerformed == Actions.ACTIVE_LIMITER) {
//            accel_brake[0] = 0.3;
//            accel_brake[1] = 0.0;
//            return accel_brake;
//        } else
//        if (actionPerformed == Actions.PRESS_FULL_THROTTLE) {
//            accel_brake[0] = (2 / (1 + Math.exp(sensorModel.getSpeed() - maxSpeed)) - 1);
//            accel_brake[1] = 0.0;
//            return accel_brake;
//        } else if (actionPerformed == Actions.PRESS_ACCELERATOR) {
//            accel_brake[0] = 0.3;
//            accel_brake[1] = 0.0;
//            return accel_brake;
//        } else if (actionPerformed == Actions.PRESS_BRAKE) {
//
//            accel_brake[0] = 0.0;
//            accel_brake[1] = 0.6;
//            return accel_brake;
//        } else if (actionPerformed == Actions.PULL_HAND_BRAKE) {
//
//            accel_brake[0] = 0.0;
//            accel_brake[1] = 1.0;
//            return accel_brake;
//        } else {
//            accel_brake[0] = 0.0;
//            accel_brake[1] = 0.0;
//            return accel_brake;
//        }
    }

    public static double calculateReward(SensorModel lastSensorModel, SensorModel currentSensorModel) {
        if (lastSensorModel == null) {
            return 0.0;
        } else {
            if ((lastSensorModel.getTrackPosition() < 1 && lastSensorModel.getTrackPosition() > -1) && (currentSensorModel.getTrackPosition() < 1 && currentSensorModel.getTrackPosition() > -1)) {
                float rxSensor = (float) lastSensorModel.getTrackEdgeSensors()[10];
                // reading of sensor parallel to car axis
                float sensorsensor = (float) lastSensorModel.getTrackEdgeSensors()[9];
                // reading of sensor at -5 degree w.r.t. car axis
                float sxSensor = (float) lastSensorModel.getTrackEdgeSensors()[8];

                float targetSpeed;
                if (sensorsensor > maxSpeedDist || (sensorsensor >= rxSensor && sensorsensor >= sxSensor))
                    return lastSensorModel.getSpeed() < currentSensorModel.getSpeed() ? 100.0 : -100.0;
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
                    targetSpeed = (float) (2 / (1 + Math.exp(lastSensorModel.getSpeed() - targetSpeed)) - 1);
                    if (targetSpeed > 0)
                        return lastSensorModel.getSpeed() < currentSensorModel.getSpeed() ? 100.0 : -100.0;
                    else {
                        double brake = -targetSpeed;
                        // convert speed to m/s
                        float speed = (float) (lastSensorModel.getSpeed() / 3.6);
                        // when speed lower than min speed for abs do nothing
                        if (speed < absMinSpeed)
                            return lastSensorModel.getSpeed() > currentSensorModel.getSpeed() ? 100.0 : -100.0;
                        else {
                            // compute the speed of wheels in m/s
                            float slip = 0.0f;
                            for (int i = 0; i < 4; i++) {
                                slip += lastSensorModel.getWheelSpinVelocity()[i] * wheelRadius[i];
                            }
                            // slip is the difference between actual speed of car and average speed of wheels
                            slip = speed - slip / 4.0f;
                            // when slip too high apply ABS
                            if (slip > absSlip) {
                                brake = brake - (slip - absSlip) / absRange;
                            }
                            // check brake is not negative, otherwise set it to zero
                            if (brake < 0)
                                return lastSensorModel.getSpeed() > currentSensorModel.getSpeed() ? 100.0 : -100.0;
                            else
                                return lastSensorModel.getSpeed() > currentSensorModel.getSpeed() ? 100.0 : -100.0;
                        }
                    }
                }
            } else {
                if (lastSensorModel.getTrackPosition() < 1 && lastSensorModel.getTrackPosition() > -1) return -1000.0;
                else return 0.0;
            }
        }
    }

    public enum Actions {
        //        ACTIVE_LIMITER,
        PRESS_FULL_THROTTLE,
        PRESS_ACCELERATOR,
        PRESS_BRAKE,
        LET_IT_ROLL,
        PULL_HAND_BRAKE
    }

    public enum States {
        OFF_TRACK,
        IN_STRAIGHT_LINE,
        IN_CURVE_LOW_SPEED,
        IN_CURVE_HIGH_SPEED,
        IN_CURVE_ENOUGH_SPEED,
        IN_CURVE_BLOCKING
    }

}
