package mdp;

import torcs.Constants;
import torcs.SensorModel;

public class AccelControl {

    // Accel Variables   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private static final double maxSpeedDist = 70;
    private static final double sin5 = (float) 0.08716;
    private static final double cos5 = (float) 0.99619;
    private static final double maxSpeed = 150;
    // ABS Variables --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private static final float absMinSpeed = (float) 3.0;
    private static final float wheelRadius[] = {(float) 0.3179, (float) 0.3179, (float) 0.3276, (float) 0.3276};
    private static final float absSlip = (float) 2.0;
    private static final float absRange = (float) 3.0;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static States evaluateAccelState(SensorModel current) {
        // reading of sensor at +5 degree w.r.t. car axis
        double rxSensor = current.getTrackEdgeSensors()[10];
        // reading of sensor parallel to car axis
        double sensorsensor = current.getTrackEdgeSensors()[9];
        // reading of sensor at -5 degree w.r.t. car axis
        double sxSensor = current.getTrackEdgeSensors()[8];

        // track is straight and enough far from a turn so goes to max speed
        if (sensorsensor > maxSpeedDist || (sensorsensor >= rxSensor && sensorsensor >= sxSensor)) {
            return States.STRAIGHT_LINE;
        } else {
            double targetSpeed;
            // approaching a turn on right
            if (rxSensor > sxSensor) {
                // computing approximately the "angle" of turn
                double h = sensorsensor * sin5;
                double b = rxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }
            // approaching a turn on left
            else {
                // computing approximately the "angle" of turn
                double h = sensorsensor * sin5;
                double b = sxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }

            // accel/brake command is exponentially scaled w.r.t. the difference between target speed and current one
            double accel_and_brake = (2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1);

            if (accel_and_brake > 0) {
                return States.IN_CURVE_SHOULD_ACCEL;
            } else {
                double brake = -accel_and_brake;

                // convert speed to m/s
                float speed = (float) (current.getSpeed() / 3.6);

                // when speed lower than min speed for abs do nothing
                if (speed < absMinSpeed) {
                    return States.IN_CURVE_SHOULD_BRAKE;
                } else {
                    // compute the speed of wheels in m/s
                    float slip = 0.0f;
                    for (int i = 0; i < 4; i++) {
                        slip += current.getWheelSpinVelocity()[i] * wheelRadius[i];
                    }
                    // slip is the difference between actual speed of car and average speed of wheels
                    slip = speed - slip / 4.0f;
                    // when slip too high applu ABS
                    if (slip > absSlip) {
                        brake = brake - (slip - absSlip) / absRange;
                    }

                    // check brake is not negative, otherwise set it to zero
                    if (brake >= 0) return States.IN_CURVE_SHOULD_HAND_BRAKE;
                    else return States.IN_CURVE_SHOULD_ALLOW_ROLL;
                }
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static Double[] accelAction2Double(SensorModel current, Actions action) {
        Double[] result = new Double[2];

        // reading of sensor at +5 degree w.r.t. car axis
        double rxSensor = current.getTrackEdgeSensors()[10];
        // reading of sensor parallel to car axis
        double sensorsensor = current.getTrackEdgeSensors()[9];
        // reading of sensor at -5 degree w.r.t. car axis
        double sxSensor = current.getTrackEdgeSensors()[8];

        // track is straight and enough far from a turn so goes to max speed
        if (action == Actions.FULL_THROTTLE) {
            result[0] = Constants.round((2 / (1 + Math.exp(current.getSpeed() - maxSpeed)) - 1), 8);
            result[1] = 0.0;
            return result;
        } else {
            double targetSpeed;
            // approaching a turn on right
            if (rxSensor > sxSensor) {
                // computing approximately the "angle" of turn
                double h = sensorsensor * sin5;
                double b = rxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }
            // approaching a turn on left
            else {
                // computing approximately the "angle" of turn
                double h = sensorsensor * sin5;
                double b = sxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }

            // accel/brake command is exponentially scaled w.r.t. the difference between target speed and current one
            double accel_and_brake = (2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1);

            if (action == Actions.ACCELERATE) {
                result[0] = Constants.round(Math.abs((2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1)), 8);
                result[1] = 0.0;
                return result;
            } else {
                double brake = Math.abs((2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1));

                // convert speed to m/s
                float speed = (float) (current.getSpeed() / 3.6);

                // when speed lower than min speed for abs do nothing
                if (action == Actions.BRAKE) {
                    result[0] = 0.0;
                    result[1] = Constants.round(brake, 8);
                    return result;
                } else {
                    // compute the speed of wheels in m/s
                    float slip = 0.0f;
                    for (int i = 0; i < 4; i++) {
                        slip += current.getWheelSpinVelocity()[i] * wheelRadius[i];
                    }
                    // slip is the difference between actual speed of car and average speed of wheels
                    slip = speed - slip / 4.0f;
                    // when slip too high applu ABS
                    if (slip > absSlip) {
                        brake = brake - (slip - absSlip) / absRange;
                    }

                    // check brake is not negative, otherwise set it to zero
                    if (action == Actions.HANDBRAKE) {
                        result[0] = 0.0;
                        result[1] = Constants.round(brake, 8);
                        return result;
                    } else {
                        // if action == Actions.KEEP_ROLLING
                        result[0] = 0.0;
                        result[1] = 0.0;
                        return result;
                    }
                }
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public static double calculateReward(SensorModel previous, SensorModel current) {
        if (previous == null) {
            return 0.0;
        } else {
            if (Math.abs(previous.getTrackPosition()) < 1 && Math.abs(current.getTrackPosition()) >= 1) {
                return -1000.0;
            } else {
                States previousState = evaluateAccelState(previous);
                States currentState = evaluateAccelState(current);

                switch (previousState) {

                    case STRAIGHT_LINE -> {
                        switch (currentState) {

                            case STRAIGHT_LINE -> {
                                if (previous.getSpeed() < current.getSpeed() || current.getSpeed() >= 149) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ACCEL -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_HAND_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ALLOW_ROLL -> {
                                if ((int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                        }
                    }
                    case IN_CURVE_SHOULD_ACCEL -> {
                        switch (currentState) {

                            case STRAIGHT_LINE -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ACCEL -> {
                                if (previous.getSpeed() < current.getSpeed()
                                        || (int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_HAND_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ALLOW_ROLL -> {
                                if ((int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                        }
                    }
                    case IN_CURVE_SHOULD_BRAKE -> {
                        switch (currentState) {

                            case STRAIGHT_LINE -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ACCEL -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()
                                        || (int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_HAND_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ALLOW_ROLL -> {
                                if ((int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                        }
                    }
                    case IN_CURVE_SHOULD_HAND_BRAKE -> {
                        switch (currentState) {

                            case STRAIGHT_LINE -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ACCEL -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_HAND_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ALLOW_ROLL -> {
                                if ((int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                        }
                    }
                    case IN_CURVE_SHOULD_ALLOW_ROLL -> {
                        switch (currentState) {

                            case STRAIGHT_LINE -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ACCEL -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_BRAKE -> {
                                if (previous.getSpeed() > current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_HAND_BRAKE -> {
                                if (previous.getSpeed() < current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                            case IN_CURVE_SHOULD_ALLOW_ROLL -> {
                                if ((int) previous.getSpeed() == (int) current.getSpeed()) return 100.0;
                                else return -100.0;
                            }
                        }
                    }
                }
            }
        }
        return 0.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum Actions {
        FULL_THROTTLE,
        ACCELERATE,
        BRAKE,
        HANDBRAKE,
        KEEP_ROLLING,
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public enum States {
        STRAIGHT_LINE,
        IN_CURVE_SHOULD_ACCEL,
        IN_CURVE_SHOULD_BRAKE,
        IN_CURVE_SHOULD_HAND_BRAKE,
        IN_CURVE_SHOULD_ALLOW_ROLL
    }
}
