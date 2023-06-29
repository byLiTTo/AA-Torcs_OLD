package mdp;

import torcs.Constants;
import torcs.SensorModel;

/**
 * The AccelControl class handles the control and evaluation of acceleration in a TORCS racing simulation.
 */
public class AccelControl {

    // Accel Variables
    private static final double maxSpeedDist = 70;
    private static final double sin5 = (float) 0.08716;
    private static final double cos5 = (float) 0.99619;
    private static final double maxSpeed = 150;

    // ABS Variables
    private static final double absMinSpeed = 3.0;
    private static final double[] wheelRadius = {0.3179, 0.3179, 0.3276, 0.3276};
    private static final double absSlip = 2.0;
    private static final double absRange = 3.0;

    /**
     * Evaluates the current acceleration state based on the sensor model.
     *
     * @param current The current sensor model of the car.
     *
     * @return The state of the acceleration as an enum value.
     */
    public static States evaluateAccelState(SensorModel current) {
        double rxSensor = current.getTrackEdgeSensors()[10];
        double sensorsensor = current.getTrackEdgeSensors()[9];
        double sxSensor = current.getTrackEdgeSensors()[8];

        if (sensorsensor > maxSpeedDist || (sensorsensor >= rxSensor && sensorsensor >= sxSensor)) {
            return States.STRAIGHT_LINE;
        } else {
            double targetSpeed;
            if (rxSensor > sxSensor) {
                double h = sensorsensor * sin5;
                double b = rxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            } else {
                double h = sensorsensor * sin5;
                double b = sxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }

            double accel_and_brake = (2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1);

            if (accel_and_brake > 0) {
                return States.IN_CURVE_SHOULD_ACCEL;
            } else {
                double brake = -accel_and_brake;
                double speed = (current.getSpeed() / 3.6);
                double slip = 0.0;
                for (int i = 0; i < 4; i++) {
                    slip += current.getWheelSpinVelocity()[i] * wheelRadius[i];
                }
                slip = speed - slip / 4.0;
                if (slip > absSlip) {
                    brake = brake - (slip - absSlip) / absRange;
                }

                if (brake >= 0) {
                    return States.IN_CURVE_SHOULD_HAND_BRAKE;
                } else {
                    return States.IN_CURVE_SHOULD_ALLOW_ROLL;
                }
            }
        }
    }

    /**
     * Converts the acceleration action to an array of double values based on the current sensor model and action.
     *
     * @param current The current sensor model of the car.
     * @param action  The acceleration control action.
     *
     * @return The array containing the throttle and brake values.
     */
    public static Double[] accelAction2Double(SensorModel current, Actions action) {
        Double[] result = new Double[2];

        double rxSensor = current.getTrackEdgeSensors

                ()[10];
        double sensorsensor = current.getTrackEdgeSensors()[9];
        double sxSensor = current.getTrackEdgeSensors()[8];

        if (action == Actions.FULL_THROTTLE) {
            result[0] = Constants.round((2 / (1 + Math.exp(current.getSpeed() - maxSpeed)) - 1), 8);
            result[1] = 0.0;
            return result;
        } else {
            double targetSpeed;
            if (rxSensor > sxSensor) {
                double h = sensorsensor * sin5;
                double b = rxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            } else {
                double h = sensorsensor * sin5;
                double b = sxSensor - sensorsensor * cos5;
                double sinAngle = b * b / (h * h + b * b);
                targetSpeed = maxSpeed * (sensorsensor * sinAngle / maxSpeedDist);
            }

            double accel_and_brake = (2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1);

            if (action == Actions.ACCELERATE) {
                result[0] = Constants.round(Math.abs((2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1)), 8);
                result[1] = 0.0;
                return result;
            } else {
                double brake = Math.abs((2 / (1 + Math.exp(current.getSpeed() - targetSpeed)) - 1));
                float speed = (float) (current.getSpeed() / 3.6);

                float slip = 0.0f;
                for (int i = 0; i < 4; i++) {
                    slip += current.getWheelSpinVelocity()[i] * wheelRadius[i];
                }
                slip = speed - slip / 4.0f;
                if (slip > absSlip) {
                    brake = brake - (slip - absSlip) / absRange;
                }

                if (action == Actions.HANDBRAKE) {
                    result[0] = 0.0;
                    result[1] = Constants.round(brake, 8);
                    return result;
                } else {
                    result[0] = 0.0;
                    result[1] = 0.0;
                    return result;
                }
            }
        }
    }

    /**
     * Calculates the reward based on the previous and current sensor models, and the previous and current acceleration values.
     *
     * @param previous      The previous sensor model of the car.
     * @param current       The current sensor model of the car.
     * @param previousAccel The previous acceleration value.
     * @param currentAccel  The current acceleration value.
     *
     * @return The reward value.
     */
    public static double calculateReward(SensorModel previous, SensorModel current, double previousAccel, double currentAccel) {
        if (previous == null || current.getCurrentLapTime() <= 0.018) {
            return 0.0;
        } else {
            if (previous.getDistanceRaced() == current.getDistanceRaced()) return -100.0;
            if (Math.abs(previous.getTrackPosition()) < 1 && Math.abs(current.getTrackPosition()) >= 1) return -1000.0;

            States previousState = evaluateAccelState(previous);
            States currentState = evaluateAccelState(current);

            previousAccel = Constants.round(previousAccel, 3);
            currentAccel = Constants.round(currentAccel, 3);

            if (previousState == States.STRAIGHT_LINE) {
                if (currentState == States.STRAIGHT_LINE) {
                    if (previous.getSpeed() < current.getSpeed() && previousAccel < currentAccel) return 100.0;
                    else if ((int) previous.getSpeed() == (maxSpeed - 1)) return 100.0;
                    else return -100.0;
                }
                if (currentState == States.IN_CURVE_SHOULD_ACCEL) {
                    if (previous.getSpeed() < current.getSpeed() && previousAccel < currentAccel) return 100.0;
                    else return -100.0;
                }
                if (currentState == States.IN_CURVE_SHOULD_HAND_BRAKE) {
                    if (previous.getSpeed() > current.getSpeed()) {
                        if (previousAccel > currentAccel) return 200.0;
                        else return 100.0;
                    } else return -100.0;
                }
                if (previous.getSpeed() > current.getSpeed()) return -100.0;
            }

            if (previousState == States.IN_CURVE_SHOULD_ACCEL) {
                if (currentState == States.STRAIGHT_LINE) {
                    if (previous.getSpeed() < current.getSpeed()) {
                        if (previousAccel < currentAccel) return 200.0;
                        else return 100.0;
                    } else return -100.0;
                }
                if (currentState == States.IN_CURVE_SHOULD_ACCEL) {
                    if (previous.getSpeed() < current.getSpeed()) {
                        if (previousAccel < currentAccel) return 200.0;
                        else return 100.0;
                    } else if ((int) previous.getSpeed() == (int) current.getSpeed()) return 200.0;
                    else return -100.0;
                }
                if (currentState == States.IN_CURVE_SHOULD_HAND_BRAKE) {
                    if (previous.getSpeed() < current.getSpeed() && previousAccel > currentAccel) return 100.0;
                    if (previous.getSpeed() > current.getSpeed()) return 100.0;
                    else return -100.0;
                }
                if (previous.getSpeed() > current.getSpeed()) return -100.0;
            }

            if (previousState == States.IN_CURVE_SHOULD_HAND_BRAKE) {
                if (currentState == States.IN_CURVE_SHOULD_ACCEL) {
                    if (previous.getSpeed() > current.getSpeed()) return 100.0;
                    else return -100.0;
                }

                if (currentState == States.IN_CURVE_SHOULD_HAND_BRAKE) {
                    if (previous.getSpeed() > current.getSpeed() && previousAccel > currentAccel) return 200.0;
                    else if (previous.getSpeed() > current.getSpeed() && previousAccel < currentAccel) return 100.0;
                    else return -100.0;
                }

                if (previous.getSpeed() < current.getSpeed()) return -100.0;
            }

            if (previousState == States.IN_CURVE_SHOULD_ALLOW_ROLL) {
                if (currentState == States.IN_CURVE_SHOULD_ACCEL) {
                    if (previous.getSpeed() > current.getSpeed() && previousAccel > currentAccel) return 200.0;
                    if (previous.getSpeed() > current.getSpeed() && previousAccel < currentAccel) return 100.0;
                    else return -100.0;
                }

                if (currentState == States.IN_CURVE_SHOULD_HAND_BRAKE) {
                    if (previous.getSpeed() > current.getSpeed() && previousAccel > currentAccel) return 100.0;
                    else return -100.0;
                }

                if (previous.getSpeed() < current.getSpeed()) return -100.0;
            }
        }
        return -10.0;
    }

    /**
     * Enumeration of possible acceleration control actions.
     */
    public enum Actions {
        FULL_THROTTLE,
        ACCELERATE,
        HANDBRAKE,
        KEEP_ROLLING,
    }

    /**
     * Enumeration of possible acceleration states.
     */
    public enum States {
        STRAIGHT_LINE,
        IN_CURVE_SHOULD_ACCEL,
        IN_CURVE_SHOULD_HAND_BRAKE,
        IN_CURVE_SHOULD_ALLOW_ROLL
    }
}