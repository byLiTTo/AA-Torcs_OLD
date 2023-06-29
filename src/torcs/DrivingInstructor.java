package torcs;

/**
 * The DrivingInstructor class provides static methods for controlling the car's behavior in the TORCS (The Open Racing
 * Car Simulator) environment.
 * It includes methods for clutching, acceleration, gear shifting, ABS filtering, and steering.
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: Mar 4, 2008</p>
 * <p>Time: 3:44:29 PM</p>
 */
public class DrivingInstructor {

    /* Gear Changing Constants */
    public static final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    public static final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};

    /* Stuck constants */
    public static final int stuckTime = 25;
    public static final float stuckAngle = (float) 0.523598775; // PI/6

    /* Accel and Brake Constants */
    public static final float maxSpeedDist = 70;
    public static final float maxSpeed = 150;
    public static final float sin5 = (float) 0.08716;
    public static final float cos5 = (float) 0.99619;

    /* Steering constants */
    public static final float steerLock = (float) 0.785398;
    public static final float steerSensitivityOffset = (float) 80.0;
    public static final float wheelSensitivityCoeff = 1;

    /* ABS Filter Constants */
    public static final float[] wheelRadius = {(float) 0.3179, (float) 0.3179, (float) 0.3276, (float) 0.3276};
    public static final float absSlip = (float) 2.0;
    public static final float absRange = (float) 3.0;
    public static final float absMinSpeed = (float) 3.0;

    /* Clutching Constants */
    public static final float clutchMax = (float) 0.5;
    public static final float clutchDelta = (float) 0.05;
    public static final float clutchRange = (float) 0.82;
    public static final float clutchDeltaTime = (float) 0.02;
    public static final float clutchDeltaRaced = 10;
    public static final float clutchDec = (float) 0.01;
    public static final float clutchMaxModifier = (float) 1.3;
    public static final float clutchMaxTime = (float) 1.5;

    /**
     * Adjusts the clutch value based on the current situation and stage of the race.
     *
     * @param sensors The sensor readings from the car.
     * @param clutch  The current clutch value.
     * @param stage   The current stage of the race.
     *
     * @return The adjusted clutch value.
     */
    public static float clutching(SensorModel sensors, float clutch, Controller.Stage stage) {
        float maxClutch = clutchMax;

        // Check if the current situation is the race start
        if (sensors.getCurrentLapTime() < clutchDeltaTime
                && stage == Controller.Stage.RACE
                && sensors.getDistanceRaced() < clutchDeltaRaced)
            clutch = maxClutch;

        // Adjust the current value of the clutch
        if (clutch > 0) {
            double delta = clutchDelta;
            if (sensors.getGear() < 2) {
                // Apply a stronger clutch output when the gear is one and the race is just started
                delta /= 2;
                maxClutch *= clutchMaxModifier;
                if (sensors.getCurrentLapTime() < clutchMaxTime)
                    clutch = maxClutch;
            }

            // Check if the clutch is not bigger than maximum values
            clutch = Math.min(maxClutch, clutch);

            // If clutch is not at max value, decrease it quite quickly
            if (clutch != maxClutch) {
                clutch -= delta;
                clutch = Math.max((float) 0.0, clutch);
            }
            // If clutch is at max value, decrease it very slowly
            else
                clutch -= clutchDec;
        }
        return clutch;
    }

    /**
     * Calculates the acceleration command based on the current track position and speed of the car.
     *
     * @param sensors The sensor readings from the car.
     *
     * @return The acceleration command value.
     */
    public static float getAccel(SensorModel sensors) {
        // Check if the car is out of the track
        if (sensors.getTrackPosition() < 1 && sensors.getTrackPosition() > -1) {
            // Reading of sensor at +5 degrees w.r.t. car axis
            float rxSensor = (float) sensors.getTrackEdgeSensors()[10];
            // Reading of sensor parallel to car axis
            float sensorSensor = (float) sensors.getTrackEdgeSensors()[9];
            // Reading of sensor at -5 degrees w.r.t. car axis
            float sxSensor = (float) sensors.getTrackEdgeSensors()[8];

            float targetSpeed;

            // If the track is straight and far enough from a turn, go to max speed
            if (sensorSensor > maxSpeedDist || (sensorSensor >= rxSensor && sensorSensor >= sxSensor))
                targetSpeed = maxSpeed;
            else {
                // Approaching a turn on the right
                if (rxSensor > sxSensor) {
                    // Computing approximately the "angle" of turn
                    float h = sensorSensor * sin5;
                    float b = rxSensor - sensorSensor * cos5;
                    float sinAngle = b * b / (h * h + b * b);
                    // Estimate the target speed depending on the turn and how close it is
                    targetSpeed = maxSpeed * (sensorSensor * sinAngle / maxSpeedDist);
                }
                // Approaching a turn on the left
                else {
                    // Computing approximately the "angle" of turn
                    float h = sensorSensor * sin5;
                    float b = sxSensor - sensorSensor * cos5;
                    float sinAngle = b * b / (h * h + b * b);
                    // Estimate the target speed depending on the turn and how close it is
                    targetSpeed = maxSpeed * (sensorSensor * sinAngle / maxSpeedDist);
                }

            }

            // The accel/brake command is exponentially scaled w.r.t. the difference between target speed and current speed
            return (float) (2 / (1 + Math.exp(sensors.getSpeed() - targetSpeed)) - 1);
        } else
            return (float) 0.3; // When out of track, return a moderate acceleration command
    }

    /**
     * Determines the appropriate gear for the car based on the current RPM and gear shifting thresholds.
     *
     * @param sensors The sensor readings from the car.
     *
     * @return The gear value.
     */
    public static int getGear(SensorModel sensors) {
        int gear = sensors.getGear();
        double rpm = sensors.getRPM();

        // If gear is 0 (N) or - 1 (R), just return 1
        if (gear < 1)
            return 1;
        // Check if the RPM value of the car is greater than the one suggested to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return gear + 1;
        else
            // Check if the RPM value of the car is lower than the one suggested to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1])
                return gear - 1;
            else // Otherwise, keep the current gear
                return gear;
    }

    /**
     * Applies ABS (Anti-Lock Braking System) filtering to the brake command based on the current speed and wheel slip.
     *
     * @param sensors The sensor readings from the car.
     * @param brake   The current brake command value.
     *
     * @return The adjusted brake command value.
     */
    public static float filterABS(SensorModel sensors, float brake) {
        // Convert speed to m/s
        float speed = (float) (sensors.getSpeed() / 3.6);
        // When speed is lower than min speed for ABS, do nothing
        if (speed < absMinSpeed)
            return brake;

        // Compute the speed of wheels in m/s
        float slip = 0.0f;
        for (int i = 0; i < 4; i++) {
            slip += sensors.getWheelSpinVelocity()[i] * wheelRadius[i];
        }
        // Slip is the difference between actual speed of the car and average speed of the wheels
        slip = speed - slip / 4.0f;
        // When slip is too high, apply ABS
        if (slip > absSlip) {
            brake = brake - (slip - absSlip) / absRange;
        }

        // Check if brake is not negative, otherwise set it to zero
        if (brake < 0)
            return 0;
        else
            return brake;
    }

    /**
     * Calculates the steering command based on the current track position and car angle.
     *
     * @param sensors The sensor readings from the car.
     *
     * @return The steering command value.
     */
    public static float getSteer(SensorModel sensors) {
        // Steering angle is computed by correcting the actual car angle w.r.t. the track axis and adjusting the car position w.r.t. the middle of the track
        float targetAngle = (float) (sensors.getAngleToTrackAxis() - sensors.getTrackPosition() * 0.5);
        // At high speed, reduce the steering command to avoid losing control
        if (sensors.getSpeed() > steerSensitivityOffset)
            return (float) (targetAngle / (steerLock * (sensors.getSpeed() - steerSensitivityOffset) * wheelSensitivityCoeff));
        else
            return (targetAngle) / steerLock;
    }
}