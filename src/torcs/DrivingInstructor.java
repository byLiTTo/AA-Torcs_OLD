package torcs;

public class DrivingInstructor {
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Gear Changing Constants */
    public static final int[] gearUp = {5000, 6000, 6000, 6500, 7000, 0};
    public static final int[] gearDown = {0, 2500, 3000, 3000, 3500, 3500};
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Stuck constants */
    public static final int stuckTime = 25;
    public static final float stuckAngle = (float) 0.523598775; //PI/6
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Accel and Brake Constants */
    public static final float maxSpeedDist = 170;
    public static final float maxSpeed = 250;
    public static final float sin5 = (float) 0.08716;
    public static final float cos5 = (float) 0.99619;
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Steering constants */
    public static final float steerLock = (float) 0.785398;
    public static final float steerSensitivityOffset = (float) 80.0;
    public static final float wheelSensitivityCoeff = 1;
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* ABS Filter Constants */
    public static final float wheelRadius[] = {(float) 0.3179, (float) 0.3179, (float) 0.3276, (float) 0.3276};
    public static final float absSlip = (float) 2.0;
    public static final float absRange = (float) 3.0;
    public static final float absMinSpeed = (float) 3.0;
    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    /* Clutching Constants */
    public static final float clutchMax = (float) 0.5;
    public static final float clutchDelta = (float) 0.05;
    public static final float clutchRange = (float) 0.82;
    public static final float clutchDeltaTime = (float) 0.02;
    public static final float clutchDeltaRaced = 10;
    public static final float clutchDec = (float) 0.01;
    public static final float clutchMaxModifier = (float) 1.3;
    public static final float clutchMaxTime = (float) 1.5;

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

            // check clutch is not bigger than maximum values
            clutch = Math.min(maxClutch, clutch);

            // if clutch is not at max value decrease it quite quickly
            if (clutch != maxClutch) {
                clutch -= delta;
                clutch = Math.max((float) 0.0, clutch);
            }
            // if clutch is at max value decrease it very slowly
            else
                clutch -= clutchDec;
        }
        return clutch;
    }

    public static float getAccel(SensorModel sensors) {
        // checks if car is out of track
        if (sensors.getTrackPosition() < 1 && sensors.getTrackPosition() > -1) {
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
        } else
            return (float) 0.3; // when out of track returns a moderate acceleration command

    }

    public static int getGear(SensorModel sensors) {
        int gear = sensors.getGear();
        double rpm = sensors.getRPM();

        // if gear is 0 (N) or -1 (R) just return 1
        if (gear < 1)
            return 1;
        // check if the RPM value of car is greater than the one suggested
        // to shift up the gear from the current one
        if (gear < 6 && rpm >= gearUp[gear - 1])
            return gear + 1;
        else
            // check if the RPM value of car is lower than the one suggested
            // to shift down the gear from the current one
            if (gear > 1 && rpm <= gearDown[gear - 1])
                return gear - 1;
            else // otherwhise keep current gear
                return gear;
    }

    public static float filterABS(SensorModel sensors, float brake) {
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
        // when slip too high applu ABS
        if (slip > absSlip) {
            brake = brake - (slip - absSlip) / absRange;
        }

        // check brake is not negative, otherwise set it to zero
        if (brake < 0)
            return 0;
        else
            return brake;
    }

    public static float getSteer(SensorModel sensors) {
        // steering angle is compute by correcting the actual car angle w.r.t. to track
        // axis [sensors.getAngle()] and to adjust car position w.r.t to middle of track [sensors.getTrackPos()*0.5]
        float targetAngle = (float) (sensors.getAngleToTrackAxis() - sensors.getTrackPosition() * 0.5);
        // at high speed reduce the steering command to avoid loosing the control
        if (sensors.getSpeed() > steerSensitivityOffset)
            return (float) (targetAngle / (steerLock * (sensors.getSpeed() - steerSensitivityOffset) * wheelSensitivityCoeff));
        else
            return (targetAngle) / steerLock;

    }
}
