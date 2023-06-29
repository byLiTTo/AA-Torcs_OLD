package drivers;

import torcs.Action;
import torcs.Controller;
import torcs.SensorModel;

/**
 * This is a simple solo controller that maintains a target speed and adjusts the steering to stay on the track.
 */
public class DeadSimpleSoloController extends Controller {

    private final double targetSpeed = 55;

    /**
     * Controls the car based on the sensor input.
     *
     * @param sensorModel the sensor input received from the car
     *
     * @return the action to be taken by the car
     */
    public Action control(SensorModel sensorModel) {
        Action action = new Action();
        if (sensorModel.getSpeed() < targetSpeed) {
            action.accelerate = 1;
        }
        if (sensorModel.getAngleToTrackAxis() < 0) {
            action.steering = -0.1;
        } else {
            action.steering = 0.1;
        }
        action.gear = 1;
        return action;
    }

    /**
     * Resets the state of the controller.
     */
    public void reset() {
        System.out.println("Restarting the race!");
    }

    /**
     * Shuts down the controller.
     */
    public void shutdown() {
        System.out.println("Bye bye!");
    }
}
