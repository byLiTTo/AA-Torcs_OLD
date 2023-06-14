package drivers.tester;

import champ2011client.Action;
import champ2011client.Controller;
import champ2011client.SensorModel;
import mdp.QLearning;
import mdp.SteerControlVariables;

public class TurnerDriver extends Controller {

    final double targetSpeed = 50;

    private final QLearning steerControlSystem;
    private SteerControlVariables.States lastState;
    private SteerControlVariables.States currentState;
    private SteerControlVariables.Actions actionPerformed;

    public TurnerDriver() {
        this.steerControlSystem = new QLearning("./steerQTable.csv");
        this.lastState = SteerControlVariables.States.STARTING_GRID;
        this.currentState = lastState;
        this.actionPerformed = SteerControlVariables.Actions.KEEP_STEERING_WHEEL_STRAIGHT;
    }


    public Action control(SensorModel sensorModel) {
        Action action = new Action();
        if (sensorModel.getSpeed() < targetSpeed) {
            action.accelerate = 1;
        }
        this.lastState = currentState;
        this.currentState = SteerControlVariables.evaluateSteerState(sensorModel);
        this.actionPerformed = this.steerControlSystem.nextOnlyBestAction(currentState);
        action.steering = SteerControlVariables.steerAction2Double(sensorModel, this.actionPerformed);

        action.gear = 1;
        return action;
    }

    public void reset() {
        System.out.println("Restarting the race!");

    }

    public void shutdown() {
        this.steerControlSystem.result("./steerQTable.csv");
        System.out.println("Bye bye!");
    }

}
