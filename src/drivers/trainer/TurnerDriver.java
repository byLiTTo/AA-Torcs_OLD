package drivers.trainer;

import champ2011client.Action;
import champ2011client.Controller;
import champ2011client.SensorModel;
import mdp.SteerControlVariables;

public class TurnerDriver extends Controller {
    final double targetSpeed = 15;


    private SteerControlVariables.States lastState;
    private SteerControlVariables.States currentState;
    private SteerControlVariables.Actions actionPerformed;

    public TurnerDriver() {
//        this.steerControlSystem = new QLearning("./steerQTable.csv");
//        this.lastState = SteerControlVariables.States.STARTING_GRID;
//        this.currentState = lastState;
//        this.actionPerformed = SteerControlVariables.Actions.AHEAD;
    }


    public Action control(SensorModel sensorModel) {
        Action action = new Action();
        if (sensorModel.getSpeed() < targetSpeed) {
            action.accelerate = 1;
        }
        action.gear = 1;
        return action;
    }

    public void reset() {
        System.out.println("Restarting the race!");

    }

    public void shutdown() {
        System.out.println("Bye bye!");
    }
}
