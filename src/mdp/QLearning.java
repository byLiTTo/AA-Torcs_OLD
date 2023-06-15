package mdp;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import mdp.SteerControlVariables.Actions;
import mdp.SteerControlVariables.States;

public class QLearning {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    private final HashMap<String, HashMap<String, Double>> qTable;
    private final List<Actions> possibleActions;
    private States lastState;

    private double epsilon;
    private double epsilonDecay;
    private int epochs;
    private Random random;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public QLearning(String filePath) {
        this.qTable = new HashMap<>();
        this.possibleActions = Arrays.asList(Actions.values());

        this.epsilon = SteerControlVariables.INITIAL_VALUE;
        this.epochs = 0;

        this.random = new Random(System.currentTimeMillis());

        File f = new File(filePath);
        if (!f.exists()) this.createQTable();
        else this.loadQTable(filePath);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void createQTable() {
        for (States state : States.values()) {
            HashMap<String, Double> row = new HashMap<>();
            for (Actions action : this.possibleActions) {
                row.put(action.name(), 0.0);
            }
            qTable.put(state.name(), row);
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void loadQTable(String filePath) {
        qTable.clear();
        try (Scanner file = new Scanner(new File(filePath))) {
            String[] rowLabels = file.nextLine().split(SteerControlVariables.SEPARATOR);

            while (file.hasNextLine()) {
                String[] row = file.nextLine().split(SteerControlVariables.SEPARATOR);
                String state = row[0];
                HashMap<String, Double> rowHash = new HashMap<>();
                for (int i = 1; i < row.length; i++) {
                    rowHash.put(rowLabels[i], Double.parseDouble(row[i]));
                }
                this.qTable.put(state, rowHash);

            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not load tablaQ from .csv file...");
            e.printStackTrace();
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void saveTable(String filePath) {
        try (PrintWriter file = new PrintWriter((filePath))) {
            file.write(" Q-TABLE ");
            file.write(SteerControlVariables.SEPARATOR);
            for (Actions action : this.possibleActions) {
                file.write(action.name());
                file.write(SteerControlVariables.SEPARATOR);
            }
            file.write("\n");

            for (States state : States.values()) {
                file.write(state.name());
                file.write(SteerControlVariables.SEPARATOR);
                for (Actions action : possibleActions) {
                    String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                    file.write(value);
                    file.write(SteerControlVariables.SEPARATOR);
                }
                file.write("\n");
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save tableQ in .csv file...");
            e.printStackTrace();
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public Actions Update(States lastState, States currentState, Actions actionPerformed, double reward) {
        this.lastState = lastState;
        if (lastState != null) {
            double newQValue = this.getQValue(lastState, actionPerformed) + SteerControlVariables.LEARNING_RATE *
                    (reward + SteerControlVariables.DISCOUNT_FACTOR * this.getMaxQValue(lastState));
            this.setQValue(lastState, actionPerformed, newQValue);
        }
        return nextAction(currentState);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public void lastUpdate(Actions lastAction, double reward) {
        if (this.lastState != null) {
            double newQValue = (1 - SteerControlVariables.LEARNING_RATE) * this.getQValue(this.lastState, lastAction)
                    + SteerControlVariables.LEARNING_RATE
                    * (reward + SteerControlVariables.DISCOUNT_FACTOR * this.getMaxQValue(this.lastState));
            this.setQValue(this.lastState, lastAction, newQValue);
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private double getQValue(States state, Actions action) {
        return this.qTable.get(state.name()).get(action.name());
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void setQValue(States state, Actions action, double value) {
        HashMap<String, Double> row = this.qTable.get(state.name());
        row.replace(action.name(), value);
        this.qTable.replace(state.name(), row);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private double getMaxQValue(States state) {
        double maxValue = -Double.MAX_VALUE;
        HashMap<String, Double> values = qTable.get(state.name());
        ArrayList<Actions> candidates = new ArrayList<>();
        for (Actions action : this.possibleActions) {
            double value = values.get(action.name());
            if (maxValue < value) {
                maxValue = value;
                candidates.clear();
                candidates.add(action);
            } else if (maxValue == value) {
                candidates.add(action);
            }
        }
        int index = random.nextInt(candidates.size());
        Actions selected = candidates.get(index);
        return values.get(selected.name());
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public Actions nextAction(States state) {
        double probability = random.nextDouble();
        if (probability < epsilon) {
            return this.getRandomAction();
        } else {
            return this.getBestAction(state);
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private Actions getRandomAction() {
        int index = random.nextInt(this.possibleActions.size());
        return this.possibleActions.get(index);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private Actions getBestAction(States state) {
        double maxValue = -Double.MAX_VALUE;
        HashMap<String, Double> values = qTable.get(state.name());
        ArrayList<Actions> candidates = new ArrayList<>();
        for (Actions action : this.possibleActions) {
            double value = values.get(action.name());
            if (maxValue < value) {
                maxValue = value;
                candidates.clear();
                candidates.add(action);
            } else if (maxValue == value) {
                candidates.add(action);
            }
        }
        int index = random.nextInt(candidates.size());
        return candidates.get(index);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public Actions nextOnlyBestAction(States state) {
        double maxValue = -Double.MAX_VALUE;
        HashMap<String, Double> values = qTable.get(state.name());
        Actions theBest = this.getRandomAction();
        for (Actions action : this.possibleActions) {
            double value = values.get(action.name());
            if (maxValue < value) {
                maxValue = value;
                theBest = action;
            }
        }
        return theBest;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public void result(String filePath) {
        this.epsilon = (SteerControlVariables.INITIAL_VALUE * 250)
                / (250 + (this.epochs * 4.7));
        this.saveTable(filePath);
    }

}
