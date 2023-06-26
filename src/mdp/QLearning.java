package mdp;

import torcs.Constants;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

import static torcs.Constants.*;

public class QLearning {

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    private final HashMap<String, HashMap<String, Double>> qTable;
    private List<Object> possibleActions = null;
    private Object lastState;

    private double epsilon;
    private double epsilonDecay;
    private int maxEpochs;
    private int epochs;
    private Random random;
    private ControlSystems system;
    private String qTablePath;

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public QLearning(ControlSystems system) {
        this.qTable = new HashMap<>();
        this.epsilon = INITIAL_EPSILON;
        this.epochs = 0;

        this.random = new Random(System.currentTimeMillis());

        this.system = system;
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                this.possibleActions = Arrays.asList(SteerControl.Actions.values());
                this.qTablePath = STEER_Q_TABLE_PATH;
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                this.possibleActions = Arrays.asList(AccelControl.Actions.values());
                this.qTablePath = ACCEL_Q_TABLE_PATH;
            }
            case GEAR_CONTROL_SYSTEM -> {
                this.possibleActions = Arrays.asList(GearControl.Actions.values());
                this.qTablePath = GEAR_Q_TABLE_PATH;
            }
        }
        File f = new File(this.qTablePath);
        if (!f.exists()) this.createQTable();
        else this.loadQTable();
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->

    public QLearning(ControlSystems system, int maxEpochs) {
        this.qTable = new HashMap<>();
        this.epsilon = INITIAL_EPSILON;
        this.maxEpochs = maxEpochs;
        this.epsilonDecay = INITIAL_EPSILON / this.maxEpochs;
        this.epochs = 0;

        this.random = new Random(System.currentTimeMillis());
        this.system = system;
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                this.possibleActions = Arrays.asList(SteerControl.Actions.values());
                this.qTablePath = STEER_Q_TABLE_PATH;
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                this.possibleActions = Arrays.asList(AccelControl.Actions.values());
                this.qTablePath = ACCEL_Q_TABLE_PATH;
            }
            case GEAR_CONTROL_SYSTEM -> {
                this.possibleActions = Arrays.asList(GearControl.Actions.values());
                this.qTablePath = GEAR_Q_TABLE_PATH;
            }
        }
        File f = new File(this.qTablePath);
        if (!f.exists()) this.createQTable();
        else this.loadQTable();
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void createQTable() {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                for (SteerControl.States state : SteerControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        SteerControl.Actions action = (SteerControl.Actions) (tmp);
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                for (AccelControl.States state : AccelControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        AccelControl.Actions action = (AccelControl.Actions) (tmp);
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
            }
            case GEAR_CONTROL_SYSTEM -> {
                for (GearControl.States state : GearControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        GearControl.Actions action = (GearControl.Actions) (tmp);
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void loadQTable() {
        qTable.clear();
        try (Scanner file = new Scanner(new File(this.qTablePath))) {
            String[] rowLabels = file.nextLine().split(SEPARATOR);

            while (file.hasNextLine()) {
                String[] row = file.nextLine().split(SEPARATOR);
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
    public void saveTable() {
        try (PrintWriter file = new PrintWriter((this.qTablePath))) {
            file.write(" Q-TABLE ");
            file.write(SEPARATOR);
            switch (this.system) {
                case STEERING_CONTROL_SYSTEM -> {
                    for (Object tmp : this.possibleActions) {
                        SteerControl.Actions action = (SteerControl.Actions) (tmp);
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (SteerControl.States state : SteerControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            SteerControl.Actions action = (SteerControl.Actions) (tmp);
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                }
                case ACCELERATION_CONTROL_SYSTEM -> {
                    for (Object tmp : this.possibleActions) {
                        AccelControl.Actions action = (AccelControl.Actions) (tmp);
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (AccelControl.States state : AccelControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            AccelControl.Actions action = (AccelControl.Actions) (tmp);
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                }
                case GEAR_CONTROL_SYSTEM -> {
                    for (Object tmp : this.possibleActions) {
                        GearControl.Actions action = (GearControl.Actions) (tmp);
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (GearControl.States state : GearControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            GearControl.Actions action = (GearControl.Actions) (tmp);
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                }
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save tableQ in .csv file...");
            e.printStackTrace();
        }
    }

    public Object Update(Object lastState, Object currentState, Object actionPerformed, double reward) {
        this.lastState = lastState;
        if (lastState != null) {
            double newQValue = this.getQValue(lastState, actionPerformed) + LEARNING_RATE * (reward + DISCOUNT_FACTOR
                    * this.getMaxQValue(lastState));
            this.setQValue(lastState, actionPerformed, (Constants.round(newQValue, 8) / 10));
        }
        return nextAction(currentState);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public void lastUpdate(Object lastAction, double reward) {
        if (this.lastState != null) {
            double newQValue = (1 - LEARNING_RATE) * this.getQValue(this.lastState, lastAction) + LEARNING_RATE
                    * (reward + DISCOUNT_FACTOR * this.getMaxQValue(this.lastState));
            this.setQValue(this.lastState, lastAction, (Constants.round(newQValue, 8) / 10));
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private double getQValue(Object stateO, Object actionO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                SteerControl.States state = (SteerControl.States) (stateO);
                SteerControl.Actions action = (SteerControl.Actions) (actionO);
                return this.qTable.get(state.name()).get(action.name());
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                AccelControl.States state = (AccelControl.States) (stateO);
                AccelControl.Actions action = (AccelControl.Actions) (actionO);
                return this.qTable.get(state.name()).get(action.name());
            }
            case GEAR_CONTROL_SYSTEM -> {
                GearControl.States state = (GearControl.States) (stateO);
                GearControl.Actions action = (GearControl.Actions) (actionO);
                return this.qTable.get(state.name()).get(action.name());
            }
        }
        return 0.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private void setQValue(Object stateO, Object actionO, double value) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                SteerControl.States state = (SteerControl.States) (stateO);
                SteerControl.Actions action = (SteerControl.Actions) (actionO);
                HashMap<String, Double> row = this.qTable.get(state.name());
                row.replace(action.name(), value);
                this.qTable.replace(state.name(), row);
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                AccelControl.States state = (AccelControl.States) (stateO);
                AccelControl.Actions action = (AccelControl.Actions) (actionO);
                HashMap<String, Double> row = this.qTable.get(state.name());
                row.replace(action.name(), value);
                this.qTable.replace(state.name(), row);
            }
            case GEAR_CONTROL_SYSTEM -> {
                GearControl.States state = (GearControl.States) (stateO);
                GearControl.Actions action = (GearControl.Actions) (actionO);
                HashMap<String, Double> row = this.qTable.get(state.name());
                row.replace(action.name(), value);
                this.qTable.replace(state.name(), row);
            }
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private double getMaxQValue(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                SteerControl.States state = (SteerControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<SteerControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) (tmp);
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
                SteerControl.Actions selected = candidates.get(index);
                return values.get(selected.name());
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                AccelControl.States state = (AccelControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<AccelControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) (tmp);
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
                AccelControl.Actions selected = candidates.get(index);
                return values.get(selected.name());
            }
            case GEAR_CONTROL_SYSTEM -> {
                GearControl.States state = (GearControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<GearControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) (tmp);
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
                GearControl.Actions selected = candidates.get(index);
                return values.get(selected.name());
            }
        }
        return 0.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public Object nextAction(Object state) {
        double probability = random.nextDouble();
        if (probability < epsilon) {
            return this.getRandomAction();
        } else {
            return this.getBestAction(state);
        }
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private Object getRandomAction() {
        int index = random.nextInt(this.possibleActions.size());
        return this.possibleActions.get(index);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    private Object getBestAction(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                SteerControl.States state = (SteerControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<SteerControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) (tmp);
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
            case ACCELERATION_CONTROL_SYSTEM -> {
                AccelControl.States state = (AccelControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<AccelControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) (tmp);
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
            case GEAR_CONTROL_SYSTEM -> {
                GearControl.States state = (GearControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<GearControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) (tmp);
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
        }
        return 0.0;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public Object nextOnlyBestAction(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM -> {
                SteerControl.States state = (SteerControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                Object theBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) (tmp);
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        theBest = action;
                    }
                }
                return theBest;
            }
            case ACCELERATION_CONTROL_SYSTEM -> {
                AccelControl.States state = (AccelControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                Object theBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) (tmp);
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        theBest = action;
                    }
                }
                return theBest;
            }
            case GEAR_CONTROL_SYSTEM -> {
                GearControl.States state = (GearControl.States) (stateO);
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                Object theBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) (tmp);
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        theBest = action;
                    }
                }
                return theBest;
            }
        }
        return null;
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public void saveStatistics(String newResults) {
        this.saveStatistics(STATISTICS_TEST_PATH, newResults);
    }

    //   --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> --> -->
    public void saveQTableAndStatistics(String newResults) {
        this.epochs++;
        this.saveTable();
        this.saveStatistics(STATISTICS_TRAIN_PATH, newResults);
    }

    private void saveStatistics(String filePath, String newResults) {
        List<String> content = new ArrayList<>();
        try (Scanner file = new Scanner(new File(filePath))) {
            while (file.hasNextLine()) {
                content.add(file.nextLine());
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not load statistics from .csv file...");
            e.printStackTrace();
        }
        try (PrintWriter file = new PrintWriter((filePath))) {
            for (String line : content) {
                file.write(line + "\n");
            }
            file.write(newResults + "\n");
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save statistics in .csv file...");
            e.printStackTrace();
        }
    }

    public void decreaseEpsilon() {
        this.epsilon -= this.epsilonDecay;
    }
}
