package torcs;

public class Constants {
    public static final String SEPARATOR = ",";
    public static final double INITIAL_EPSILON = 0.85;
    public static final double LEARNING_RATE = 0.7;
    public static final double DISCOUNT_FACTOR = 0.95;

    public static final String STEER_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Steer.csv";
    public static final String ACCEL_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Accel.csv";
    public static final String CLUTCH_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Clutch.csv";
    public static final String STATISTICS_TRAIN_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTrain.csv";
    public static final String STATISTICS_TEST_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTest.csv";

    public enum ControlSystems {
        STEERING_CONTROL_SYSTEM, ACCELERATION_CONTROL_SYSTEM, CLUTCH_CONTROL_SYSTEM
    }
}
