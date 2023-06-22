package torcs;

public class Constants {
    public static final String SEPARATOR = ",";
    public static final double INITIAL_EPSILON = 1.0;
    public static final double LEARNING_RATE = 0.2;
    public static final double DISCOUNT_FACTOR = 0.5;
    public static final int MAX_EPOCHS = 2000;
    public static final int RANGE_EPOCHS = 1000;
    public static final String STEER_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Steer.csv";
    public static final String ACCEL_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Accel.csv";
    public static final String CLUTCH_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Clutch.csv";
    public static final String STATISTICS_TRAIN_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTrain.csv";
    public static final String STATISTICS_TEST_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTest.csv";
    public static final double TRACK_LENGTH = 3185.83;

    public enum ControlSystems {
        STEERING_CONTROL_SYSTEM, ACCELERATION_CONTROL_SYSTEM, CLUTCH_CONTROL_SYSTEM
    }
}
