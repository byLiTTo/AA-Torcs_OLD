package torcs;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Constants {
    public static final String SEPARATOR = ",";
    public static final double INITIAL_EPSILON = 1.0;
    public static final double LEARNING_RATE = 0.2;
    public static final double DISCOUNT_FACTOR = 0.5;
    public static final int MAX_EPOCHS = 10;
    public static final int RANGE_EPOCHS = 5;
    public static final int GRAPHIC_MODE_TIME = 50;
    public static final double CONSOLE_MODE_TIME = 0.0000001;
    public static final String STEER_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Steer.csv";
    public static final String ACCEL_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Accel.csv";
    public static final String GEAR_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Gear.csv";
    public static final String STATISTICS_TRAIN_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTrain.csv";
    public static final String STATISTICS_TEST_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTest.csv";

    public static double round(double number, int dec) {
        BigDecimal bd = new BigDecimal(number);
        bd = bd.setScale(dec, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    public enum ControlSystems {
        STEERING_CONTROL_SYSTEM, ACCELERATION_CONTROL_SYSTEM, GEAR_CONTROL_SYSTEM
    }
}
