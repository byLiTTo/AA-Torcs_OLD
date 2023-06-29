package torcs;

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * The Constants class provides constant values used in the TORCS (The Open Racing Car Simulator) environment.
 *
 * <p>Created by IntelliJ IDEA.</p>
 * <p>User: Administrator</p>
 * <p>Date: N/A</p>
 * <p>Time: N/A</p>
 */
public class Constants {

    public static final String SEPARATOR = ",";
    public static final double INITIAL_EPSILON = 1.0;
    public static final double LEARNING_RATE = 0.2;
    public static final double DISCOUNT_FACTOR = 0.85;
    public static final int MAX_EPOCHS = 250;
    public static final int RANGE_EPOCHS = 100;
    public static final String STEER_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Steer.csv";
    public static final String ACCEL_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Accel.csv";
    public static final String GEAR_Q_TABLE_PATH = System.getProperty("user.dir") + "/mdp/resources/QTable_Gear.csv";
    public static final String STATISTICS_TRAIN_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTrain.csv";
    public static final String STATISTICS_TEST_PATH = System.getProperty("user.dir") + "/mdp/resources/StatisticsTest.csv";

    /**
     * Rounds a number to the specified number of decimal places.
     *
     * @param number The number to be rounded.
     * @param dec    The number of decimal places.
     *
     * @return The rounded number.
     */
    public static double round(double number, int dec) {
        BigDecimal bd = new BigDecimal(number);
        bd = bd.setScale(dec, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    /**
     * The ControlSystems enum represents the different control systems in TORCS.
     */
    public enum ControlSystems {
        STEERING_CONTROL_SYSTEM, ACCELERATION_CONTROL_SYSTEM, GEAR_CONTROL_SYSTEM
    }
}