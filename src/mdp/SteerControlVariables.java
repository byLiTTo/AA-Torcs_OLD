package mdp;

public class SteerControlVariables {

    public static final String SEPARATOR = ",";
    public static final double INITIAL_VALUE = 0.85;
    public static final double LEARNING_RATE = 0.2;
    public static final double DISCOUNT_FACTOR = 0.8;
    public static final int TRAIN_EPOCHS = 250;

    enum Actions {
        AHEAD(0.0);

        private final double steer;

        Actions(double steer) {
            this.steer = steer;
        }

        public double getSteer() {
            return steer;
        }
    }

    enum States {
        MIDDLE
    }

}
