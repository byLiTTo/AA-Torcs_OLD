package mdp;

public class GearClutch {
    private int gear;
    private float clutch;

    public GearClutch(int gear, float clutch) {
        this.gear = gear;
        this.clutch = clutch;
    }

    public int getGear() {
        return gear;
    }

    public void setGear(int gear) {
        this.gear = gear;
    }

    public float getClutch() {
        return clutch;
    }

    public void setClutch(float clutch) {
        this.clutch = clutch;
    }
}
