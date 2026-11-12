package org.firstinspires.ftc.teamcode.cydogs.core;

/**
 * Which side of the loading zone the robot parks on.
 * The drive team picks this on the Driver Station during auton INIT
 * using {@link ParkSideSelector}.
 */
public enum ParkSide {
    LEFT("Left"),
    RIGHT("Right"),
    /** Nobody confirmed a side before START. Each auton decides what to do. */
    NONE("None");

    private final String label;

    ParkSide(String label) {
        this.label = label;
    }

    /** Text shown on telemetry. */
    public String label() {
        return label;
    }
}