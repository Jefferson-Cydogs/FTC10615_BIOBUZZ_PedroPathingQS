package org.firstinspires.ftc.teamcode.cydogs.core;

/**
 * Where the robot takes its 2nd shot from.
 * The drive team picks this during auton INIT using {@link SecondShotSelector}.
 */
public enum SecondShotSpot {
    NORMAL("Normal"),
    CORNER("Corner");

    private final String label;

    SecondShotSpot(String label) {
        this.label = label;
    }

    /** Text shown on telemetry. */
    public String label() {
        return label;
    }
}