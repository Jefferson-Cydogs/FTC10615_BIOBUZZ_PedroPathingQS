package org.firstinspires.ftc.teamcode.cydogs.core;

import androidx.annotation.NonNull;


public enum TargetColor {
    //ARTIFACTGREEN(110, 190), //Joe's empirical values
    ARTIFACTGREEN(140, 170), //Francisco's empirical values
    //GREEN(90, 150), //Standard Color Sensor values
    //ARTIFACTPURPLE(200, 270), //Joe's empirical values
    ARTIFACTPURPLE(180, 240), //Francisco's empirical values
    //PURPLE(225, 350), //Standard Color Sensor values
    RED(0, 30, 350, 360), //Standard Color Sensor values. RED wraps around the hue circle
    BLUE(150, 225); //Standard Color Sensor values

    private final float low1, high1;
    private final float low2, high2;

    // Constructor for 1 hue range
    TargetColor(float low1, float high1) {
        this(low1, high1, -1, -1);
    }

    // Constructor for 2 hue ranges (only applies to RED)
    TargetColor(float low1, float high1, float low2, float high2)
    {
        this.low1 = low1;
        this.high1 = high1;
        this.low2 = low2;
        this.high2 = high2;
    }

    public boolean matches(float hue)
    {
        boolean inPrimaryRange = (low1 <= hue) && (hue <= high1);
        boolean inSecondaryRange = (0 <= low2) && ((low2 <= hue) && (hue <= high2)); // Only check if second range exists (different from -1)

        return inPrimaryRange || inSecondaryRange;
    }

    @Override
    @NonNull
    public String toString()
    {
        return name().charAt(0) + name().substring(1).toLowerCase(); // "Red", "Blue", etc.
    }

}