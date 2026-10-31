package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.cydogs.core.TargetColor;


public class BaseColorSensor {
    private static final double DISTANCE_THRESHOLD_CM = 5.5;
    private static final int REQUIRED_MATCHES = 2;

    // ReadArtifactColor takes this many samples, and all of them must agree
    private static final int READ_SAMPLES = 2;

    private OpMode opMode;
    private NormalizedColorSensor colorSensor;

    public BaseColorSensor(OpMode opMode, String ColorDistanceSensor)
    {
        this.opMode = opMode;
        HardwareMap hw = opMode.hardwareMap;
        this.colorSensor = hw.get(NormalizedColorSensor.class, ColorDistanceSensor);

        // Adjust the gain, as the sensor may default to an unpredictable value, leading to inconsistent color readings.
        // Recommendation based on FTC documentation: 10.0 - 20.0 for bright lighting or LEDs, 20.0 - 40.0 for normal indoor lighting.
        // 30.0 is a safe default for FTC
        colorSensor.setGain(30);
    }

    /**
     * Fast read: returns the color of the artifact in front of the sensor, or null if there is
     * none (nothing within range, or the color is not one of the TargetColors).
     * Costs ONE read when nothing is close. When something is close it takes READ_SAMPLES
     * samples (distance + color each) and they must all agree, or it returns null.
     */
    public TargetColor ReadArtifactColor()
    {
        TargetColor found = null;

        for (int i = 0; i < READ_SAMPLES; i++) {
            double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            if (distance > DISTANCE_THRESHOLD_CM) {
                return null; // nothing close
            }

            float hue = JavaUtil.colorToHue(colorSensor.getNormalizedColors().toColor());

            TargetColor seen = null;
            for (TargetColor color : TargetColor.values()) {
                if (color.matches(hue)) {
                    seen = color;
                    break;
                }
            }

            if (seen == null) {
                return null; // something is there but it is not a known color
            }
            if (i == 0) {
                found = seen;
            } else if (seen != found) {
                return null; // samples disagree, do not trust this reading
            }
        }

        return found;
    }

    public boolean SeeArtifactColor(TargetColor targetColor)
    {
        int matchCount = 0;

        for (int i = 0; i < 3; i++) {
            double Distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            if (Distance > DISTANCE_THRESHOLD_CM) {
                continue; // Too far, skip this reading
            }

            NormalizedRGBA NormalizedColors = colorSensor.getNormalizedColors();
            int RawColor = NormalizedColors.toColor();
            float Hue = JavaUtil.colorToHue(RawColor);

            if (targetColor.matches(Hue)) {
                matchCount++;
            }
        }

        return (matchCount >= REQUIRED_MATCHES);
    }

    public void WhatDoISee()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        int r = (int) (colors.red * 255);
        int g = (int) (colors.green * 255);
        int b = (int) (colors.blue * 255);
        Color.RGBToHSV(r, g, b, hsv);

        opMode.telemetry.addData("Normalized Red", "%.3f", colors.red);
        opMode.telemetry.addData("Normalized Green", "%.3f", colors.green);
        opMode.telemetry.addData("Normalized Blue", "%.3f", colors.blue);
        opMode.telemetry.addData("Normalized Alpha", "%.3f", colors.alpha);
        opMode.telemetry.addData("Distance:",((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("RGB", "(%d, %d, %d)", r, g, b);
        opMode.telemetry.addData("HSV", "H: %.1f  S: %.3f  V: %.3f", hsv[0], hsv[1], hsv[2]);
    }
}