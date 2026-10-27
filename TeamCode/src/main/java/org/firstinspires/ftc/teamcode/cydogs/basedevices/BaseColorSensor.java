package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    private LinearOpMode opMode;
    private NormalizedColorSensor colorSensor;

    public BaseColorSensor(LinearOpMode opMode, String ColorDistanceSensor)
    {
        this.opMode = opMode;
        HardwareMap hw = opMode.hardwareMap;
        this.colorSensor = hw.get(NormalizedColorSensor.class, ColorDistanceSensor);

        // Adjust the gain, as the sensor may default to an unpredictable value, leading to inconsistent color readings.
        // Recommendation based on FTC documentation: 10.0 - 20.0 for bright lighting or LEDs, 20.0 - 40.0 for normal indoor lighting.
        // 30.0 is a safe default for FTC
        colorSensor.setGain(30);
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

    public boolean SeeSquareColor(TargetColor targetColor)
    {
        int matchCount = 0;

        for (int i = 0; i < 3; i++) {
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