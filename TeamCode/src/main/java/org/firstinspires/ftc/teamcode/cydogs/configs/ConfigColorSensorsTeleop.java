package org.firstinspires.ftc.teamcode.cydogs.configs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseColorSensor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;
import org.firstinspires.ftc.teamcode.cydogs.core.TargetColor;

/**
 * Config tool for the three color sensors. It shows what each sensor sees on telemetry, all the
 * time: during init (before you press START) and while running. Nothing on the robot moves.
 *
 * For each sensor it shows:
 *   - Matches: every TargetColor the sensor matches right now (the whole enum, so it shows more
 *     than one if the hue ranges overlap). This uses the same check as FlingyMag, so it says
 *     "none" for anything farther than the distance limit in BaseColorSensor, or when the
 *     color is not in any range.
 *   - The raw readings from BaseColorSensor.WhatDoISee(): normalized RGB, distance, RGB and HSV.
 *     Read the "H:" value to tune the hue ranges in TargetColor.
 */
@TeleOp(name = "Config Color Sensors", group = "Config")
public class ConfigColorSensorsTeleop extends OpMode {

    private BaseColorSensor intakeRightSensor;
    private BaseColorSensor intakeLeftSensor;
    private BaseColorSensor launcherSensor;

    @Override
    public void init() {
        intakeRightSensor = new BaseColorSensor(this, BeethovenHardware.FlingyMag.INTAKE_SENSOR_RIGHT_NAME);
        intakeLeftSensor = new BaseColorSensor(this, BeethovenHardware.FlingyMag.INTAKE_SENSOR_LEFT_NAME);
        launcherSensor = new BaseColorSensor(this, BeethovenHardware.FlingyMag.LAUNCHER_SENSOR_NAME);

        telemetry.addLine("Color sensors ready. Readings show now and keep updating.");
        telemetry.update();
    }

    // Keep showing the readings before START is pressed
    @Override
    public void init_loop() {
        showAllSensors();
    }

    // And keep showing them while running
    @Override
    public void loop() {
        showAllSensors();
    }

    private void showAllSensors() {
        showSensor("INTAKE RIGHT (" + BeethovenHardware.FlingyMag.INTAKE_SENSOR_RIGHT_NAME + ")", intakeRightSensor);
        showSensor("INTAKE LEFT (" + BeethovenHardware.FlingyMag.INTAKE_SENSOR_LEFT_NAME + ")", intakeLeftSensor);
        showSensor("LAUNCHER (" + BeethovenHardware.FlingyMag.LAUNCHER_SENSOR_NAME + ")", launcherSensor);
        telemetry.update();
    }

    // Shows one sensor: a header, which colors it matches, then its raw readings
    private void showSensor(String label, BaseColorSensor sensor) {
        telemetry.addLine("===== " + label + " =====");
        telemetry.addData("Matches", matchingColors(sensor));
        sensor.WhatDoISee();
    }

    // Returns the names of every TargetColor this sensor matches right now, or "none".
    // New colors added to TargetColor show up here automatically.
    private String matchingColors(BaseColorSensor sensor) {
        StringBuilder matches = new StringBuilder();
        for (TargetColor color : TargetColor.values()) {
            if (sensor.SeeArtifactColor(color)) {
                matches.append(color).append(" ");
            }
        }
        return matches.length() == 0 ? "none" : matches.toString().trim();
    }
}