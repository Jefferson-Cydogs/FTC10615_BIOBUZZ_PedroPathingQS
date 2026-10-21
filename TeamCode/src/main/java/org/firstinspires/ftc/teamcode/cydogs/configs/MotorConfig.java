package org.firstinspires.ftc.teamcode.cydogs.configs;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePositionalMotor;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Motor Config")
public class MotorConfig extends OpMode {


    private static final int TEST_MIN = -1_000_000;
    private static final int TEST_MAX = 1_000_000;
    private static final double TEST_POWER = 0.5;
    private final List<String> motorNames = new ArrayList<>();
    private int selectedIndex = 0;
    private boolean selectionLocked = false;
    private BasePositionalMotor selectedMotor;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight, prevA, prevB, prevX;
    private static final double CURRENT_LIMIT_AMPS = 4.0;   // placeholder: check your motor's spec
    private String statusMessage = "";

    @Override
    public void init() {
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            motorNames.add(entry.getKey());
        }

        if (motorNames.isEmpty()) {
            telemetry.addLine("No motors found in the robot configuration!");
        } else {
            telemetry.addLine("Found " + motorNames.size() + " motor(s).");
            telemetry.addLine("D-Pad Up/Down to select, A to confirm.");
        }
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (motorNames.isEmpty()) {
            return;
        }

        boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;
        boolean aPressed = gamepad1.a && !prevA;

        // Keep every button state current each loop, so a button held when you press START
        // isn't counted as a new press in loop()
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;

        if (!selectionLocked) {
            if (dpadUpPressed) {
                selectedIndex = (selectedIndex - 1 + motorNames.size()) % motorNames.size();
            }
            if (dpadDownPressed) {
                selectedIndex = (selectedIndex + 1) % motorNames.size();
            }
            if (aPressed) {
                selectionLocked = true;
                selectedMotor = new BasePositionalMotor(
                        this,
                        motorNames.get(selectedIndex),
                        DcMotorSimple.Direction.FORWARD,
                        DcMotor.ZeroPowerBehavior.BRAKE,
                        TEST_MIN,
                        TEST_MAX);
                selectedMotor.SetIncrementPower(TEST_POWER);
                selectedMotor.Initialize();
                selectedMotor.SetCurrentLimit(CURRENT_LIMIT_AMPS);   // pick a value below the motor's stall current
            }
        }

        if (selectionLocked) {
            telemetry.addLine("Locked on: " + motorNames.get(selectedIndex));
            telemetry.addLine("Press START to begin tuning.");
        } else {
            telemetry.addLine("Select a motor (D-Pad Up/Down), then press A:");
            for (int i = 0; i < motorNames.size(); i++) {
                String marker = (i == selectedIndex) ? ">> " : "    ";
                telemetry.addLine(marker + motorNames.get(i));
            }
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if (selectedMotor == null) {
            telemetry.addLine("No motor was selected during init!");
            telemetry.update();
            return;
        }
        if (selectedMotor.IsOverCurrent()) {
            selectedMotor.Stop();
            statusMessage = "OVER CURRENT - motor stopped";
        }

        boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;
        boolean dpadRightPressed = gamepad1.dpad_right && !prevDpadRight;
        boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
        boolean bPressed = gamepad1.b && !prevB;
        boolean xPressed = gamepad1.x && !prevX;

        if (dpadUpPressed) {
            selectedMotor.UpBigIncrement();
        }
        if (dpadDownPressed) {
            selectedMotor.DownBigIncrement();
        }
        if (dpadRightPressed) {
            selectedMotor.UpSmallIncrement();
        }
        if (dpadLeftPressed) {
            selectedMotor.DownSmallIncrement();
        }
        if (bPressed) {
            selectedMotor.HoldPosition();
        }
        if (xPressed) {
            selectedMotor.ResetEncoder();
        }

        if (dpadUpPressed || dpadDownPressed || dpadRightPressed || dpadLeftPressed || bPressed) {
            statusMessage = "";   // any new command clears the old warning
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadLeft = gamepad1.dpad_left;
        prevB = gamepad1.b;
        prevX = gamepad1.x;

        telemetry.addData("Controlling motor", motorNames.get(selectedIndex));
        telemetry.addData("Position(tick)", selectedMotor.GetPosition());
        telemetry.addData("Power", "%.2f", selectedMotor.GetPower());
        telemetry.addData("Current(A)", "%.2f", selectedMotor.GetCurrent());
        telemetry.addData("Busy", selectedMotor.IsBusy());
        telemetry.addLine("Up/Down=+-100 ticks Left/Right = +- 10 ticks");

        if (!statusMessage.isEmpty()) {
            telemetry.addLine(statusMessage);
        }

        telemetry.addLine("B=Hold position  X=Reset encoder to 0");
        telemetry.update();


    }
}

