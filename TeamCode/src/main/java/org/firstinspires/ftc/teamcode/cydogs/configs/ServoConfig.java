package org.firstinspires.ftc.teamcode.cydogs.configs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

@TeleOp(name = "Servo Config")
public class ServoConfig extends OpMode {


    private static final double TEST_MIN = 0.0;
    private static final double TEST_MAX = 1.0;
    private static final double TEST_START = 0.5;

    private final List<String> servoNames = new ArrayList<>();
    private int selectedIndex = 0;
    private boolean selectionLocked = false;
    private BaseServo selectedServo;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight, prevA;

    @Override
    public void init() {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {

            servoNames.add(entry.getKey());
        }


        if (servoNames.isEmpty()) {
            telemetry.addLine("No servos found in the robot configuration!");
        } else {

            telemetry.addLine("Found " + servoNames.size() + " servo(s).");

            telemetry.addLine("Dpad Up/Down to select, A to confirm.");

        }
        telemetry.update();
    }


    @Override
    public void init_loop() {
        if (servoNames.isEmpty()) {
            return;
        }

        boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;
        boolean aPressed = gamepad1.a && !prevA;

        // Keep these current every loop so each press counts once
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevA = gamepad1.a;

        if (!selectionLocked) {
            if (dpadUpPressed) {
                selectedIndex = (selectedIndex - 1 + servoNames.size()) % servoNames.size();
            }
            if (dpadDownPressed) {
                selectedIndex = (selectedIndex + 1) % servoNames.size();
            }
            if (aPressed) {
                selectionLocked = true;
                selectedServo = new BaseServo(
                        this,
                        servoNames.get(selectedIndex),
                        Servo.Direction.FORWARD,
                        TEST_MIN,
                        TEST_MAX,
                        TEST_START);
                selectedServo.Initialize();
            }
        }

        if (selectionLocked) {
            telemetry.addLine("Locked on: " + servoNames.get(selectedIndex));
            telemetry.addLine("Press START to begin tuning.");
        } else {
            telemetry.addLine("Select a servo (D-pad Up/Down), then press A:");
            for (int i = 0; i < servoNames.size(); i++) {
                String marker = (i == selectedIndex) ? ">> " : "    ";
                telemetry.addLine(marker + servoNames.get(i));
            }
        }
        telemetry.update();
    }


    @Override
    public void start() {
                if(selectedServo != null) {
                    selectedServo.SetStartingPosition();
                }

    }




@Override
 public void loop() {
            if (selectedServo == null) {
                telemetry.addLine("No servo was selected during init!");
                telemetry.update();
                return;
            }

            boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
            boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;
            boolean dpadRightPressed = gamepad1.dpad_right && !prevDpadRight;
            boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;

            if (dpadUpPressed) {
                selectedServo.UpBigIncrement();

            }
            if (dpadDownPressed) {
                selectedServo.DownBigIncrement();

            }
            if (dpadRightPressed) {
                selectedServo.UpSmallIncrement();

            }
            if (dpadLeftPressed) {
                selectedServo.DownSmallIncrement();
            }

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft = gamepad1.dpad_left;

            telemetry.addData("Controlling servo", servoNames.get(selectedIndex));
            telemetry.addData("Position", "%.3f", selectedServo.GetPosition());
            telemetry.addLine("Up/Down = +-0.1 Left/Right +-0.01");
            telemetry.update();

        }

    }




