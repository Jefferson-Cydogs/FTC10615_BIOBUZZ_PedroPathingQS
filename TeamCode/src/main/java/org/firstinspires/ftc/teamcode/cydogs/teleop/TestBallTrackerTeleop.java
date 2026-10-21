package org.firstinspires.ftc.teamcode.cydogs.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cydogs.components.BallTracker;

@TeleOp(name = "Ball Assist TeleOp")
public class TestBallTrackerTeleop extends LinearOpMode {
    static final double KP_TURN = 0.004;
    static final double ASSIST_DRIVE_POWER = 0.4;
    static final int STOP_WIDTH = 120;

    @Override
    public void runOpMode() {
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        BallTracker detector = new BallTracker(
                hardwareMap, "huskylens", BallTracker.Alliance.BLUE);

        // Pick alliance during init
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) detector.setAlliance(BallTracker.Alliance.BLUE);
            if (gamepad1.b) detector.setAlliance(BallTracker.Alliance.RED);
            telemetry.addData("Alliance (X=blue, B=red)", detector.getAlliance());
            telemetry.addData("HuskyLens connected", detector.isConnected());
            telemetry.update();
        }

        while (opModeIsActive()) {
            detector.update();

            double forward, turn;
            if (gamepad1.left_bumper && detector.hasTarget()) {
                // Assist: hold left bumper to auto-drive toward nearest valid ball
                if (detector.isWithin(STOP_WIDTH)) {
                    forward = 0; turn = 0;
                } else {
                    forward = ASSIST_DRIVE_POWER;
                    turn = detector.getXError() * KP_TURN;
                }
            } else {
                // Manual driving
                forward = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;
            }

            left.setPower(Range.clip(forward + turn, -1, 1));
            right.setPower(Range.clip(forward - turn, -1, 1));

            telemetry.addData("Has target", detector.hasTarget());
            telemetry.addData("Target id", detector.getTargetId());
            telemetry.addData("X error", detector.getXError());
            telemetry.addData("Width", detector.getTargetWidth());
            telemetry.update();
        }
    }
}