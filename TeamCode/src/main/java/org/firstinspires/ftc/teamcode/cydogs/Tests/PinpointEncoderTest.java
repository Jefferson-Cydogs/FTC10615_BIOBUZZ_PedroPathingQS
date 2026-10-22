package org.firstinspires.ftc.teamcode.cydogs.Tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pinpoint Encoder Test", group = "Diagnostics")
public class PinpointEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        "Pinpoint"
                );

        telemetry.addLine("Pinpoint Encoder Test");
        telemetry.addLine("Lift robot safely before testing.");
        telemetry.addLine("Press START, then rotate each pod by hand.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read the latest data from the Pinpoint.
            pinpoint.update();

            // Display raw encoder counts.
            telemetry.addData("Raw X Encoder (forward)",
                    pinpoint.getEncoderX());

            telemetry.addData("Raw Y Encoder (strafe)",
                    pinpoint.getEncoderY());

            telemetry.addData("Device Status",
                    pinpoint.getDeviceStatus());

            telemetry.update();
        }
    }
}
