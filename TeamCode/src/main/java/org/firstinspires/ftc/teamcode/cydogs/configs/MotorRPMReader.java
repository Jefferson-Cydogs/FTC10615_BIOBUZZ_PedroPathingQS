package org.firstinspires.ftc.teamcode.cydogs.configs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp
public class MotorRPMReader extends LinearOpMode {

    private DcMotor motor;
    private ElapsedTime timer = new ElapsedTime();

    double currentTime;
    int currentPosition;
    double deltaTime;
    int deltaTicks;
    int lastEncoderPosition = 0;
    double lastTime = 0;
    double ticksPerSecond;
    double rpm;
    double maxrpm = 0;

    // Set this to the encoder ticks per revolution of the motor shaft
    private static final double TICKS_PER_REV = 28.0; //value at the shaft (without gearbox)

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "LauncherMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            currentTime = timer.seconds();
            currentPosition = motor.getCurrentPosition();

            deltaTime = currentTime - lastTime;
            deltaTicks = currentPosition - lastEncoderPosition;

            lastTime = currentTime;
            lastEncoderPosition = currentPosition;

            ticksPerSecond = deltaTicks / deltaTime;
            rpm = (ticksPerSecond / TICKS_PER_REV) * 60.0;
            if (rpm > maxrpm) {
                maxrpm = rpm;
            }

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("RPM", rpm);
            telemetry.addData("Max RPM", maxrpm);
            telemetry.update();

            motor.setPower(1.0); // Run motor at full power
            sleep(100); // Update every 100ms
        }

        motor.setPower(0);
    }
}
