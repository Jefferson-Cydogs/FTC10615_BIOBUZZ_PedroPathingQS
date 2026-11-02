package org.firstinspires.ftc.teamcode.cydogs.configs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * Launch-wheel PIDF tuner for FTC Dashboard.
 *
 * Dashboard variables:
 *     targetRPM
 *     kP
 *     kI
 *     kD
 *     kF
 *     wheelEnabled
 *     motorDirection
 *
 * FTC Dashboard address for a Control Hub:
 *     http://192.168.43.1:8080/dash
 */
@Config
@TeleOp(
        name = "Launch Wheel PIDF Tuner",
        group = "Tuning"
)
public class LaunchWheelPIDFTuner extends LinearOpMode {

    /*
     * All public static non-final variables appear in the FTC Dashboard Config panel.
     */

    /**
     * Requested launch-wheel speed.
     */
    public static volatile double targetRPM = 2600.0;

    /**
     * Velocity PIDF coefficients.
     *
     * These are starting values only.
     * Tune F first, then P, then D, and use I only if needed.
     */
    public static volatile double kP = 0.0;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0.0;
    public static volatile double kF = 11.7;

    /**
     * The 1:1 6000 RPM Yellow Jacket has a 28 PPR
     * output-shaft encoder.
     */
    public static volatile double ticksPerRevolution = 28.0;

    /**
     * Software limit for this tuning OpMode.
     */
    public static volatile double maximumTestRPM = 5334.0;

    /**
     * Set true in FTC Dashboard to run the wheel.
     */
    public static volatile boolean wheelEnabled = false;

    /**
     * Set to 1 for positive velocity.
     * Set to -1 if the wheel must run in the opposite direction.
     */
    public static volatile int motorDirection = 1;

    /**
     * Limits Dashboard transmission frequency.
     *
     * 50 ms produces approximately 20 Dashboard updates per second.
     */
    public static volatile long dashboardIntervalMs = 50;

    /*
     * Diagnostic variable.
     * Change this in Dashboard to verify live configuration updates.
     */
    public static volatile int testMarker = 0;

    private DcMotorEx launchWheel;
    private FtcDashboard dashboard;

    /*
     * Last coefficients sent to the REV motor controller.
     */
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    private long lastDashboardUpdateMs = 0;


    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        /*
         * This name must exactly match the motor name in the
         * Robot Configuration.
         */
        launchWheel = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
        /*
         * FLOAT allows the flywheel to coast when disabled.
         */
        launchWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*
         * Velocity control requires RUN_USING_ENCODER.
         */
        launchWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDFIfChanged();

        telemetry.addLine("Launch-wheel PIDF tuner initialized");
        telemetry.addLine("Use FTC Dashboard Config to change PIDF values");
        telemetry.addLine("Set wheelEnabled=true to start the wheel");
        telemetry.addData("Motor configuration name", "launchWheel");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            launchWheel.setPower(0.0);
            return;
        }

        try {
            while (opModeIsActive()) {
                /*
                 * Reapply PIDF when a value is changed from
                 * FTC Dashboard.
                 */
                applyPIDFIfChanged();

                double safeRPM = clamp(Math.abs(targetRPM), 0.0, Math.abs(maximumTestRPM));

                int safeDirection = motorDirection >= 0 ? 1 : -1;

                double requestedTicksPerSecond = rpmToTicksPerSecond(safeRPM);
                double targetTicksPerSecond = safeDirection * requestedTicksPerSecond;

                if (wheelEnabled) {
                    launchWheel.setVelocity(targetTicksPerSecond);
                } else {
                    launchWheel.setPower(0.0);
                    targetTicksPerSecond = 0.0;
                }

                double measuredTicksPerSecond = launchWheel.getVelocity();

                double targetSignedRPM = ticksPerSecondToRPM(targetTicksPerSecond);
                double measuredRPM = ticksPerSecondToRPM(measuredTicksPerSecond);

                double velocityError = targetTicksPerSecond - measuredTicksPerSecond;
                double rpmError = targetSignedRPM - measuredRPM;

                double motorPower = launchWheel.getPower();
                double motorCurrent = launchWheel.getCurrent(CurrentUnit.AMPS);

                /*
                 * Send slower text telemetry to the Driver Station.
                 */
                telemetry.addData("Wheel enabled", wheelEnabled);
                telemetry.addData("Target RPM", "%.1f", targetSignedRPM);
                telemetry.addData("Measured RPM", "%.1f", measuredRPM);
                telemetry.addData("RPM error", "%.1f", rpmError);
                telemetry.addData("Motor power", "%.3f", motorPower);
                telemetry.addData("Motor current", "%.2f A", motorCurrent);
                telemetry.addData("PIDF", "P=%.4f I=%.4f D=%.4f F=%.4f", kP, kI, kD, kF);
                telemetry.update();

                /*
                 * Send numeric values to FTC Dashboard.
                 *
                 * These keys can be selected as graph series.
                 */
                long currentTimeMs = System.currentTimeMillis();

                if (currentTimeMs - lastDashboardUpdateMs >= Math.max(20, dashboardIntervalMs)) {
                    TelemetryPacket packet = new TelemetryPacket();

                    packet.put("Target RPM", targetSignedRPM);
                    packet.put("Measured RPM", measuredRPM);
                    packet.put("RPM Error", rpmError);
                    packet.put("Target ticks/sec", targetTicksPerSecond);
                    packet.put("Measured ticks/sec", measuredTicksPerSecond);
                    packet.put("Velocity Error", velocityError);
                    packet.put("Motor Power", motorPower);
                    packet.put("Motor Current A", motorCurrent);
                    packet.put("Wheel Enabled", wheelEnabled);

                    packet.put("kP", kP);
                    packet.put("kI", kI);
                    packet.put("kD", kD);
                    packet.put("kF", kF);

                    dashboard.sendTelemetryPacket(packet);

                    lastDashboardUpdateMs = currentTimeMs;
                }

                sleep(10);
            }
        } finally {
            /*
             * Always stop the motor when the OpMode exits,
             * including when an exception occurs.
             */
            launchWheel.setPower(0.0);
        }
    }


    /**
     * Applies new PIDF values only when one has changed.
     */
    private void applyPIDFIfChanged() {
        boolean changed =
                Double.compare(kP, appliedP) != 0
                        || Double.compare(kI, appliedI) != 0
                        || Double.compare(kD, appliedD) != 0
                        || Double.compare(kF, appliedF) != 0;

        if (!changed) {
            return;
        }

        launchWheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        appliedP = kP;
        appliedI = kI;
        appliedD = kD;
        appliedF = kF;
    }

    /**
     * Converts output-shaft RPM to encoder ticks per second.
     */
    private double rpmToTicksPerSecond(double rpm) {
        if (ticksPerRevolution <= 0.0) {
            return 0.0;
        }
        return rpm * ticksPerRevolution / 60.0;
    }

    /**
     * Converts encoder ticks per second to output-shaft RPM.
     */
    private double ticksPerSecondToRPM(double ticksPerSecond) {
        if (ticksPerRevolution <= 0.0) {
            return 0.0;
        }
        return ticksPerSecond * 60.0 / ticksPerRevolution;
    }

    /**
     * Restricts a number to a safe range.
     */
    private double clamp(double value, double minimum, double maximum) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}
