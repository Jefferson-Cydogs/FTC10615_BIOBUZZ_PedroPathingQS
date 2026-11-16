package org.firstinspires.ftc.teamcode.cydogs.configs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * Flywheel PIDF tuner controlled entirely from Gamepad 1.
 *
 * FTC Dashboard is used only for telemetry and graphing.
 * Dashboard Config is not used.
 *
 * Motor configuration name:
 *     launchWheel
 *
 * GAMEPAD 1
 *
 * A                  Enable/disable wheel
 * D-pad left/right   Select RPM, F, P, D, or I
 * Left bumper        Decrease selected value
 * Right bumper       Increase selected value
 * X                  Cycle adjustment size
 * Y                  Arm/reset pollen impact measurement
 * B                  Set selected PIDF coefficient to zero
 * START + BACK       Emergency wheel disable
 *
 * Recommended order:
 *     F -> P -> D only if needed -> I only if needed
 */
@TeleOp(
        name = "Flywheel PIDF Tuner - Gamepad + Dashboard",
        group = "Tuning"
)
public class FlywheelPIDFTunerGamepad extends LinearOpMode {

    /*
     * Must exactly match the Robot Configuration.
     */
    private static final String MOTOR_NAME = "LauncherMotor";
    /*
     * 1:1 6000 RPM goBILDA Yellow Jacket:
     * 28 encoder ticks per output-shaft revolution.
     */
    private static final double TICKS_PER_REVOLUTION = 28.0;
    private static final double MAXIMUM_TEST_RPM = 5300.0;
    /*
     * Starting test values.
     *
     * F = 11.7 is the theoretical value based on:
     *
     *     6000 RPM / 60 * 28 = 2800 ticks/sec
     *     32767 / 2800 = 11.70
     */
    private double targetRPM = 2400.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 13.6;

    /*
     * Change to -1 if the commanded direction is wrong.
     */
    private int motorDirection = 1;

    private boolean wheelEnabled = false;

    private DcMotorEx launchWheel;
    private FtcDashboard dashboard;

    /*
     * Values most recently applied to the REV Hub.
     */
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    /*
     * User interface selections.
     */
    private Parameter selectedParameter = Parameter.RPM;
    private StepSize selectedStepSize = StepSize.MEDIUM;

    /*
     * Button edge detection.
     */
    private boolean previousA;
    private boolean previousB;
    private boolean previousX;
    private boolean previousY;

    private boolean previousLeftBumper;
    private boolean previousRightBumper;

    private boolean previousDpadLeft;
    private boolean previousDpadRight;

    /*
     * Dashboard update timing.
     */
    private long lastDashboardUpdateMs = 0;
    private static final long DASHBOARD_INTERVAL_MS = 40;

    /*
     * Impact-test measurements.
     */
    private ImpactState impactState = ImpactState.IDLE;

    private double impactMinimumRPM = Double.NaN;
    private double impactMaximumRPM = Double.NaN;
    private double impactDropRPM = Double.NaN;
    private double impactOvershootRPM = Double.NaN;
    private double impactRecoverySeconds = Double.NaN;

    private long impactStartNanos;
    private long recoveryBandStartNanos;

    /*
     * A drop greater than 3% arms impact detection.
     */
    private static final double IMPACT_DROP_PERCENT = 0.03;

    /*
     * Recovered means within 2% of target.
     */
    private static final double RECOVERY_TOLERANCE_PERCENT = 0.02;

    /*
     * Must remain recovered for 0.20 seconds.
     */
    private static final double RECOVERY_HOLD_SECONDS = 0.20;


    private enum Parameter {
        RPM,
        F,
        P,
        D,
        I
    }


    private enum StepSize {
        FINE,
        MEDIUM,
        COARSE
    }


    private enum ImpactState {
        IDLE,
        ARMED,
        IMPACT_DETECTED,
        RECOVERY_HOLD,
        COMPLETE
    }


    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        launchWheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        /*
         * A flywheel normally should coast rather than brake.
         */
        launchWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*
         * Velocity control requires RUN_USING_ENCODER.
         */
        launchWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDFIfChanged();

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("GAMEPAD + DASHBOARD PIDF TUNER");
        telemetry.addLine("");
        telemetry.addLine("A: enable/disable flywheel");
        telemetry.addLine("D-pad left/right: select value");
        telemetry.addLine("LB/RB: decrease/increase");
        telemetry.addLine("X: select adjustment size");
        telemetry.addLine("Y: arm pollen impact test");
        telemetry.addLine("B: zero selected PIDF term");
        telemetry.addLine("START + BACK: emergency disable");
        telemetry.addLine("");
        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.addData(
                "Starting PIDF",
                "P=%.4f I=%.4f D=%.4f F=%.4f",
                kP,
                kI,
                kD,
                kF
        );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            launchWheel.setPower(0.0);
            return;
        }

        try {
            while (opModeIsActive()) {

                processGamepad();

                /*
                 * Push new coefficients to the REV Hub
                 * whenever the gamepad changes one.
                 */
                applyPIDFIfChanged();
                double safeTargetRPM = clamp(targetRPM, 0.0, MAXIMUM_TEST_RPM);
                double targetTicksPerSecond = rpmToTicksPerSecond(safeTargetRPM) * motorDirection;

                if (wheelEnabled) {
                    launchWheel.setVelocity(targetTicksPerSecond);
                } else {
                    launchWheel.setPower(0.0);
                    targetTicksPerSecond = 0.0;
                }

                double measuredTicksPerSecond = launchWheel.getVelocity();
                double signedTargetRPM = ticksPerSecondToRPM(targetTicksPerSecond);
                double measuredRPM = ticksPerSecondToRPM(measuredTicksPerSecond);
                double rpmError = signedTargetRPM - measuredRPM;
                double motorPower = launchWheel.getPower();
                double motorCurrent = launchWheel.getCurrent(CurrentUnit.AMPS);

                updateImpactTest(
                        Math.abs(signedTargetRPM),
                        Math.abs(measuredRPM)
                );

                sendDriverStationTelemetry(
                        signedTargetRPM,
                        measuredRPM,
                        rpmError,
                        measuredTicksPerSecond,
                        motorPower,
                        motorCurrent
                );

                sendDashboardTelemetry(
                        signedTargetRPM,
                        measuredRPM,
                        rpmError,
                        measuredTicksPerSecond,
                        motorPower,
                        motorCurrent
                );

                sleep(10);
            }

        } finally {
            launchWheel.setPower(0.0);
        }
    }


    private void processGamepad() {

        boolean aPressed = gamepad1.a && !previousA;
        boolean bPressed = gamepad1.b && !previousB;
        boolean xPressed = gamepad1.x && !previousX;
        boolean yPressed = gamepad1.y && !previousY;
        boolean leftBumperPressed = gamepad1.left_bumper && !previousLeftBumper;
        boolean rightBumperPressed = gamepad1.right_bumper && !previousRightBumper;
        boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;
        /*
         * Emergency stop.
         */
        if (gamepad1.start && gamepad1.back) {
            wheelEnabled = false;
            impactState = ImpactState.IDLE;
        }
        /*
         * Enable or disable the flywheel.
         */
        if (aPressed) {
            wheelEnabled = !wheelEnabled;
            if (!wheelEnabled) {
                impactState = ImpactState.IDLE;
            }
        }
        /*
         * Select a tuning field.
         */
        if (dpadLeftPressed) {selectPreviousParameter();}
        if (dpadRightPressed) {selectNextParameter();}
        /*
         * Choose Fine, Medium, or Coarse.
         */
        if (xPressed) {cycleStepSize();}
        /*
         * Decrease or increase selected value.
         */
        if (leftBumperPressed) {adjustSelectedParameter(-1.0);}
        if (rightBumperPressed) {adjustSelectedParameter(1.0);}
        /*
         * Zero the selected PIDF coefficient.
         * Does nothing when RPM is selected.
         */
        if (bPressed) {zeroSelectedCoefficient();}
        /*
         * Prepare to measure one pollen impact.
         */
        if (yPressed) {armImpactTest();}

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;

        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;
    }


    private void selectPreviousParameter() {
        switch (selectedParameter) {
            case RPM:
                selectedParameter = Parameter.I;
                break;
            case F:
                selectedParameter = Parameter.RPM;
                break;
            case P:
                selectedParameter = Parameter.F;
                break;
            case D:
                selectedParameter = Parameter.P;
                break;
            case I:
                selectedParameter = Parameter.D;
                break;
        }
    }


    private void selectNextParameter() {
        switch (selectedParameter) {
            case RPM:
                selectedParameter = Parameter.F;
                break;
            case F:
                selectedParameter = Parameter.P;
                break;
            case P:
                selectedParameter = Parameter.D;
                break;
            case D:
                selectedParameter = Parameter.I;
                break;
            case I:
                selectedParameter = Parameter.RPM;
                break;
        }
    }


    private void cycleStepSize() {
        switch (selectedStepSize) {
            case FINE:
                selectedStepSize = StepSize.MEDIUM;
                break;
            case MEDIUM:
                selectedStepSize = StepSize.COARSE;
                break;
            case COARSE:
                selectedStepSize = StepSize.FINE;
                break;
        }
    }


    private void adjustSelectedParameter(double direction) {

        double adjustment = getSelectedAdjustment();

        switch (selectedParameter) {
            case RPM:
                targetRPM = clamp(targetRPM + direction * adjustment, 0.0, MAXIMUM_TEST_RPM);
                break;
            case F:
                kF = Math.max(0.0, kF + direction * adjustment);
                break;
            case P:
                kP = Math.max(0.0, kP + direction * adjustment);
                break;
            case D:
                kD = Math.max(0.0, kD + direction * adjustment);
                break;
            case I:
                kI = Math.max(0.0, kI + direction * adjustment);
                break;
        }
    }


    private double getSelectedAdjustment() {

        switch (selectedParameter) {
            case RPM:
                switch (selectedStepSize) {
                    case FINE:
                        return 25.0;
                    case MEDIUM:
                        return 100.0;
                    case COARSE:
                        return 250.0;
                }
                break;
            case F:
                switch (selectedStepSize) {
                    case FINE:
                        return 0.005;
                    case MEDIUM:
                        return 0.05;
                    case COARSE:
                        return 0.5;
                }
                break;
            case P:
                switch (selectedStepSize) {
                    case FINE:
                        return 0.02;
                    case MEDIUM:
                        return 0.10;
                    case COARSE:
                        return 0.50;
                }
                break;
            case D:
                switch (selectedStepSize) {
                    case FINE:
                        return 0.001;
                    case MEDIUM:
                        return 0.01;
                    case COARSE:
                        return 0.10;
                }
                break;
            case I:
                switch (selectedStepSize) {
                    case FINE:
                        return 0.001;
                    case MEDIUM:
                        return 0.01;
                    case COARSE:
                        return 0.05;
                }
                break;
        }
        return 0.0;
    }


    private void zeroSelectedCoefficient() {

        switch (selectedParameter) {
            case F:
                kF = 0.0;
                break;
            case P:
                kP = 0.0;
                break;
            case D:
                kD = 0.0;
                break;
            case I:
                kI = 0.0;
                break;
            case RPM:
                /*
                 * Intentionally do nothing.
                 */
                break;
        }
    }


    /**
     * Applies PIDF values only when the gamepad has changed
     * one of them.
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
                new PIDFCoefficients(
                        kP,
                        kI,
                        kD,
                        kF
                )
        );

        appliedP = kP;
        appliedI = kI;
        appliedD = kD;
        appliedF = kF;
    }


    private void armImpactTest() {

        impactMinimumRPM = Double.NaN;
        impactMaximumRPM = Double.NaN;
        impactDropRPM = Double.NaN;
        impactOvershootRPM = Double.NaN;
        impactRecoverySeconds = Double.NaN;

        impactStartNanos = 0;
        recoveryBandStartNanos = 0;

        if (wheelEnabled && targetRPM > 0.0) {
            impactState = ImpactState.ARMED;
        } else {
            impactState = ImpactState.IDLE;
        }
    }


    private void updateImpactTest(double absoluteTargetRPM, double absoluteMeasuredRPM) {

        if (!wheelEnabled || absoluteTargetRPM <= 0.0) {
            if (impactState != ImpactState.COMPLETE) {
                impactState = ImpactState.IDLE;
            }
            return;
        }
        double impactThreshold = absoluteTargetRPM * (1.0 - IMPACT_DROP_PERCENT);
        double tolerance = absoluteTargetRPM * RECOVERY_TOLERANCE_PERCENT;
        double lowerRecoveryLimit = absoluteTargetRPM - tolerance;
        double upperRecoveryLimit = absoluteTargetRPM + tolerance;

        switch (impactState) {
            case IDLE:
                break;
            case ARMED:
                if (absoluteMeasuredRPM < impactThreshold) {
                    impactState = ImpactState.IMPACT_DETECTED;
                    impactStartNanos = System.nanoTime();
                    impactMinimumRPM = absoluteMeasuredRPM;
                    impactMaximumRPM = absoluteMeasuredRPM;
                }
                break;
            case IMPACT_DETECTED:
                impactMinimumRPM = Math.min(impactMinimumRPM, absoluteMeasuredRPM);
                impactMaximumRPM = Math.max(impactMaximumRPM, absoluteMeasuredRPM);
                if (absoluteMeasuredRPM
                        >= lowerRecoveryLimit
                        && absoluteMeasuredRPM
                        <= upperRecoveryLimit) {
                    impactState = ImpactState.RECOVERY_HOLD;
                    recoveryBandStartNanos = System.nanoTime();
                }
                break;
            case RECOVERY_HOLD:
                impactMinimumRPM = Math.min(impactMinimumRPM, absoluteMeasuredRPM);
                impactMaximumRPM = Math.max(impactMaximumRPM, absoluteMeasuredRPM);
                boolean stillRecovered =
                        absoluteMeasuredRPM
                                >= lowerRecoveryLimit
                                && absoluteMeasuredRPM
                                <= upperRecoveryLimit;
                if (!stillRecovered) {
                    impactState = ImpactState.IMPACT_DETECTED;
                    recoveryBandStartNanos = 0;
                } else {
                    double holdSeconds = nanosToSeconds(System.nanoTime() - recoveryBandStartNanos);
                    if (holdSeconds >= RECOVERY_HOLD_SECONDS) {
                        impactRecoverySeconds = nanosToSeconds(recoveryBandStartNanos - impactStartNanos);
                        impactDropRPM = absoluteTargetRPM - impactMinimumRPM;
                        impactOvershootRPM = Math.max(0.0, impactMaximumRPM - absoluteTargetRPM);
                        impactState = ImpactState.COMPLETE;
                    }
                }
                break;
            case COMPLETE:
                /*
                 * Continue tracking late overshoot.
                 */
                impactMaximumRPM = Math.max(impactMaximumRPM, absoluteMeasuredRPM);
                impactOvershootRPM = Math.max(0.0, impactMaximumRPM - absoluteTargetRPM);
                break;
        }
    }


    private void sendDriverStationTelemetry(
            double signedTargetRPM,
            double measuredRPM,
            double rpmError,
            double measuredTicksPerSecond,
            double motorPower,
            double motorCurrent
    ) {

        telemetry.addLine("=== GAMEPAD + DASHBOARD TUNER ===");
        telemetry.addData(
                "Wheel",
                wheelEnabled
                        ? "ENABLED"
                        : "DISABLED"
        );
        telemetry.addData(
                "Selected",
                "%s | %s",
                selectedParameter,
                selectedStepSize
        );
        telemetry.addData("Adjustment", getSelectedAdjustment());
        telemetry.addLine("----------------------------");
        telemetry.addData(
                selectedParameter == Parameter.RPM
                        ? "> RPM"
                        : "  RPM",
                "%.1f",
                targetRPM
        );
        telemetry.addData(
                selectedParameter == Parameter.F
                        ? "> F"
                        : "  F",
                "%.4f",
                kF
        );
        telemetry.addData(
                selectedParameter == Parameter.P
                        ? "> P"
                        : "  P",
                "%.4f",
                kP
        );
        telemetry.addData(
                selectedParameter == Parameter.D
                        ? "> D"
                        : "  D",
                "%.4f",
                kD
        );
        telemetry.addData(
                selectedParameter == Parameter.I
                        ? "> I"
                        : "  I",
                "%.4f",
                kI
        );
        telemetry.addLine("----------------------------");
        telemetry.addData(
                "Target RPM",
                "%.1f",
                signedTargetRPM
        );
        telemetry.addData(
                "Measured RPM",
                "%.1f",
                measuredRPM
        );
        telemetry.addData(
                "RPM error",
                "%.1f",
                rpmError
        );
        telemetry.addData(
                "Ticks/sec",
                "%.1f",
                measuredTicksPerSecond
        );
        telemetry.addData(
                "Motor power",
                "%.3f",
                motorPower
        );
        telemetry.addData(
                "Motor current",
                "%.2f A",
                motorCurrent
        );
        telemetry.addLine("----------------------------");
        telemetry.addData(
                "Impact test",
                impactState
        );
        if (!Double.isNaN(impactMinimumRPM)) {
            telemetry.addData(
                    "Minimum RPM",
                    "%.1f",
                    impactMinimumRPM
            );
        }
        if (!Double.isNaN(impactDropRPM)) {
            telemetry.addData(
                    "RPM drop",
                    "%.1f",
                    impactDropRPM
            );
        }
        if (!Double.isNaN(impactRecoverySeconds)) {
            telemetry.addData(
                    "Recovery time",
                    "%.3f sec",
                    impactRecoverySeconds
            );
        }
        if (!Double.isNaN(impactOvershootRPM)) {
            telemetry.addData(
                    "Overshoot",
                    "%.1f RPM",
                    impactOvershootRPM
            );
        }
        telemetry.addLine("----------------------------");
        telemetry.addLine("A wheel | D-pad select | LB/RB change");
        telemetry.addLine("X step | Y impact | B zero PIDF term");

        telemetry.update();
    }


    /**
     * Sends numeric series to FTC Dashboard for plotting.
     *
     * Dashboard Config is not used.
     */
    private void sendDashboardTelemetry(
            double signedTargetRPM,
            double measuredRPM,
            double rpmError,
            double measuredTicksPerSecond,
            double motorPower,
            double motorCurrent
    ) {

        long currentTimeMs = System.currentTimeMillis();

        if (currentTimeMs - lastDashboardUpdateMs < DASHBOARD_INTERVAL_MS) {
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();

        /*
         * Main graph series.
         */
        packet.put("Flywheel/Target RPM", signedTargetRPM);
        packet.put("Flywheel/Measured RPM", measuredRPM);
        packet.put("Flywheel/RPM Error", rpmError);
        packet.put("Flywheel/Measured ticks-sec", measuredTicksPerSecond);
        packet.put("Flywheel/Motor Power", motorPower);
        packet.put("Flywheel/Motor Current A", motorCurrent);

        /*
         * Current gamepad-selected tuning values.
         */
        packet.put("Tuning/Target RPM Setting", targetRPM);
        packet.put("Tuning/kF", kF);
        packet.put("Tuning/kP", kP);
        packet.put("Tuning/kD", kD);
        packet.put("Tuning/kI", kI);

        packet.put("Tuning/Wheel Enabled", wheelEnabled);
        packet.put("Tuning/Selected Parameter", selectedParameter.toString());
        packet.put("Tuning/Step Size", selectedStepSize.toString());

        /*
         * Impact-test results.
         */
        packet.put("Impact/State", impactState.toString());

        if (!Double.isNaN(impactMinimumRPM)) {
            packet.put("Impact/Minimum RPM", impactMinimumRPM);
        }
        if (!Double.isNaN(impactDropRPM)) {
            packet.put("Impact/RPM Drop", impactDropRPM);
        }
        if (!Double.isNaN(impactRecoverySeconds)) {
            packet.put("Impact/Recovery Seconds", impactRecoverySeconds);
        }
        if (!Double.isNaN(impactOvershootRPM)) {
            packet.put("Impact/Overshoot RPM", impactOvershootRPM);
        }

        dashboard.sendTelemetryPacket(packet);

        lastDashboardUpdateMs =
                currentTimeMs;
    }


    private double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REVOLUTION / 60.0;
    }


    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REVOLUTION;
    }


    private double nanosToSeconds(long nanoseconds) {
        return nanoseconds / 1_000_000_000.0;
    }


    private double clamp(double value, double minimum, double maximum) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}
