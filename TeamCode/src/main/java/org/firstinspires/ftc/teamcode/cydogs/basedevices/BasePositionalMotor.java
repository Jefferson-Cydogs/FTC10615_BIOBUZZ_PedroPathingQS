package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class BasePositionalMotor {


    private static final int DEFAULT_POSITION_TOLERANCE = 10; // SDK default, in ticks
    private static final int BIG_INCREMENT = 100;       // ticks
    private static final int SMALL_INCREMENT = 10;      // ticks
    private static final double DEFAULT_INCREMENT_POWER = 0.5;

    /**
     * Positions are in encoder ticks.
     */
    private final OpMode opMode;
    private final String hardwareName;
    private final DcMotorSimple.Direction direction;
    private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private final int minPosition;
    private final int maxPosition;

    private DcMotorEx motor;
    private int positionTolerance = DEFAULT_POSITION_TOLERANCE;
    private double currentLimitAmps = 0; // 0 or less means stall protection is off
    private double incrementPower = DEFAULT_INCREMENT_POWER;


    public BasePositionalMotor(OpMode opMode,
                               String hardwareName,
                               DcMotorSimple.Direction direction,
                               DcMotor.ZeroPowerBehavior zeroPowerBehavior,
                               int minPosition,
                               int maxPosition) {

        if (minPosition > maxPosition) {
            throw new IllegalArgumentException(
                    "BasePositionMotor'" + hardwareName + "': minPosition (" + minPosition
                            + ") cannot be greater than maxPosition (" + maxPosition + ")");
        }

        this.opMode = opMode;
        this.hardwareName = hardwareName;
        this.direction = direction;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    /**
     * Gets the motor from the hardware map, applies direction, zero power behavior, and
     * position
     * tolerance, and zeroes the encoder. The mechanism should be at its home (min) position
     * when it turns
     */
    public void Initialize() {
        motor = opMode.hardwareMap.get(DcMotorEx.class, hardwareName);
        motor.setDirection(direction);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setTargetPositionTolerance(positionTolerance);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

//---------- Configuration ----------

    /**
     * Sets how close (in ticks) the motor must get to its target for RunToPosition to count as
     * arrived (IsBusy returns false). Can be called before or after initialize().
     */

    public void SetPositionTolerance(int ticks) {
        if (ticks < 0) {
            throw new IllegalArgumentException(
                    "BasePositionalMotor '" + hardwareName + "': tolerance cannot be negative");
        }
        positionTolerance = ticks;
        if (motor != null) {
            motor.setTargetPositionTolerance(ticks);
        }
    }

    /**
     * Enables stall protection: SetPower will cut power while the motor draws more than this
     * many amps. Pass 0 or less to turn it off. Can be called before or after initialize().
     */

    public void SetCurrentLimit(double amps) {
        currentLimitAmps = amps;
    }

    /**
     * Sets the power used by the Up/Down increment methods (0 to 1, default 0.5).
     */
    public void SetIncrementPower(double power) {
        incrementPower = Math.min(1.0, Math.abs(power));
    }


// ---------- Control ----------

    /**
     * Manual control. Power is limited to [-1, 1]. Power that would drive further past a
     * position limit is zeroed, and power is also zeroed if the current limit is exceeded.
     * Call this every loop so the protections stay active.
     */

    public void SetPower(double power) {
        checkInitialized();
        ensureMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = Math.max(-1.0, Math.min(1.0, power));

        int position = motor.getCurrentPosition();
        if (power > 0 && position >= maxPosition) {
            power = 0;
        } else if (power < 0 && position <= minPosition) {
            power = 0;
        }

        if (IsOverCurrent()) {
            power = 0;
        }

        motor.setPower(power);
    }

    public void Stop() {
        checkInitialized();
        motor.setPower(0);
        // Leave RUN_TO_POSITION so the old target can't be reused. The next
        // increment will then build on the motor's actual position.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Holds the motor at the position it is at right now. Unlike Stop(), the motor
     * actively resists being pushed away, using up to the given power.
     */
    public void HoldPosition(double power) {
        checkInitialized();
        RunToPosition(motor.getCurrentPosition(), power);
    }

    /** Holds the current position using the increment power (default 0.5). */
    public void HoldPosition() {
        HoldPosition(incrementPower);
    }

    /**
     * Runs to an encoder position at the given power. Target is limited to [min, max].
     */
    public void RunToPosition(int targetPosition, double power) {
        checkInitialized();
        targetPosition = Math.max(minPosition, Math.min(maxPosition, targetPosition));

        motor.setTargetPosition(targetPosition);
        ensureMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.min(1.0, Math.abs(power)));
    }

    public void SetToMinimum(double power) {
        RunToPosition(minPosition, power);
    }

    public void SetToMaximum(double power) {
        RunToPosition(maxPosition, power);
    }
//---------- Increment control ----------

    public void UpBigIncrement() {
        RunToPosition(getIncrementBase() + BIG_INCREMENT, incrementPower);
    }

    public void UpSmallIncrement() {
        RunToPosition(getIncrementBase() + SMALL_INCREMENT, incrementPower);
    }

    public void DownBigIncrement() {
        RunToPosition(getIncrementBase() - BIG_INCREMENT, incrementPower);
    }

    public void DownSmallIncrement() {
        RunToPosition(getIncrementBase() - SMALL_INCREMENT, incrementPower);
    }

    /**
     * Increments build on the current target if a position move is in progress (so rapid presses
     * accumulate), otherwise on the actual current position
     */

    private int getIncrementBase() {
        checkInitialized();
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return motor.getTargetPosition();
        }
        return motor.getCurrentPosition();
    }

//-------------Feedback---------------

    /**
     * True while a RunToPosition move is still in progress (outside the position tolerance).
     */

    public boolean IsBusy() {
        checkInitialized();
        return motor.isBusy();
    }

    public int GetPosition() {
        checkInitialized();
        return motor.getCurrentPosition();
    }

    public double GetPower() {
        checkInitialized();
        return motor.getPower();
    }

    /**
     * Current draw n amps.
     */
    public double GetCurrent() {
        checkInitialized();
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * True if a current limit is set and the motor is drawing more than that limit.
     */

    public boolean IsOverCurrent() {
        checkInitialized();
        return currentLimitAmps > 0 && motor.getCurrent(CurrentUnit.AMPS) > currentLimitAmps;
    }

    /**
     * Re-zeros the encoder at the current physical position and stops the motor.
     */
    public void ResetEncoder() {
        checkInitialized();
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

// -----------Internals ------------

    private void ensureMode(DcMotor.RunMode mode) {
        if (motor.getMode() != mode) {
            motor.setMode(mode);
        }
    }

    private void checkInitialized() {
        if (motor == null) {
            throw new IllegalStateException(
                    "BasePositionalMotor '" + hardwareName + "'used before Initialize() was called");
        }
    }
}