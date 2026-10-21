package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseServo {

    private static final double BIG_INCREMENT = 0.1;
    private static final double SMALL_INCREMENT = 0.01;

    private final OpMode opMode;
    private final String hardwareName;
    private final Servo.Direction direction;
    private final double min;
    private final double max;
    private final double startingPosition;

    private Servo servo;
    private double currentPosition;

    public BaseServo(OpMode opMode,
                     String hardwareName,
                     Servo.Direction direction,
                     double min,
                     double max,
                     double startingPosition) {

        if (min > max) {
            throw new IllegalArgumentException(
                    "BaseServo '" + hardwareName + "': min (" + min + ") cannot be greater than max (" + max + ")");
        }

        this.opMode = opMode;
        this.hardwareName = hardwareName;
        this.direction = direction;
        this.min = min;
        this.max = max;
        this.startingPosition = clamp(startingPosition);
        this.currentPosition = this.startingPosition;
    }

    /** Gets the servo from the hardware map and applies direction. Call once before using the servo. */
    public void Initialize() {
        servo = opMode.hardwareMap.get(Servo.class, hardwareName);
        servo.setDirection(direction);
    }

    public void SetStartingPosition() {
        SetPosition(startingPosition);
    }

    public void SetToMinimum() {
        SetPosition(min);
    }

    public void SetToMaximum() {
        SetPosition(max);
    }

    /** Sets the servo position, limited to the range [min, max]. */
    public void SetPosition(double position) {
        checkInitialized();
        currentPosition = clamp(position);
        servo.setPosition(currentPosition);
    }

    public void IncrementPosition(double amount) {
        SetPosition(currentPosition + amount);
    }

    public void DecrementPosition(double amount) {
        SetPosition(currentPosition - amount);
    }

    public void UpBigIncrement() {
        SetPosition(currentPosition + BIG_INCREMENT);
    }

    public void UpSmallIncrement() {
        SetPosition(currentPosition + SMALL_INCREMENT);
    }

    public void DownBigIncrement() {
        SetPosition(currentPosition - BIG_INCREMENT);
    }

    public void DownSmallIncrement() {
        SetPosition(currentPosition - SMALL_INCREMENT);
    }

    /** Returns the last commanded position (not a sensed position). */
    public double GetPosition() {
        return currentPosition;
    }

    private double clamp(double position) {
        return Math.max(min, Math.min(max, position));
    }

    private void checkInitialized() {
        if (servo == null) {
            throw new IllegalStateException(
                    "BaseServo '" + hardwareName + "' used before Initialize() was called");
        }
    }
}