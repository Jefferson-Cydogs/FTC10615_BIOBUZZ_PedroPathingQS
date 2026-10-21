package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

/**
 * FlowerScoringServo - the gate that opens to let the scoring elements drop into the flower,
 * and closes again to hold them in.
 *
 * A FlowerScoringServo IS a BaseServo, so it inherits Initialize, SetPosition and the rest.
 * This class only adds the settings for this servo and the two methods Open and Close.
 *
 * How an OpMode uses it:
 *   1. Make one:            new FlowerScoringServo(this)
 *   2. Call Initialize()    once (inherited). This connects to the servo but does not move it.
 *   3. Call Close()         right after, so the gate starts the match shut.
 *   4. Call Open() when the driver wants to dump, and Close() when done.
 */
public class FlowerScoringServo extends BaseServo {

    // Must match the servo's name in the Robot Configuration on the Driver Hub.
    private static final String HARDWARE_NAME = "FlowerScoringServo";

    // Which way the servo turns. Use REVERSE if open and close come out swapped on the robot.
    private static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;

    // Safety limits: BaseServo never lets a position go outside this range.
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;

    // The two gate positions, measured on the robot. Both must be between MIN and MAX.
    private static final double CLOSED_POSITION = 0.5;
    private static final double OPEN_POSITION = 0.8;

    /** The closed position is also the starting position, so the gate's home is shut. */
    public FlowerScoringServo(OpMode opMode) {
        super(opMode, HARDWARE_NAME, DIRECTION, MIN_POSITION, MAX_POSITION, CLOSED_POSITION);
    }

    /** Swings the gate open so the scoring elements can drop out. */
    public void Open() {
        SetPosition(OPEN_POSITION);
    }

    /** Swings the gate shut to hold the scoring elements in. */
    public void Close() {
        SetPosition(CLOSED_POSITION);
    }
}