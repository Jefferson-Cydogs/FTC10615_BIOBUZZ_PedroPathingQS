package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

/**
 * Three-bar dump arm built from three BaseServo joints (all positional servos):
 *   lower_flower_servo  = base joint  (bottom bar)
 *   medium_flower_servo = elbow joint (middle bar)
 *   upper_flower_servo  = wrist joint (top bar, carries the channel)
 *
 * Positions (from the sketch):
 *   1 = REST: folded flat (Z shape), all bars roughly horizontal
 *   2 = DUMP: raised; top bar stays nearly level and tips ~10 deg downward to dump.
 *
 * Angle convention for every joint: 0 deg = folded/down, angle grows as the arm unfolds/raises.
 *   Lower  : angle of the first bar above horizontal
 *   Medium : interior angle between bar 1 and bar 2
 *   Upper  : interior angle between bar 2 and bar 3 (the mechanism bar)
 * Where 0 deg sits on each servo is set by ZERO_POSITIONS below.
 *
 * Usage in the new TeleOp:
 *   DumpArm Arm = new DumpArm(this);
 *   Arm.Initialize();                       // in initializeDevices()
 *   Arm.SnapToRest();                       // in initializePositions()
 *   if (gamepad2.dpadUpWasPressed())   Arm.MoveToDump();
 *   if (gamepad2.dpadDownWasPressed()) Arm.MoveToRest();
 *   Arm.Update();                           // once per loop, every loop
 */
public class DumpArm {

    // ---- Hardware names (must match the robot configuration) ----
    private static final String LOWER_NAME  = "lowerflower";
    private static final String MEDIUM_NAME = "midflower";
    private static final String UPPER_NAME  = "upperflower";

    // ---- Per-servo direction ----
    // If a joint unfolds the wrong way, switch it to REVERSE. Note that with REVERSE the angle
    // also counts the other way from that joint's zero position, so re-tune that joint's
    // ZERO_POSITIONS entry (and the angles if needed).
    private static final Servo.Direction LOWER_DIRECTION  = Servo.Direction.FORWARD;
    private static final Servo.Direction MEDIUM_DIRECTION = Servo.Direction.FORWARD;
    private static final Servo.Direction UPPER_DIRECTION  = Servo.Direction.FORWARD;

    // ---- Servo settings ----
    // Degrees of travel for position 0.0 -> 1.0. Standard servos = 180; use 270/300 for longer-range servos.
    private static final double SERVO_RANGE_DEG = 180.0;
    // BaseServo limits each servo to this range (end-stop safety). This does NOT set where 0 deg is.
    private static final double MIN_POSITION = 0.02;
    private static final double MAX_POSITION = 0.98;

    // ---- Where 0 degrees sits on each servo (servo position 0.0 - 1.0): {Lower, Medium, Upper} ----
    // Position = ZERO_POSITION + angle / SERVO_RANGE_DEG
    private static final double[] ZERO_POSITIONS = {0.2, 0.2, 0.2};

    // ---- Joint angles in degrees: {Lower, Medium, Upper} ----
    // Estimated from the sketch. Real values depend on how the servo horns are mounted, so expect to tune.
    private static final double[] REST_ANGLES = {0, 35, 10};
    private static final double[] DUMP_ANGLES = {80, 110, 30};

    // ---- Movement time (seconds for a full move). Increase for a slower, gentler move. ----
    private static final double RAISE_SECONDS = 1.5;
    private static final double LOWER_SECONDS = 1.5;

    private static final int LOWER = 0;
    private static final int MEDIUM = 1;
    private static final int UPPER = 2;

    private final BaseServo[] servos = new BaseServo[3];
    private final double[] startPositions = new double[3];
    private final double[] targetPositions = new double[3];
    private final ElapsedTime moveTimer = new ElapsedTime();
    private double moveSeconds = 0;
    private boolean moving = false;

    public DumpArm(OpMode opMode)
    {
        // Each servo's starting position is the rest position (the arm starts the match folded)
        servos[LOWER] = new BaseServo(opMode, LOWER_NAME, LOWER_DIRECTION,
                MIN_POSITION, MAX_POSITION, AngleToPosition(LOWER, REST_ANGLES[LOWER]));
        servos[MEDIUM] = new BaseServo(opMode, MEDIUM_NAME, MEDIUM_DIRECTION,
                MIN_POSITION, MAX_POSITION, AngleToPosition(MEDIUM, REST_ANGLES[MEDIUM]));
        servos[UPPER] = new BaseServo(opMode, UPPER_NAME, UPPER_DIRECTION,
                MIN_POSITION, MAX_POSITION, AngleToPosition(UPPER, REST_ANGLES[UPPER]));
    }

    /** Gets the servos from the hardware map. Call once before using the arm. */
    public void Initialize()
    {
        for (BaseServo servo : servos) {
            servo.Initialize();
        }
    }

    /** Jump straight to the rest position (use at init, when the arm is already folded). */
    public void SnapToRest()
    {
        moving = false;
        for (BaseServo servo : servos) {
            servo.SetStartingPosition();
        }
    }

    /** Start a smooth move to the elevated dump position. Keep calling Update() every loop. */
    public void MoveToDump()
    {
        StartMove(DUMP_ANGLES, RAISE_SECONDS);
    }

    /** Start a smooth move back to the folded rest position. Keep calling Update() every loop. */
    public void MoveToRest()
    {
        StartMove(REST_ANGLES, LOWER_SECONDS);
    }

    /**
     * Call once per loop. All three servos move together and arrive at the same time,
     * easing in and out so the arm doesn't jerk and spill the channel.
     */
    public void Update()
    {
        if (!moving) return;

        double t = Range.clip(moveTimer.seconds() / moveSeconds, 0.0, 1.0);
        double eased = t * t * (3.0 - 2.0 * t); // smoothstep: slow start, slow finish

        for (int i = 0; i < 3; i++) {
            servos[i].SetPosition(startPositions[i] + (targetPositions[i] - startPositions[i]) * eased);
        }

        if (t >= 1.0) {
            moving = false;
        }
    }

    public boolean IsMoving()
    {
        return moving;
    }

    private void StartMove(double[] targetAngles, double seconds)
    {
        // Start from each servo's last commanded position, so reversing mid-move is smooth
        for (int i = 0; i < 3; i++) {
            startPositions[i] = servos[i].GetPosition();
            targetPositions[i] = AngleToPosition(i, targetAngles[i]);
        }
        moveSeconds = seconds;
        moveTimer.reset();
        moving = true;
    }

    /** Converts a joint angle (degrees) to a servo position, measured from that joint's zero position. */
    private static double AngleToPosition(int joint, double angleDegrees)
    {
        return Range.clip(ZERO_POSITIONS[joint] + angleDegrees / SERVO_RANGE_DEG,
                MIN_POSITION, MAX_POSITION);
    }
}