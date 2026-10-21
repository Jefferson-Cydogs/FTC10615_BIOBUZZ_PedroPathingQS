package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

/**
 * Turret — aims left/right for shooting / camera.
 * Robot angles (degrees):
 *   0°   = intake ("front")
 *   180° = straight ahead for aiming (servo 0.5)
 *   We allow about 75° left and 75° right of straight.
 * Servo positions we allow:
 *   0.3 = max one way
 *   0.5 = straight
 *   0.7 = max the other way
 * Call initialize() once, then setTurretDegrees() or nudgeByDegrees().
 */
public class Turret extends BaseServo {

    private static final String TURRET_SERVO_NAME = "TurretTurnerServo";

    // ----- Aim angles (degrees on the robot) -----

    /** Straight ahead for aiming (away from intake) */
    public static final double STRAIGHT_DEGREES = 180.0;

    /** How far left or right of straight we allow (degrees) */
    public static final double MAX_OFFSET_DEG = 50.0;

    /** Farthest aim one direction */
    public static final double MIN_DEGREES = STRAIGHT_DEGREES - MAX_OFFSET_DEG; // 105

    /** Farthest aim the other direction */
    public static final double MAX_DEGREES = STRAIGHT_DEGREES + MAX_OFFSET_DEG; // 255

    // ----- Servo positions (safe range from testing) -----

    private static final double MIN_POSITION = 0.35;
    private static final double STRAIGHT_POSITION = 0.5;
    private static final double MAX_POSITION = 0.65;

    /** Last angle we commanded */
    private double currentDegrees;

    /**
     * @param opMode the running OpMode
     */
    public Turret(OpMode opMode) {
        super(
                opMode,
                TURRET_SERVO_NAME,
                Servo.Direction.FORWARD,
                MIN_POSITION,
                MAX_POSITION,
                STRAIGHT_POSITION
        );
        currentDegrees = STRAIGHT_DEGREES;
    }

    /**
     * Hook up the servo and aim straight.
     * Call once during OpMode init.
     */
    public void initialize() {
        Initialize();
        setTurretDegrees(STRAIGHT_DEGREES);
    }

    /**
     * Aim to a robot angle in degrees (clamped to the allowed range).
     * Example: setTurretDegrees(180) aims straight.
     */
    public void setTurretDegrees(double degrees) {
        currentDegrees = clampDegrees(degrees);
        SetPosition(degreesToPosition(currentDegrees));
    }

    /**
     * Change aim by a small amount from where we are now.
     * Used by Aimer when tracking a tag (bearing nudge).
     */
    public void nudgeByDegrees(double deltaDegrees) {
        setTurretDegrees(currentDegrees + deltaDegrees);
    }

    public double getTurretDegrees() {
        return currentDegrees;
    }

    public boolean isAtMin() {
        return currentDegrees <= MIN_DEGREES + 0.5;
    }

    public boolean isAtMax() {
        return currentDegrees >= MAX_DEGREES - 0.5;
    }

    /**
     * Map robot degrees → servo position.
     *
     *   105° → 0.3
     *   180° → 0.5
     *   255° → 0.7
     *
     * One straight line through those points (symmetric around center).
     */
    private double degreesToPosition(double degrees) {
        // How far from straight, as a fraction of max offset (-1 .. +1)
        double t = (degrees - STRAIGHT_DEGREES) / MAX_OFFSET_DEG;
        return STRAIGHT_POSITION + t * (MAX_POSITION - STRAIGHT_POSITION);
    }

    private double clampDegrees(double degrees) {
        if (degrees < MIN_DEGREES) {
            return MIN_DEGREES;
        }
        if (degrees > MAX_DEGREES) {
            return MAX_DEGREES;
        }
        return degrees;
    }
}