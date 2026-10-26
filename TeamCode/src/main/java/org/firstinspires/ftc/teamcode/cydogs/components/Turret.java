package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

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

    /** Last angle we commanded */
    private double currentDegrees;

    /**
     * @param opMode the running OpMode
     */
    public Turret(OpMode opMode) {
        super(
                opMode,
                BeethovenHardware.Turret.NAME,
                BeethovenHardware.Turret.DIRECTION,
                BeethovenHardware.Turret.MIN_POSITION,
                BeethovenHardware.Turret.MAX_POSITION,
                BeethovenHardware.Turret.STRAIGHT_POSITION
        );
        currentDegrees = BeethovenHardware.Turret.STRAIGHT_DEGREES;
    }

    /**
     * Hook up the servo and aim straight.
     * Call once during OpMode init.
     */
    public void initialize() {
        Initialize();
        setTurretDegrees(BeethovenHardware.Turret.STRAIGHT_DEGREES);
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
        return currentDegrees <= BeethovenHardware.Turret.MIN_DEGREES + 0.5;
    }

    public boolean isAtMax() {
        return currentDegrees >= BeethovenHardware.Turret.MAX_DEGREES - 0.5;
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
        double t = (degrees - BeethovenHardware.Turret.STRAIGHT_DEGREES) / BeethovenHardware.Turret.MAX_OFFSET_DEGREES;
        return BeethovenHardware.Turret.STRAIGHT_POSITION + t * (BeethovenHardware.Turret.MAX_POSITION - BeethovenHardware.Turret.STRAIGHT_POSITION);
    }

    private double clampDegrees(double degrees) {
        if (degrees < BeethovenHardware.Turret.MIN_DEGREES) {
            return BeethovenHardware.Turret.MIN_DEGREES;
        }
        if (degrees > BeethovenHardware.Turret.MAX_DEGREES) {
            return BeethovenHardware.Turret.MAX_DEGREES;
        }
        return degrees;
    }
}