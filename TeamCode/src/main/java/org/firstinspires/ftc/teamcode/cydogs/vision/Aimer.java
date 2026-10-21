package org.firstinspires.ftc.teamcode.cydogs.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.components.Turret;

/**
 * Aimer — decides how the turret looks each loop.
 * Modes:
 *   HOLD   — do not move the turret
 *   PRESET — look at a saved angle (degrees)
 *   TRACK  — if we see a good tag, center on it; if not, hold still
 *   SEARCH — sweep until a tag is found, then center
 * powerSave (default false):
 *   false — camera stays on (good for Init camera stream / simpler testing)
 *   true  — camera on only for TRACK/SEARCH; off for HOLD/PRESET
 * Call update() once every loop.
 * Use isLocked() before shooting.
 * Hardware names live inside TagSight and Turret (Webcam 1, turret).
 */
public class Aimer {

    public enum Mode {
        HOLD,
        PRESET,
        TRACK,
        SEARCH
    }

    // How close to center counts as locked (degrees of bearing)
    private static final double LOCK_TOL_DEG = 3.0;

    // How hard to nudge toward the tag while tracking
    private static final double TRACK_GAIN = 0.15;

    // How far to move each loop while searching (turret degrees)
    private static final double SEARCH_STEP_DEG = 0.1;

    private final TagSight eyes;
    private final Turret turret;

    private Mode mode = Mode.HOLD;
    private double presetDegrees = 0.0;
    private double searchDirection = 1.0;

    /**
     * When true, camera is on only during TRACK/SEARCH.
     * When false (default), camera stays on so Init/stream and aiming are simple.
     */
    private boolean powerSave = false;

    /**
     * @param opMode    the running OpMode (needed for Turret)
     * @param alliance  RED, BLUE, or BOTH — passed through to TagSight
     */
    public Aimer(OpMode opMode, TagSight.Alliance alliance) {
        eyes = new TagSight(opMode.hardwareMap, alliance);
        turret = new Turret(opMode);
        turret.initialize();

        // Default: camera on (powerSave is false)
        applyCameraForMode();
    }

    /**
     * Turn power saving on or off.
     * true  = camera only for TRACK/SEARCH
     * false = camera stays on
     */
    public void setPowerSave(boolean enabled) {
        powerSave = enabled;
        applyCameraForMode();
    }

    public boolean isPowerSave() {
        return powerSave;
    }

    /** Do not move the turret. */
    public void hold() {
        mode = Mode.HOLD;
        applyCameraForMode();
    }

    /**
     * Look toward a known angle (turret degrees).
     * Good for auto: preset before driving to a shoot pose.
     */
    public void preset(double turretDegrees) {
        presetDegrees = turretDegrees;
        mode = Mode.PRESET;
        applyCameraForMode();
        turret.setTurretDegrees(presetDegrees);
    }

    /**
     * Follow a visible tag. If no tag, stay put (does not start a sweep).
     * Best default for match autos.
     */
    public void track() {
        mode = Mode.TRACK;
        applyCameraForMode();
    }

    /**
     * Sweep until a tag appears, then center on it (still SEARCH mode).
     * If the tag is lost, sweeping resumes.
     * Prefer preset + track in auto so you do not waste time scanning.
     */
    public void search() {
        mode = Mode.SEARCH;
        applyCameraForMode();
    }

    public Mode getMode() {
        return mode;
    }

    /**
     * Camera policy:
     *   powerSave false → always open
     *   powerSave true  → open only for TRACK and SEARCH
     */
    private void applyCameraForMode() {
        if (!powerSave) {
            eyes.openCamera();
            return;
        }

        if (mode == Mode.TRACK || mode == Mode.SEARCH) {
            eyes.openCamera();
        } else {
            eyes.stopCamera();
        }
    }

    /**
     * Run one step of the current mode.
     * Call this every loop of the OpMode (alongside follower.update() if driving).
     */
    public void update() {
        eyes.update();

        switch (mode) {
            case HOLD:
                // intentionally do nothing to the turret
                break;

            case PRESET:
                turret.setTurretDegrees(presetDegrees);
                break;

            case TRACK:
                if (eyes.hasTarget) {
                    if (Math.abs(eyes.bearingDegrees) > LOCK_TOL_DEG) {
                        // If it tracks the wrong way, negate bearing:
                        // turret.nudgeByDegrees(-eyes.bearingDegrees * TRACK_GAIN);
                        turret.nudgeByDegrees(eyes.bearingDegrees * TRACK_GAIN);
                    }
                }
                break;

            case SEARCH:
                if (eyes.hasTarget) {
                    if (Math.abs(eyes.bearingDegrees) > LOCK_TOL_DEG) {
                        turret.nudgeByDegrees(eyes.bearingDegrees * TRACK_GAIN);
                    }
                } else {
                    if (turret.isAtMax()) {
                        searchDirection = -1.0;
                    } else if (turret.isAtMin()) {
                        searchDirection = 1.0;
                    }
                    turret.nudgeByDegrees(searchDirection * SEARCH_STEP_DEG);
                }
                break;
        }
    }

    /**
     * True when we see a valid target and it is centered enough to shoot.
     */
    public boolean isLocked() {
        return eyes.hasTarget && Math.abs(eyes.bearingDegrees) <= LOCK_TOL_DEG;
    }

    public TagSight getEyes() {
        return eyes;
    }

    public Turret getTurret() {
        return turret;
    }

    /** Call when the OpMode ends to shut the camera down fully. */
    public void close() {
        eyes.close();
    }
}