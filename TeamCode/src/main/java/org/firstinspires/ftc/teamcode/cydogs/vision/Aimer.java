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
        SEARCH,
        SEARCH2
    }

    // How close to center counts as locked (degrees of bearing)
    private static final double LOCK_TOL_DEG = 3.0;

    // How hard to nudge toward the tag while tracking
    private static final double TRACK_GAIN = 0.15;

    // How far to move each loop while searching (turret degrees)
    private static final double SEARCH_STEP_DEG = 0.3;

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

    // =====================================================================================
    // SEARCH2 for Aimer.java — paste-in blocks. Your existing code and comments stay as is.
    //
    // Idea: a tag sighting is an EVENT, not a state.
    //   SWEEP  → see a tag → PAUSE (hold still ~150 ms so the camera catches up)
    //   PAUSE  → AIM once at (turret angle + bearing), not a stream of small nudges
    //   VERIFY → ignore frames while the servo moves / stale frames arrive, then look:
    //              centered  → LOCKED
    //              off center→ aim again (and flip the sign if the last aim made it worse)
    //              not seen  → LOCAL search around where we were aiming
    //   LOCAL  → try a few small offsets either side, then give up and SWEEP again
    //   LOCKED → latched. A single missed frame does NOT unlock; ~300 ms with no tag does.
    // =====================================================================================

    // ---- SEARCH2 tuning ----
    /** Hold still this long after first seeing a tag (lets stale frames flush out). */
    private static final long S2_PAUSE_MS = 150;
    /** After any turret move, ignore the camera this long (servo travel + stale frames). */
    private static final long S2_SETTLE_MS = 200;
    /** After settling, how long to look for the tag before trying the local search. */
    private static final long S2_VERIFY_MS = 400;
    /** How many times we re-aim from a new bearing before switching to the local search. */
    private static final int S2_MAX_AIM_TRIES = 3;
    /** Tag missing this long while LOCKED = unlock and recover. */
    private static final long S2_UNLOCK_AFTER_MS = 300;
    /** Gentle correction while LOCKED (much smaller than TRACK_GAIN). */
    private static final double S2_LOCKED_GAIN = 0.05;
    /** While LOCKED, correct at most this often (bearing only updates once per camera frame). */
    private static final long S2_NUDGE_INTERVAL_MS = 100;
    /** Local search offsets (turret degrees) around where we last aimed. */
    private static final double[] S2_LOCAL_OFFSETS = {4, -4, 8, -8};
    /** Dwell at each local offset. Must be longer than S2_SETTLE_MS. */
    private static final long S2_LOCAL_DWELL_MS = 350;

    private enum Search2State { SWEEP, PAUSE, VERIFY, LOCAL, LOCKED }

    private Search2State s2State = Search2State.SWEEP;
    private long s2StateStartMs = 0;
    /** +1 matches the existing nudge direction. Flips itself if aiming makes the bearing worse. */
    private double s2Sign = 1.0;
    private double s2LastBearing = 0;
    private double s2BearingBeforeAim = 0;
    private int s2AimTries = 0;
    private long s2LastSeenMs = 0;
    private long s2LastNudgeMs = 0;
    private int s2LocalIndex = 0;
    private double s2LocalCenter = 0;

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

    /**
     * Step-and-stare search for a camera that does not always see the tag.
     * Call isLocked() to know when it has a latched lock.
     */
    public void search2() {
        mode = Mode.SEARCH2;
        applyCameraForMode();
        s2Enter(Search2State.SWEEP);
    }

    /** For telemetry: SWEEP / PAUSE / VERIFY / LOCAL / LOCKED. */
    public String getSearch2State() {
        return s2State.toString();
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

            case SEARCH2:
                updateSearch2();
                break;
        }
    }

    /**
     * True when we see a valid target and it is centered enough to shoot.
     */

    public boolean isLocked() {
        if (mode == Mode.SEARCH2) {
            return s2State == Search2State.LOCKED;   // latched: one missed frame does not unlock
        }
        return eyes.hasTarget && Math.abs(eyes.bearingDegrees) <= LOCK_TOL_DEG;
    }

    public TagSight getEyes() {
        return eyes;
    }

    public Turret getTurret() {
        return turret;
    }

    public boolean isTagVisible() {
        return eyes.hasTarget;
    }

    private void s2Enter(Search2State next) {
        s2State = next;
        s2StateStartMs = System.currentTimeMillis();
    }

    private void s2EnterLocked(long now) {
        s2Enter(Search2State.LOCKED);
        s2LastSeenMs = now;
    }

    /** One move to where the last good bearing says the tag is, then wait for it to settle. */
    private void s2AimFromBearing() {
        double target = turret.getTurretDegrees() + s2Sign * s2LastBearing;
        turret.setTurretDegrees(target);
        s2Enter(Search2State.VERIFY);
    }

    /** Try small offsets around where we are now. */
    private void s2StartLocal() {
        s2LocalCenter = turret.getTurretDegrees();
        s2LocalIndex = 0;
        turret.setTurretDegrees(s2LocalCenter + S2_LOCAL_OFFSETS[0]);
        s2Enter(Search2State.LOCAL);
    }

    private void updateSearch2() {
        long now = System.currentTimeMillis();
        long inState = now - s2StateStartMs;

        switch (s2State) {

            case SWEEP:
                if (eyes.hasTarget) {
                    s2Enter(Search2State.PAUSE);
                    s2LastBearing = eyes.bearingDegrees;
                } else {
                    if (turret.isAtMax()) {
                        searchDirection = -1.0;
                    } else if (turret.isAtMin()) {
                        searchDirection = 1.0;
                    }
                    turret.nudgeByDegrees(searchDirection * SEARCH_STEP_DEG);
                }
                break;

            case PAUSE:
                // Turret is not moving. Keep the newest bearing; frames from while it was
                // still moving go stale during this pause.
                if (eyes.hasTarget) {
                    s2LastBearing = eyes.bearingDegrees;
                }
                if (inState >= S2_PAUSE_MS) {
                    s2AimTries = 0;
                    s2BearingBeforeAim = s2LastBearing;
                    s2AimFromBearing();
                }
                break;

            case VERIFY:
                if (inState < S2_SETTLE_MS) {
                    break;   // servo still moving / stale frames: do not trust the camera yet
                }
                if (eyes.hasTarget) {
                    s2LastBearing = eyes.bearingDegrees;
                    if (Math.abs(s2LastBearing) <= LOCK_TOL_DEG) {
                        s2EnterLocked(now);
                    } else if (s2AimTries >= S2_MAX_AIM_TRIES) {
                        s2StartLocal();
                    } else {
                        // Last aim made it clearly worse = wrong sign. Flip it.
                        if (Math.abs(s2LastBearing) > Math.abs(s2BearingBeforeAim) * 1.2) {
                            s2Sign = -s2Sign;
                        }
                        s2AimTries++;
                        s2BearingBeforeAim = s2LastBearing;
                        s2AimFromBearing();
                    }
                } else if (inState >= S2_SETTLE_MS + S2_VERIFY_MS) {
                    s2StartLocal();   // never saw it after aiming
                }
                break;

            case LOCAL:
                if (eyes.hasTarget && inState >= S2_SETTLE_MS) {
                    s2LastBearing = eyes.bearingDegrees;
                    s2BearingBeforeAim = s2LastBearing;
                    s2AimTries = 0;
                    if (Math.abs(s2LastBearing) <= LOCK_TOL_DEG) {
                        s2EnterLocked(now);
                    } else {
                        s2AimFromBearing();
                    }
                } else if (inState >= S2_LOCAL_DWELL_MS) {
                    s2LocalIndex++;
                    if (s2LocalIndex >= S2_LOCAL_OFFSETS.length) {
                        s2Enter(Search2State.SWEEP);   // give up locally, resume the sweep
                    } else {
                        turret.setTurretDegrees(s2LocalCenter + S2_LOCAL_OFFSETS[s2LocalIndex]);
                        s2Enter(Search2State.LOCAL);
                    }
                }
                break;

            case LOCKED:
                if (eyes.hasTarget) {
                    s2LastSeenMs = now;
                    if (Math.abs(eyes.bearingDegrees) > LOCK_TOL_DEG
                            && now - s2LastNudgeMs >= S2_NUDGE_INTERVAL_MS) {
                        turret.nudgeByDegrees(s2Sign * eyes.bearingDegrees * S2_LOCKED_GAIN);
                        s2LastNudgeMs = now;
                    }
                } else if (now - s2LastSeenMs >= S2_UNLOCK_AFTER_MS) {
                    s2StartLocal();   // lost it for a while: recover near where we were
                }
                break;
        }
    }

    /** Call when the OpMode ends to shut the camera down fully. */
    public void close() {
        eyes.close();
    }
}