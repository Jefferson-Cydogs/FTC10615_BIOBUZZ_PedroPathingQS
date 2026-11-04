package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;
import org.firstinspires.ftc.teamcode.cydogs.components.Intake;
import org.firstinspires.ftc.teamcode.cydogs.components.Launcher;
import org.firstinspires.ftc.teamcode.cydogs.components.LauncherSizeServo;
import org.firstinspires.ftc.teamcode.cydogs.components.LoaderServo;
import org.firstinspires.ftc.teamcode.cydogs.components.MoveToLauncherMotor;
import org.firstinspires.ftc.teamcode.cydogs.vision.Aimer;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;
import org.firstinspires.ftc.teamcode.cydogs.components.FlingyMag;
import org.firstinspires.ftc.teamcode.cydogs.components.FlowerRampServo;

/**
 * Shared autonomous skills for every Course.
 *
 * Hardware names and motor powers come from BeethovenHardware.
 * This class only builds Ivy command sequences.
 *
 * OpMode use:
 *   skills = new AutoSkills(this);
 *   skills.initialize();
 *   // every loop:
 *   skills.update();
 *   // in autoRoutine:
 *   skills.shootFourPollen(), skills.intakeGarden(), skills.intakeFlower()
 */
public class AutoSkills {

    // --- Auto-only timings (strategy, not device wiring) ---
    // Move these into BeethovenHardware.Auto later if you want one constants file.
    private static final long SPIN_UP_MS = 10000;
    private static final long BETWEEN_SHOTS_MS = 500;
    private static final long FEED_MS = 200; //changed from 350
    private static final long INTAKE_SHOT_MS = 150;   // short intake between pops
    private static final long INTAKE_MICRO_MS = 10;   // short intake between pops
    private static final long GARDEN_INTAKE_MS = 2000;
    private static final long FLOWER_INTAKE_MS = 2000;
    /** How long feed + intake run together (tune this). */
    private static final long FEED_AND_INTAKE_MS = 200;
    /** Wait long enough for LoaderServo.Update() to finish LaunchOnce. */
    private static final long POP_MS = (long) (BeethovenHardware.Loader.LAUNCH_HOLD_SECONDS * 1000.0) + 100;
    /** How long to watch after a preset aim (ms). Lock = no tip. */
    private static final long TIP_WATCH_MS = 2000;
    /** How long to SEARCH full sweep when no preset (ms). Tune so turret can cover range. */
    private static final long TIP_SEARCH_MS = 6000;
    private static final long FEED_TIMEOUT_MS = 1500;
    private static final long INTAKE_TO_MID_TIMEOUT_MS = 800;  // keep short so ball 4 doesn’t overshoot
    private static final long SHOOT_CLEAR_TIMEOUT_MS = 800;
    /** true = no lock after looking (tipped); false = locked on basket (no tip) */
    private static final long RAMP_MOVE_MS = 400;

    private static final long EMPTY_MAGAZINE_MS = 1200;

    private static final long TIP_SUCCESS_MIN_WATCH_MS = 250;

    /** The tag must be seen over at least this long to count as "partner tipped" (rejects one-frame blips). */
    private static final long PARTNER_TIP_CONFIRM_MS = 200;
    /** A gap in sightings longer than this restarts the confirm (short flickers are forgiven). */
    private static final long PARTNER_TIP_MAX_GAP_MS = 150;
    private boolean hiveTipped = true;
    private boolean keepLauncherRunning = false;

    private final OpMode opMode;

    private LauncherSizeServo launcherSize;
    private Launcher launcher;
    private LoaderServo loader;
    private MoveToLauncherMotor moveToLauncher;
    private Intake intake;
    private Aimer aimer;
    private FlingyMag flingyMag;
    private FlowerRampServo flowerRamp;

    public AutoSkills(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Create and init mechanisms. Call once before waitForStart(). */
    public void initialize() {
        launcherSize = new LauncherSizeServo(opMode);
        launcher = new Launcher(opMode);
        loader = new LoaderServo(opMode);
        moveToLauncher = new MoveToLauncherMotor(opMode);
        intake = new Intake(opMode);
        flingyMag = new FlingyMag(opMode);
        flowerRamp = new FlowerRampServo(opMode);

        launcherSize.Initialize();
        launcherSize.SetPollen();
        launcher.Initialize();
        loader.Initialize();
        moveToLauncher.Initialize();
        intake.Initialize();
        flowerRamp.Initialize();   // if BaseServo uses Initialize() like your other servos
        flowerRamp.MoveUp();       // start raised for auto
    }

    /** Call every OpMode loop. */
    public void update() {
        if (launcher != null) {
            launcher.Update();
        }
        if (loader != null) {
            loader.Update();
        }
        if (aimer != null) {
            aimer.update();
        }
    }

    // ----- private building blocks -----

    private void ensureAimer(TagSight.Alliance alliance) {
        if (aimer == null) {
            aimer = new Aimer(opMode, alliance);
        }
    }

    /** Wait until flywheel is ready, or give up after timeout. */
    private Command waitLauncherReady() {
        final long[] startMs = {0L};
        return Command.build()
                .setStart(() -> startMs[0] = System.currentTimeMillis())
                .setDone(() ->
                        launcher.IsLauncherAtSpeed()
                                || System.currentTimeMillis() - startMs[0] >= SPIN_UP_MS
                );
    }

    private Command popOnce() {
        return sequential(
                instant(() -> loader.LaunchOnce()),
                waitMs(POP_MS)
        );
    }

    private Command feedOnce() {
        return sequential(
                instant(() -> moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER)),
                waitMs(FEED_MS),
                instant(() -> moveToLauncher.Stop())
        );
    }

    private Command intakeShort() {
        return sequential(
                instant(() -> intake.RunIntake()),
                waitMs(INTAKE_SHOT_MS),
                instant(() -> intake.StopIntake())
        );
    }

    private Command intakeMicro() {
        return sequential(
                instant(() -> intake.RunIntake()),
                waitMs(INTAKE_MICRO_MS),
                instant(() -> intake.StopIntake())
        );
    }

    /**
     * Run move-to-launcher and intake at the same time, then stop both.
     * Helps keep balls 3 and 4 from jamming.
     */
    private Command feedAndIntakeOnce() {
        return sequential(
                instant(() -> {
                    moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
                    intake.RunIntake();
                }),
                waitMs(FEED_AND_INTAKE_MS),
                instant(() -> {
                    moveToLauncher.Stop();
                    intake.StopIntake();
                })
        );
    }

    private Command stopShootingHardware() {
        return instant(() -> {
            intake.StopIntake();
            moveToLauncher.Stop();
            if (!keepLauncherRunning) {
                launcher.StopLauncher();
            }
        });
    }

    /** Wait until shoot slot is EMPTY (safe to run feeder), or timeout. */
    private Command waitUntilShootClear() {
        final long[] start = {0L};
        return Command.build()
                .setStart(() -> start[0] = System.currentTimeMillis())
                .setDone(() ->
                        !flingyMag.IsBallReadyForLaunch()
                                || System.currentTimeMillis() - start[0] >= SHOOT_CLEAR_TIMEOUT_MS
                );
    }

    /** Wait until a ball is at the launcher, or timeout. */
    private Command waitUntilReadyForLaunch() {
        final long[] start = {0L};
        return Command.build()
                .setStart(() -> start[0] = System.currentTimeMillis())
                .setDone(() ->
                        flingyMag.IsBallReadyForLaunch()
                                || System.currentTimeMillis() - start[0] >= FEED_TIMEOUT_MS
                );
    }

    /** Wait until intake L/R sees a ball, or short timeout. */
    private Command waitUntilBallInMid() {
        final long[] start = {0L};
        return Command.build()
                .setStart(() -> start[0] = System.currentTimeMillis())
                .setDone(() ->
                        flingyMag.IsBallInMidPosition()
                                || System.currentTimeMillis() - start[0] >= INTAKE_TO_MID_TIMEOUT_MS
                );
    }

    /** Sensor path: fire only; clear is handled before feed. */
    private Command popNow() {
        return instant(() -> loader.LaunchOnce());
    }

    /**
     * Do not run feeder while a ball is in the shoot slot.
     * Then feed until a ball is ready (or timeout).
     */
    private Command feedUntilReady() {
        return sequential(
                waitUntilShootClear(),
                instant(() -> moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER)),
                waitUntilReadyForLaunch(),
                instant(() -> moveToLauncher.Stop())
        );
    }

    /** Intake until mid sensors see a ball, then stop (limits ball 4 travel). */
    private Command intakeUntilMid() {
        return sequential(
                instant(() -> intake.RunIntake()),
                waitUntilBallInMid(),
                instant(() -> intake.StopIntake())
        );
    }

    /** Intake + feeder together until ball at launcher (ball 4). */
    private Command intakeAndFeedUntilReady() {
        return sequential(
                waitUntilShootClear(),
                instant(() -> {
                    intake.RunIntake();
                    moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
                }),
                waitUntilReadyForLaunch(),
                instant(() -> {
                    intake.StopIntake();
                    moveToLauncher.Stop();
                })
        );
    }


    // ----- skills for courses -----

    /**
     * Start launcher, fire up to four pollen balls, then stop.
     * Sequence: (pop → feed → short intake) × 3, then final pop.
     */
    public Command shootFourPollen_old() {
        return sequential(
                instant(() -> launcher.StartLauncher()),
                waitLauncherReady(),

                popOnce(), feedOnce(), intakeShort(), waitMs(BETWEEN_SHOTS_MS),
                popOnce(), feedOnce(), intakeShort(), waitMs(BETWEEN_SHOTS_MS),
                popOnce(), feedOnce(), intakeShort(), waitMs(BETWEEN_SHOTS_MS),
                popOnce(),

                stopShootingHardware()
        );
    }

    public Command shootFourPollen2() {
        return sequential(
                instant(() -> launcher.StartLauncher()),
                waitLauncherReady(),

                // Ball 1 — already at the popper
                popOnce(),
                feedOnce(),
                intakeShort(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 2
                popOnce(),
                feedAndIntakeOnce(),   // feeder + intake together
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 3
                popOnce(),
                feedAndIntakeOnce(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 4
                popOnce(),

                stopShootingHardware()
        );
    }

    public Command shootFourPollen() {
        return sequential(
                instant(() -> launcher.StartLauncher()),
                waitLauncherReady(),
                waitMs(250),

                // Ball 1 — already at the popper
                popOnce(),
                intakeMicro(),
                feedOnce(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 2
                popOnce(),
                intakeMicro(),
                feedOnce(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 3
                popOnce(),
                feedAndIntakeOnce(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 4
                popOnce(),

                stopShootingHardware()
        );
    }

    /**
     * Four pollen using FlingyMag sensors.
     * Queue at start: shoot=1, feeder=2, intake=3 then 4.
     * Rule: never run feeder while IsBallReadyForLaunch() is true.
     */
    public Command shootFourPollenSensor() {
        return sequential(
                instant(() -> launcher.StartLauncher()),
                waitLauncherReady(),

                // Ball 1 — already at shoot
                popNow(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 2 — under feeder
                feedUntilReady(),
                popNow(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 3 — short intake to mid, then feed
                intakeUntilMid(),
                feedUntilReady(),
                popNow(),
                waitMs(BETWEEN_SHOTS_MS),

                // Ball 4
                intakeAndFeedUntilReady(),
                popNow(),

                stopShootingHardware()
        );
    }

    /** Long intake at the garden. */
    public Command intakeGarden() {
        return sequential(
                instant(() -> intake.RunIntake()),
                waitMs(GARDEN_INTAKE_MS),
                instant(() -> intake.StopIntake())
        );
    }

    /** Long intake at a flower. */
    public Command intakeFlower() {
        return sequential(
                instant(() -> intake.RunIntake()),
                waitMs(FLOWER_INTAKE_MS),
                instant(() -> intake.StopIntake())
        );
    }

    /** Turn intake on (stays on until stopIntake). */
    public Command startIntake() {
        return instant(() -> intake.RunIntake());
    }

    /** Turn intake off. */
    public Command stopIntake() {
        return instant(() -> intake.StopIntake());
    }

    /**
     * Garden intake while also running the move-to-launcher belt
     * so balls advance and make room (uses BeethovenHardware power).
     */
    public Command intakeGardenAndFeed() {
        return sequential(
                instant(() -> {
                    intake.RunIntake();
                    moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
                }),
                waitMs(GARDEN_INTAKE_MS),
                instant(() -> {
                    intake.StopIntake();
                    moveToLauncher.Stop();
                })
        );
    }

    /**
     * Flower intake while running move-to-launcher to make room.
     */
    public Command intakeFlowerAndFeed() {
        return sequential(
                instant(() -> {
                    intake.RunIntake();
                    moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
                }),
                waitMs(FLOWER_INTAKE_MS),
                instant(() -> {
                    intake.StopIntake();
                    moveToLauncher.Stop();
                })
        );
    }

    public Command startIntakeAndFeed() {
        return instant(() -> {
            intake.RunIntake();
            moveToLauncher.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
        });
    }

    public Command stopIntakeAndFeed() {
        return instant(() -> {
            intake.StopIntake();
            moveToLauncher.Stop();
        });
    }

    /**
     * Spin the launcher up now and keep it running between shots for the rest of auto.
     * Call once at the start of the routine. Courses that never call this behave as before.
     */
    public Command startLauncher() {
        return instant(() -> {
            keepLauncherRunning = true;
            launcher.StartLauncher();
        });
    }

    /** Stop only the move-to-launcher belt (feeder). */
    public Command stopFeed() {
        return instant(() -> moveToLauncher.Stop());
    }

    /**
     * Stop the feeder after a delay WITHOUT pausing the routine: the timer runs alongside
     * whatever command comes next (for example, the drive into the flower).
     */
    public Command stopFeedAfter(long delayMs) {
        return instant(() -> schedule(sequential(waitMs(delayMs), stopFeed())));
    }

    /**
     * Stop the intake after a delay WITHOUT pausing the routine: the timer runs alongside
     * whatever command comes next (for example, the drive into the flower).
     */
    public Command stopIntakeAfter(long delayMs) {
        return instant(() -> schedule(sequential(waitMs(delayMs), stopIntake())));
    }

    /**
     * INIT-loop vision check. Call every loop after prepareVision() and before start.
     * Reads the camera only (eyes.update()); the turret does NOT move.
     * Shows whether the camera is streaming and whether it sees our hive tag.
     */
    public void updateInitVision() {
        if (aimer == null) {
            opMode.telemetry.addLine("VISION: not prepared (call prepareVision first)");
            return;
        }

        TagSight eyes = aimer.getEyes();
        eyes.update();

        boolean streaming = eyes.isCameraOpen() && "STREAMING".equals(eyes.getCameraStateName());
        String status;
        if (!streaming) {
            status = "CAMERA NOT STREAMING";
        } else if (eyes.hasTarget) {
            status = "OK - TAG VISIBLE";
        } else {
            status = "NO TAG IN VIEW";
        }

        opMode.telemetry.addData("VISION", status);
        opMode.telemetry.addData("Camera", "%s  %.0f fps", eyes.getCameraStateName(), eyes.getFps());
        opMode.telemetry.addData("Tag", eyes.hasTarget ? eyes.targetName : "none");
        if (eyes.hasTarget) {
            opMode.telemetry.addData("Bearing (deg)", "%.1f", eyes.bearingDegrees);
            opMode.telemetry.addData("Range (in)", "%.1f", eyes.rangeInches);
            opMode.telemetry.addData("Roll (deg)", "%.1f", eyes.rollDegrees);
            opMode.telemetry.addData("Cluster found (%)", eyes.percentClusterFound);
        }
        opMode.telemetry.addData("Locked (centered)", aimer.isLocked());
    }

    /**
     * Create vision and open the camera so Driver Station Camera Stream works from Init.
     * Call once after initialize(), before waitForStart().
     * Does not move the turret.
     */
    /**
     * Create Aimer/TagSight and open the webcam so Driver Station
     * Camera Stream works from Init. Does not move the turret.
     * Call once after initialize(), before waitForStart().
     */
    public void prepareVision(TagSight.Alliance alliance) {
        ensureAimer(alliance);
        aimer.getEyes().openCamera();  // TagSight: resumeStreaming
    }

    /** Straight ahead. Only works once aimer exists. */
    public void aimStraight() {
        if (aimer == null) {
            return;
        }
        aimer.preset(180);
    }

    public Command turretStraight() {
        return instant(() -> aimStraight());
    }

    public Command flowerRampUp() {
        return sequential(
                instant(() -> flowerRamp.MoveUp()),
                waitMs(RAMP_MOVE_MS)
        );
    }

    public Command flowerRampDown() {
        return sequential(
                instant(() -> flowerRamp.MoveDown()),
                waitMs(RAMP_MOVE_MS)
        );
    }

    /** Last tip check: true = did not lock (treat as tipped), false = locked (basket still up). */
    public boolean hiveTipped() {
        return hiveTipped;
    }

    /** SEARCH full range, then decide. */
    public Command checkForTip(TagSight.Alliance alliance) {
        return checkForTipCommand(alliance, null);
    }

    /** Preset aim, watch ~2 s, then decide. */
    public Command checkForTip(TagSight.Alliance alliance, double presetDegrees) {
        //return checkForTipCommand(alliance, presetDegrees);
        return checkForTipSuccessCommand(alliance, presetDegrees);
    }

    private Command checkForTipCommand(TagSight.Alliance alliance, Double presetDegrees) {
        final long[] startMs = {0L};

        return Command.build()
                .setStart(() -> {
                    ensureAimer(alliance);
                    startMs[0] = System.currentTimeMillis();
                    hiveTipped = true;

                    if (presetDegrees != null) {
                        aimer.preset(presetDegrees);
                        aimer.track();
                    } else {
                        aimer.search();
                    }
                })
                .setDone(() -> {
                    if (aimer.isLocked()) {
                        hiveTipped = false;
                        aimer.hold();
                        return true;
                    }

                    long limitMs = (presetDegrees != null) ? TIP_WATCH_MS : TIP_SEARCH_MS;
                    if (System.currentTimeMillis() - startMs[0] >= limitMs) {
                        hiveTipped = true;
                        aimer.hold();
                        return true;
                    }
                    return false;
                });
    }

    private Command checkForTipSuccessCommand(TagSight.Alliance alliance, Double presetDegrees) {
        final long[] startMs = {0L};

        return Command.build()
                .setStart(() -> {
                    ensureAimer(alliance);
                    startMs[0] = System.currentTimeMillis();
                    hiveTipped = true;

                    if (presetDegrees != null) {
                        aimer.preset(presetDegrees);
                        aimer.track();
                    } else {
                        aimer.search();
                    }
                })
                .setDone(() -> {
                    // Positive signal: tag in view, so our side is up (no tip).
                    if (aimer.isTagVisible()) {
                        hiveTipped = false;
                        aimer.hold();
                        return true;
                    }

                    // No tag yet. Do not decide "tipped" before the minimum look time,
                    // even if the watch window is ever set shorter than that.
                    long limitMs = Math.max(
                            (presetDegrees != null) ? TIP_WATCH_MS : TIP_SEARCH_MS,
                            TIP_SUCCESS_MIN_WATCH_MS);
                    if (System.currentTimeMillis() - startMs[0] >= limitMs) {
                        hiveTipped = true;
                        aimer.hold();
                        return true;
                    }
                    return false;
                });
    }

    /**
     * After checkForTip (or partner wait): pick a path.
     * tippedCmd      = hiveTipped == true  (no lock)
     * stillLockedCmd = hiveTipped == false (still locked on basket)
     */
    public Command branchOnTip(Command tippedCmd, Command stillLockedCmd) {
        return conditional(
                this::hiveTipped,
                tippedCmd,
                stillLockedCmd
        );
    }

    /**
     * At garden after we already tipped: wait until partner tips or timeout.
     * Partner tip = we lock on the hive again (basket up for them), then same
     * rules as checkForTip will set hiveTipped.
     * Simpler: keep watching; if we LOCK, partner has not finished / state unclear;
     * if we never lock until timeout, hiveTipped stays true (treat as partner done
     * or use timeout path).
     *
     * Practical rule matching Course2HT:
     *   - If we get a lock → hiveTipped = false (still "up" — partner miss path if you want)
     *   - If timeout with no lock → hiveTipped = true (partner tipped / go happy)
     *
     * Tune PARTNER_TIP_TIMEOUT_MS as needed.
     */
    private static final long PARTNER_TIP_TIMEOUT_MS = 8000;

    public Command waitForPartnerTipOrTimeout(TagSight.Alliance alliance, double presetDegrees) {
        return waitForPartnerTipSuccessCommand(alliance, presetDegrees, PARTNER_TIP_TIMEOUT_MS);
        /*final long[] startMs = {0L};

        return Command.build()
                .setStart(() -> {
                    ensureAimer(alliance);
                    startMs[0] = System.currentTimeMillis();
                    hiveTipped = true;
                    aimer.preset(presetDegrees);
                    aimer.track();
                })
                .setDone(() -> {
                    // Saw basket locked again → not treating as "partner tip yet"
                    // For Course2HT: no lock through timeout = partner tipped (hiveTipped true)
                    // If you prefer "lock means partner tipped", flip the hiveTipped assignment.
                    if (aimer.isLocked()) {
                        hiveTipped = false;
                        aimer.hold();
                        return true;
                    }
                    if (System.currentTimeMillis() - startMs[0] >= PARTNER_TIP_TIMEOUT_MS) {
                        hiveTipped = true;
                        aimer.hold();
                        return true;
                    }
                    return false;
                });*/
    }

    // ---- PASTE THIS RIGHT AFTER your existing two-argument waitForPartnerTipOrTimeout(...) ----
    // ---- (before the final closing brace of the class). Nothing else in AutoSkills changes. ----

    /**
     * Same as the two-argument version above, but with a per-course timeout
     * (Course3HT passes a longer one).
     * Lock before timeout → hiveTipped = false; timeout with no lock → hiveTipped = true.
     */
    public Command waitForPartnerTipOrTimeout(TagSight.Alliance alliance, double presetDegrees,
                                              long timeoutMs) {
        return waitForPartnerTipSuccessCommand(alliance, presetDegrees, timeoutMs);
        /*final long[] startMs = {0L};

        return Command.build()
                .setStart(() -> {
                    ensureAimer(alliance);
                    startMs[0] = System.currentTimeMillis();
                    hiveTipped = true;
                    aimer.preset(presetDegrees);
                    aimer.track();
                })
                .setDone(() -> {
                    if (aimer.isLocked()) {
                        hiveTipped = false;
                        aimer.hold();
                        return true;
                    }
                    if (System.currentTimeMillis() - startMs[0] >= timeoutMs) {
                        hiveTipped = true;
                        aimer.hold();
                        return true;
                    }
                    return false;
                }); */
    }

    private Command waitForPartnerTipSuccessCommand(TagSight.Alliance alliance,
                                                    double presetDegrees,
                                                    long timeoutMs) {
        final long[] startMs = {0L};
        final long[] firstSeenMs = {-1L};   // when the current run of sightings began (-1 = none)
        final long[] lastSeenMs = {-1L};    // most recent sighting

        return Command.build()
                .setStart(() -> {
                    ensureAimer(alliance);
                    startMs[0] = System.currentTimeMillis();
                    firstSeenMs[0] = -1L;
                    lastSeenMs[0] = -1L;
                    hiveTipped = true;
                    aimer.preset(presetDegrees);
                    aimer.track();
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();

                    if (aimer.isTagVisible()) {
                        if (firstSeenMs[0] < 0) {
                            firstSeenMs[0] = now;
                        }
                        lastSeenMs[0] = now;

                        // Seen across the confirm time (gaps up to MAX_GAP are forgiven): partner tipped
                        if (lastSeenMs[0] - firstSeenMs[0] >= PARTNER_TIP_CONFIRM_MS) {
                            hiveTipped = false;
                            aimer.hold();
                            return true;
                        }
                    } else if (firstSeenMs[0] >= 0 && now - lastSeenMs[0] > PARTNER_TIP_MAX_GAP_MS) {
                        firstSeenMs[0] = -1L;   // it was just a blip: start the confirm over
                    }

                    if (now - startMs[0] >= timeoutMs) {
                        hiveTipped = true;
                        aimer.hold();
                        return true;
                    }
                    return false;
                });
    }

    /**
     * Run the intake roller and the feeder BACKWARDS (teleop's reverse powers) until you stop
     * them. Stop with the existing stopIntakeAndFeed(), which works in either direction.
     */
    public Command startReverseIntakeAndFeed() {
        return instant(() -> {
            intake.ReverseIntake();
            moveToLauncher.SetPower(-BeethovenHardware.MoveToLauncher.RUN_POWER);
        });
    }

    /**
     * Empty the magazine out the intake: reverse both for EMPTY_MAGAZINE_MS, then stop both.
     * The routine WAITS for it to finish. Call after shootFourPollen() when you want to be sure
     * nothing is left before the garden or the flower.
     */
    public Command emptyMagazine() {
        return sequential(
                startReverseIntakeAndFeed(),
                waitMs(EMPTY_MAGAZINE_MS),
                stopIntakeAndFeed()
        );
    }

    /**
     * Same purge, but it does NOT hold up the routine: it starts the reverse now and the stop
     * timer runs alongside the next command (for example, the drive to the garden).
     * Saves the purge time, as long as the drive lasts at least EMPTY_MAGAZINE_MS.
     */
    public Command emptyMagazineWhileDriving() {
        return instant(() -> {
            intake.ReverseIntake();
            moveToLauncher.SetPower(-BeethovenHardware.MoveToLauncher.RUN_POWER);
            schedule(sequential(waitMs(EMPTY_MAGAZINE_MS), stopIntakeAndFeed()));
        });
    }

}