package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;
import org.firstinspires.ftc.teamcode.cydogs.components.Intake;
import org.firstinspires.ftc.teamcode.cydogs.components.Launcher;
import org.firstinspires.ftc.teamcode.cydogs.components.LauncherSizeServo;
import org.firstinspires.ftc.teamcode.cydogs.components.LoaderServo;
import org.firstinspires.ftc.teamcode.cydogs.components.MoveToLauncherMotor;

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
    private static final long SPIN_UP_MS = 2000;
    private static final long BETWEEN_SHOTS_MS = 400;
    private static final long FEED_MS = 1000;
    private static final long INTAKE_SHOT_MS = 400;   // short intake between pops
    private static final long GARDEN_INTAKE_MS = 2000;
    private static final long FLOWER_INTAKE_MS = 2000;

    /** Wait long enough for LoaderServo.Update() to finish LaunchOnce. */
    private static final long POP_MS = (long) (BeethovenHardware.Loader.LAUNCH_HOLD_SECONDS * 1000.0) + 100;

    private final OpMode opMode;

    private LauncherSizeServo launcherSize;
    private Launcher launcher;
    private LoaderServo loader;
    private MoveToLauncherMotor moveToLauncher;
    private Intake intake;

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

        launcherSize.Initialize();
        launcher.Initialize();
        loader.Initialize();
        moveToLauncher.Initialize();
        intake.Initialize();

        launcherSize.SetPollen();
    }

    /** Call every OpMode loop. */
    public void update() {
        if (launcher != null) {
            launcher.Update();
        }
        if (loader != null) {
            loader.Update();
        }
    }

    // ----- private building blocks -----

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

    private Command stopShootingHardware() {
        return instant(() -> {
            intake.StopIntake();
            moveToLauncher.Stop();
            launcher.StopLauncher();
        });
    }

    // ----- skills for courses -----

    /**
     * Start launcher, fire up to four pollen balls, then stop.
     * Sequence: (pop → feed → short intake) × 3, then final pop.
     */
    public Command shootFourPollen() {
        return sequential(
                instant(() -> launcher.StartLauncher()),
                waitMs(SPIN_UP_MS),

                popOnce(), feedOnce(), intakeShort(), waitMs(BETWEEN_SHOTS_MS),
                popOnce(), feedOnce(), intakeShort(), waitMs(BETWEEN_SHOTS_MS),
                popOnce(), feedOnce(), intakeShort(), waitMs(BETWEEN_SHOTS_MS),
                popOnce(),

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
}