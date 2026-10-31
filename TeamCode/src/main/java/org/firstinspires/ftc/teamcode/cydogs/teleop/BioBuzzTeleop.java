package org.firstinspires.ftc.teamcode.cydogs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cydogs.chassis.BioBuzzRobotChassis;
import org.firstinspires.ftc.teamcode.cydogs.components.FlingyMag;
import org.firstinspires.ftc.teamcode.cydogs.components.FlowerRampServo;
import org.firstinspires.ftc.teamcode.cydogs.components.FlowerScoringServo;
import org.firstinspires.ftc.teamcode.cydogs.components.FlowerSlider;
import org.firstinspires.ftc.teamcode.cydogs.components.Intake;
import org.firstinspires.ftc.teamcode.cydogs.components.Launcher;
import org.firstinspires.ftc.teamcode.cydogs.components.LauncherSizeServo;
import org.firstinspires.ftc.teamcode.cydogs.components.LoaderServo;
import org.firstinspires.ftc.teamcode.cydogs.components.MoveToLauncherMotor;
import org.firstinspires.ftc.teamcode.cydogs.components.Turret;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;
import org.firstinspires.ftc.teamcode.cydogs.core.EventTracker;
import org.firstinspires.ftc.teamcode.cydogs.vision.Aimer;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


/**
 * Common teleop for both alliances. It has no @TeleOp annotation, so it does not show up on the
 * Driver Hub. Use BioBuzzTeleopRED or BioBuzzTeleopBLUE, which pass in the alliance.
 */
public abstract class BioBuzzTeleop extends LinearOpMode {

    // Drive speeds passed to the chassis for teleop (0 to 1). Change them here.
    private static final double HIGH_SPEED_DRIVE = 0.6;   // right stick (fast movement)
    private static final double LOW_SPEED_DRIVE = 0.3;    // left stick (slow, precise movement)
    private static final double ROTATE_SPEED_DRIVE = 0.5; // trigger rotation

    // How far a trigger must be pulled to count as "pressed"
    private static final double TRIGGER_THRESHOLD = 0.4;

    // Sensing: how often the sensors are read when idle, and while gamepad2's right bumper is held.
    // MANUAL_OVERRIDE_SECONDS: how long a manual Pollen/Nectar press pauses the launcher-size sensing.
    private static final double SENSE_INTERVAL_SECONDS = 0.25;
    private static final double HELD_SENSE_INTERVAL_SECONDS = 0.1;
    private static final double MANUAL_OVERRIDE_SECONDS = 5.0;

    // Auto-feed: each pulse runs moveToLauncher this long, then the sensors are checked again.
    // After MAX_FEED_PULSES in a row (20 x 0.1 s = 2 s) with the ball still not at launch, it stops.
    private static final double FEED_PULSE_SECONDS = 0.1;
    private static final int MAX_FEED_PULSES = 20;

    /** declare variables here */
    private BioBuzzRobotChassis Wheels;

    private Intake IggyTheIntake;
    private Launcher LarryTheLauncher;
    private LoaderServo LanceTheLoaderServo;
    private LauncherSizeServo LauncherSize;
    private FlowerRampServo FlowerRamp;
    private FlowerSlider FrankTheFlowerSlider;
    private FlowerScoringServo FinnTheFlowerScoringServo;

    private MoveToLauncherMotor moveToLauncherMotor;

    private FlingyMag flingyMag;

    private ElapsedTime currentTimer;
    private ElapsedTime matchTimer;
    private EventTracker eventTracker;

    private Aimer myTurret;

    // The alliance this teleop is running for (set by BioBuzzTeleopRED / BioBuzzTeleopBLUE)
    private final TagSight.Alliance alliance;

    private boolean isLauncherAtSpeed = false;

    // Driver's sensing switch: gamepad1 D-pad left = off, D-pad right = on. Starts on.
    // OFF means no sensors are used at all: no size sensing, no blocking, no auto-feed.
    private boolean launchSensingOn = true;

    // Launcher-size sensing stays paused until currentTimer reaches this time
    // (set by a manual Pollen/Nectar press on gamepad2). Does not affect blocking or auto-feed.
    private double sensingPausedUntil = 0;

    // Latest sensor results and when the sensors are next due to be read
    private boolean launchSpotOccupied = false;
    private boolean nextSpotOccupied = false;
    private double nextSensorPassAt = 0;
    private double lastSensorPassMs = 0;

    // Auto-feed state
    private boolean feedPulsing = false;
    private double feedPulseEndsAt = 0;
    private int feedPulseCount = 0;
    private String feedStatus = "IDLE";

    protected BioBuzzTeleop(TagSight.Alliance alliance)
    {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode()
    {
        /** Execute initialization actions here */

        constructDevices();
        initializeDevices();
        initializePositions();
        currentTimer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        eventTracker = new EventTracker();

        waitForStart();

        matchTimer.reset();

        while (opModeIsActive()) {
            /** Execute OpMode actions here */
            Wheels.OptimizedTeleopDrive();
            manageDriverControls();
            manageManipulatorControls();
            manageSensorPass();
            manageMoveToLauncher();

            // Returns the loader servo to Load once a launch has been held long enough
            LanceTheLoaderServo.Update();
            LarryTheLauncher.Update();
            isLauncherAtSpeed = LarryTheLauncher.IsLauncherAtSpeed();
            //      myTurret.update();

            if (eventTracker.doEvent("Telemetry",currentTimer.seconds(),0.5)) {

                telemetry.addData("launcher target %:", LarryTheLauncher.GetTargetLaunchPercent());
                telemetry.addData("launcher actual %:", LarryTheLauncher.GetLaunchVelocityPercent());
                telemetry.addData("is launcher at speed:", isLauncherAtSpeed);
                telemetry.addData("launch sensing", sensingStatus());
                telemetry.addData("feed assist", feedStatus);
                telemetry.addData("launcher sensor last saw:", flingyMag.flingyReady);
                telemetry.addData("next sensor last saw:", flingyMag.flingyNext);
                telemetry.addData("last sensor pass ms:", lastSensorPassMs);
                telemetry.update();
            }
        }

        //   myTurret.close();
    }

    /*
     CONTROLS

     Gamepad 1 (driver)
       Left stick             Drive (move)
       Right stick            Drive (turn)
       D-pad up / down        Launcher target speed + / - (1% per press)
       D-pad left             Sensors OFF (emergency: everything manual, no blocking, no auto-feed)
       D-pad right            Sensors ON
       Triangle               Turret preset: straight (180°)
       Circle                 Turret preset: 150°
       Square                 Turret preset: 210°
       Cross (X)              Turret search (sweeps until it sees a tag, then centers on it)

     Gamepad 2 (manipulator)
       Right trigger (hold)   Intake in
       Left trigger (hold)    Intake backwards
       Right bumper (hold)    moveToLauncherMotor forward (blocked while a ball is at launch, sensors ON)
       Left bumper (hold)     moveToLauncherMotor reverse (never blocked)
       Triangle               Launcher on
       Cross (X)              Launcher off
       Circle (one push)      Loader goes to Launch, then right back to Load
       D-pad left             Launcher size: Pollen (also pauses size sensing for 5 seconds)
       D-pad right            Launcher size: Nectar (also pauses size sensing for 5 seconds)
       D-pad up               Flower ramp up
       D-pad down             Flower ramp down

     With sensors ON and no bumper held: if a ball is in the Next spot and none at Launch, the
     move-to-launcher motor pulses 0.1 s at a time (checking the sensors between pulses) until
     the ball reaches Launch.
    */
    private void manageDriverControls()
    {
        if(gamepad1.dpadUpWasPressed())
        {
            LarryTheLauncher.IncreaseLaunchPercent();
        }
        else if(gamepad1.dpadDownWasPressed())
        {
            LarryTheLauncher.DecreaseLaunchPercent();
        }

        // ----- Sensors switch: left = off, right = on -----
        if(gamepad1.dpadLeftWasPressed())
        {
            launchSensingOn = false;
        }
        else if(gamepad1.dpadRightWasPressed())
        {
            launchSensingOn = true;
            sensingPausedUntil = 0;   // turning it on also cancels any manual-override pause
            nextSensorPassAt = 0;     // read the sensors right away
            feedPulseCount = 0;       // and let a capped auto-feed try again
        }

        if(gamepad1.triangleWasPressed())
        {
            //  myTurret.preset(BeethovenHardware.Turret.STRAIGHT_DEGREES);
        }
        else if(gamepad1.circleWasPressed())
        {
            //   myTurret.preset(150);
        }
        else if(gamepad1.squareWasPressed())
        {
            //   myTurret.preset(210);
        }
        else if(gamepad1.crossWasPressed())
        {
            //   myTurret.search();
        }

    }

    private void manageManipulatorControls()
    {
        // ----- Intake: hold right trigger = in, hold left trigger = backwards -----
        if(gamepad2.right_trigger > TRIGGER_THRESHOLD)
        {
            IggyTheIntake.RunIntake();
        }
        else if(gamepad2.left_trigger > TRIGGER_THRESHOLD)
        {
            IggyTheIntake.ReverseIntake();
        }
        else
        {
            IggyTheIntake.StopIntake();
        }

        // ----- Launcher size, manual: also pauses the size sensing for a few seconds -----
        if(gamepad2.dpadLeftWasPressed())
        {
            LauncherSize.SetPollen();
            sensingPausedUntil = currentTimer.seconds() + MANUAL_OVERRIDE_SECONDS;
        }
        else if(gamepad2.dpadRightWasPressed())
        {
            LauncherSize.SetNectar();
            sensingPausedUntil = currentTimer.seconds() + MANUAL_OVERRIDE_SECONDS;
        }

        // ----- Flower ramp: D-pad up = up, D-pad down = down -----
        if(gamepad2.dpadUpWasPressed())
        {
            FlowerRamp.MoveUp();
        }
        else if(gamepad2.dpadDownWasPressed())
        {
            FlowerRamp.MoveDown();
        }

        // (Move to launcher is handled in manageMoveToLauncher(), which needs the sensor results.)

        // ----- Launcher: triangle = on, cross = off -----
        if(gamepad2.triangleWasPressed())
        {
            LarryTheLauncher.StartLauncher();
        }
        else if(gamepad2.crossWasPressed())
        {
            LarryTheLauncher.StopLauncher();
        }

        // ----- Load to launch: one push of circle = Launch position, then back to Load -----
        if(gamepad2.circleWasPressed())
        {
            LanceTheLoaderServo.LaunchOnce();
        }
    }

    /**
     * Reads the sensors when they are due: every SENSE_INTERVAL_SECONDS, every
     * HELD_SENSE_INTERVAL_SECONDS while gamepad2's right bumper is held, right when that bumper
     * is first pressed, and right after each auto-feed pulse. Does nothing when sensing is off.
     *
     * Each pass reads the launcher sensor (one read when empty). It also sets the launcher
     * size (red/blue = Nectar, yellow = Pollen) when size sensing is active and the loader is
     * not mid-launch. The two intake sensors are read only when the launch spot is empty,
     * since "next" only matters then.
     */
    private void manageSensorPass()
    {
        if (!launchSensingOn)
        {
            return;
        }

        double now = currentTimer.seconds();

        if (gamepad2.rightBumperWasPressed())
        {
            nextSensorPassAt = 0;   // fresh reading the moment the bumper goes down
        }
        if (now < nextSensorPassAt)
        {
            return;
        }

        ElapsedTime passTimer = new ElapsedTime();

        launchSpotOccupied = flingyMag.IsBallReadyForLaunch();

        if (launchSpotOccupied && isSizeSensingActive() && !LanceTheLoaderServo.IsLaunching())
        {
            String color = flingyMag.flingyReady;
            if (color.equals("red") || color.equals("blue"))
            {
                LauncherSize.SetNectar();
            }
            else if (color.equals("yellow"))
            {
                LauncherSize.SetPollen();
            }
        }

        if (!launchSpotOccupied)
        {
            nextSpotOccupied = flingyMag.IsBallInMidPosition();
        }

        lastSensorPassMs = passTimer.milliseconds();
        nextSensorPassAt = now + (gamepad2.right_bumper ? HELD_SENSE_INTERVAL_SECONDS : SENSE_INTERVAL_SECONDS);
    }

    /**
     * Decides the move-to-launcher motor power every loop.
     *   Sensors OFF:           plain manual (right bumper forward, left bumper reverse).
     *   Left bumper:           reverse, never blocked.
     *   Right bumper:          forward, unless a ball is at the launch spot (then blocked).
     *   No bumper:             auto-feed in 0.1 s pulses while a ball is in Next and none at Launch.
     */
    private void manageMoveToLauncher()
    {
        double now = currentTimer.seconds();
        boolean forward = gamepad2.right_bumper;
        boolean reverse = gamepad2.left_bumper;

        // ----- Sensors off: fully manual -----
        if (!launchSensingOn)
        {
            feedPulsing = false;
            feedPulseCount = 0;
            if (forward)
            {
                moveToLauncherMotor.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
            }
            else if (reverse)
            {
                moveToLauncherMotor.SetPower(-BeethovenHardware.MoveToLauncher.RUN_POWER);
            }
            else
            {
                moveToLauncherMotor.SetPower(0);
            }
            feedStatus = "OFF - sensors not used";
            return;
        }

        // ----- Manual reverse: always allowed -----
        if (reverse)
        {
            feedPulsing = false;
            moveToLauncherMotor.SetPower(-BeethovenHardware.MoveToLauncher.RUN_POWER);
            feedStatus = "MANUAL REVERSE";
            return;
        }

        // ----- Manual forward: blocked while a ball is at the launch spot -----
        if (forward)
        {
            feedPulsing = false;
            if (launchSpotOccupied)
            {
                moveToLauncherMotor.SetPower(0);
                feedStatus = "BLOCKED - ball at launch";
            }
            else
            {
                moveToLauncherMotor.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
                feedStatus = "MANUAL FEED";
            }
            return;
        }

        // ----- No bumper held: auto-feed -----
        if (feedPulsing)
        {
            if (now < feedPulseEndsAt)
            {
                feedStatus = "AUTO-FEEDING";
                return;   // motor is already running for this pulse
            }
            // Pulse is over: stop, then look again right away
            moveToLauncherMotor.SetPower(0);
            feedPulsing = false;
            nextSensorPassAt = 0;
            feedStatus = "CHECKING";
            return;
        }

        moveToLauncherMotor.SetPower(0);

        if (LanceTheLoaderServo.IsLaunching())
        {
            feedStatus = "WAITING - loader launching";
            return;
        }

        if (!launchSpotOccupied && nextSpotOccupied)
        {
            if (feedPulseCount >= MAX_FEED_PULSES)
            {
                feedStatus = "STOPPED - ball not reaching launch (jam?). Driver D-pad left = sensors off";
                return;
            }
            moveToLauncherMotor.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
            feedPulsing = true;
            feedPulseEndsAt = now + FEED_PULSE_SECONDS;
            feedPulseCount++;
            feedStatus = "AUTO-FEEDING";
        }
        else
        {
            feedPulseCount = 0;   // nothing to feed, or the ball made it: start counting fresh
            feedStatus = launchSpotOccupied ? "IDLE - ball at launch" : "IDLE";
        }
    }

    /** True when the driver has sensing on AND no manual-override pause is running (size sensing only). */
    private boolean isSizeSensingActive()
    {
        return launchSensingOn && currentTimer.seconds() >= sensingPausedUntil;
    }

    /** Text for the telemetry line that says whether the sensors are being used. */
    private String sensingStatus()
    {
        if (!launchSensingOn)
        {
            return "OFF (driver) - sensors not used, everything manual";
        }
        double secondsLeft = sensingPausedUntil - currentTimer.seconds();
        if (secondsLeft > 0)
        {
            return String.format("ON - size sensing paused by manual override, resumes in %.1f s", secondsLeft);
        }
        return "ON";
    }

    private void constructDevices()
    {
        Wheels = new BioBuzzRobotChassis(this);

        IggyTheIntake = new Intake(this);
        LarryTheLauncher = new Launcher(this);
        LanceTheLoaderServo = new LoaderServo(this);
        // FrankTheFlowerSlider = new FlowerSlider(this);
        // FinnTheFlowerScoringServo = new FlowerScoringServo(this);
        //  myTurret = new Aimer(this, alliance);
        LauncherSize = new LauncherSizeServo(this);
        FlowerRamp = new FlowerRampServo(this);

        moveToLauncherMotor = new MoveToLauncherMotor(this);

        flingyMag = new FlingyMag(this);
    }

    private void initializeDevices()
    {
        Wheels.InitializeChassisTeleop(HIGH_SPEED_DRIVE, LOW_SPEED_DRIVE, ROTATE_SPEED_DRIVE);

        IggyTheIntake.Initialize();
        LarryTheLauncher.Initialize();
        LanceTheLoaderServo.Initialize();
        LauncherSize.Initialize();
        FlowerRamp.Initialize();

        moveToLauncherMotor.Initialize();

        // FlingyMag has no Initialize(); its sensors are ready once constructed.

        // FrankTheFlowerSlider.Initialize();
        // FinnTheFlowerScoringServo.Initialize();
    }

    private void initializePositions()
    {
        // The loader no longer gets Load() every loop, so start it in the Load position.
        LanceTheLoaderServo.Load();
        LauncherSize.SetPollen();
    }
}