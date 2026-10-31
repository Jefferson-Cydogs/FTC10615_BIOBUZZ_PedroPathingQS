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

    // Launcher sensing: how often the launcher sensor is checked, and how long a manual
    // Pollen/Nectar press pauses the sensing before it resumes on its own.
    private static final double SENSE_INTERVAL_SECONDS = 0.25;
    private static final double MANUAL_OVERRIDE_SECONDS = 5.0;

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
    private boolean launchSensingOn = true;

    // Sensing stays paused until currentTimer reaches this time (set by a manual Pollen/Nectar press)
    private double sensingPausedUntil = 0;

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
            manageLauncherSensing();

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
                telemetry.addData("launcher sensor last saw:", flingyMag.flingyReady);
                telemetry.addData("next sensor last saw:", flingyMag.flingyNext);
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
       D-pad left             Launch sensing OFF (emergency: size is manual only)
       D-pad right            Launch sensing ON
       Triangle               Turret preset: straight (180°)
       Circle                 Turret preset: 150°
       Square                 Turret preset: 210°
       Cross (X)              Turret search (sweeps until it sees a tag, then centers on it)

     Gamepad 2 (manipulator)
       Right trigger (hold)   Intake in
       Left trigger (hold)    Intake backwards
       Right bumper (hold)    moveToLauncherMotor forward
       Left bumper (hold)     moveToLauncherMotor reverse
       Triangle               Launcher on
       Cross (X)              Launcher off
       Circle (one push)      Loader goes to Launch, then right back to Load
       D-pad left             Launcher size: Pollen (also pauses sensing for 5 seconds)
       D-pad right            Launcher size: Nectar (also pauses sensing for 5 seconds)
       D-pad up               Flower ramp up
       D-pad down             Flower ramp down
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

        // ----- Launch sensing switch: left = off, right = on -----
        if(gamepad1.dpadLeftWasPressed())
        {
            launchSensingOn = false;
        }
        else if(gamepad1.dpadRightWasPressed())
        {
            launchSensingOn = true;
            sensingPausedUntil = 0;   // turning it on also cancels any manual-override pause
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

        // ----- Launcher size, manual: also pauses the sensing for a few seconds -----
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


        // ----- Move to launcher: hold right bumper -----
        if(gamepad2.right_bumper)
        {
            moveToLauncherMotor.SetPower(BeethovenHardware.MoveToLauncher.RUN_POWER);
        }
        else if(gamepad2.left_bumper)
        {
            moveToLauncherMotor.SetPower(-BeethovenHardware.MoveToLauncher.RUN_POWER);
        }
        else
        {
            moveToLauncherMotor.SetPower(0);
        }

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
     * Looks at the launcher sensor every SENSE_INTERVAL_SECONDS and sets the launcher size:
     * red or blue (nectar) -> Nectar, yellow (pollen) -> Pollen, nothing seen -> no change.
     * Does nothing while sensing is off or paused, or while the loader is mid-launch.
     */
    private void manageLauncherSensing()
    {
        if (!isSensingActive())
        {
            return;
        }
        if (LanceTheLoaderServo.IsLaunching())
        {
            return;
        }
        if (!eventTracker.doEvent("LauncherSensor", currentTimer.seconds(), SENSE_INTERVAL_SECONDS))
        {
            return;
        }

        // Reads only the launcher sensor, fresh, then flingyReady holds the color it saw
        if (flingyMag.IsBallReadyForLaunch())
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
    }

    /** True when the driver has sensing on AND no manual-override pause is running. */
    private boolean isSensingActive()
    {
        return launchSensingOn && currentTimer.seconds() >= sensingPausedUntil;
    }

    /** Text for the telemetry line that says whether the sensors are being used. */
    private String sensingStatus()
    {
        if (!launchSensingOn)
        {
            return "OFF (driver) - launcher size is manual only";
        }
        double secondsLeft = sensingPausedUntil - currentTimer.seconds();
        if (secondsLeft > 0)
        {
            return String.format("PAUSED (manual override) - resumes in %.1f s", secondsLeft);
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