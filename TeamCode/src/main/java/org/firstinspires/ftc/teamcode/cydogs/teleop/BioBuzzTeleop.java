package org.firstinspires.ftc.teamcode.cydogs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cydogs.chassis.BioBuzzRobotChassis;
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

    /** declare variables here */
    private BioBuzzRobotChassis Wheels;

    private Intake IggyTheIntake;
    private Launcher LarryTheLauncher;
    private LoaderServo LanceTheLoaderServo;
    private LauncherSizeServo LauncherSize;
    private FlowerSlider FrankTheFlowerSlider;
    private FlowerScoringServo FinnTheFlowerScoringServo;

    private MoveToLauncherMotor moveToLauncherMotor;

    private ElapsedTime currentTimer;
    private ElapsedTime matchTimer;
    private EventTracker eventTracker;

    private Aimer myTurret;

    // The alliance this teleop is running for (set by BioBuzzTeleopRED / BioBuzzTeleopBLUE)
    private final TagSight.Alliance alliance;

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

            // Returns the loader servo to Load once a launch has been held long enough
            LanceTheLoaderServo.Update();
            LarryTheLauncher.Update();
      //      myTurret.update();

            if (eventTracker.doEvent("Telemetry",currentTimer.seconds(),0.5)) {
                telemetry.addData("launcher target %:", LarryTheLauncher.GetTargetLaunchPercent());
                telemetry.addData("launcher actual %:", LarryTheLauncher.GetLaunchVelocityPercent());
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
       D-pad left             Launcher size: Pollen
       D-pad right            Launcher size: Nectar
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
        if(gamepad2.dpadLeftWasPressed())
        {
            LauncherSize.SetPollen();
        }
        else if(gamepad2.dpadRightWasPressed())
        {
            LauncherSize.SetNectar();
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

        moveToLauncherMotor = new MoveToLauncherMotor(this);
    }

    private void initializeDevices()
    {
        Wheels.InitializeChassisTeleop(HIGH_SPEED_DRIVE, LOW_SPEED_DRIVE, ROTATE_SPEED_DRIVE);

        IggyTheIntake.Initialize();
        LarryTheLauncher.Initialize();
        LanceTheLoaderServo.Initialize();
        LauncherSize.Initialize();

        moveToLauncherMotor.Initialize();

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