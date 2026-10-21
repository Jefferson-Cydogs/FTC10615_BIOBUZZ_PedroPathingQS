package org.firstinspires.ftc.teamcode.cydogs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.cydogs.core.EventTracker;
import org.firstinspires.ftc.teamcode.cydogs.vision.Aimer;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@TeleOp(name="Beetoven TeleOp", group= "TeleOp")
public class BioBuzzTeleop extends LinearOpMode {

    // Power used while a loader motor button is held. Flip the sign if a motor runs the wrong way.
    private static final double MOVE_TO_LAUNCHER_POWER = 0.5;

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

    public String Team = "blue";

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
            myTurret.update();

            if (eventTracker.doEvent("Telemetry",currentTimer.seconds(),0.5)) {
                telemetry.addData("launcher target %:", LarryTheLauncher.GetTargetLaunchPercent());
                telemetry.addData("launcher actual %:", LarryTheLauncher.GetLaunchVelocityPercent());
                telemetry.update();
            }
        }

        myTurret.close();
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
            myTurret.preset(Turret.STRAIGHT_DEGREES);
        }
        else if(gamepad1.circleWasPressed())
        {
            myTurret.preset(150);
        }
        else if(gamepad1.squareWasPressed())
        {
            myTurret.preset(210);
        }
        else if(gamepad1.crossWasPressed())
        {
            myTurret.search();
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
            moveToLauncherMotor.SetPower(MOVE_TO_LAUNCHER_POWER);
        }
        else if(gamepad2.left_bumper)
        {
            moveToLauncherMotor.SetPower(-MOVE_TO_LAUNCHER_POWER);
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
        myTurret = new Aimer(this, TagSight.Alliance.BLUE);
        LauncherSize = new LauncherSizeServo(this);

        moveToLauncherMotor = new MoveToLauncherMotor(this);
    }

    private void initializeDevices()
    {
        Wheels.InitializeChassisTeleop(.6,.3,.5);

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