package org.firstinspires.ftc.teamcode.cydogs.trix;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseContinuousServo;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePowerMotor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BioBuzzRobotChassis;
import org.firstinspires.ftc.teamcode.cydogs.components.BallTracker;
import org.firstinspires.ftc.teamcode.cydogs.components.DumpArm;


@TeleOp
public class TrixTeleop extends LinearOpMode {

    /*
        Motors
            launcher
            intake
        Servos
            windmillServo
            right_intake_servo
            left_intake_servo
            medium_flower_servo
            lower_flower_servo
            upper_flower_servo
            LedLight


Drive	Gamepad 1 sticks	Yes
Launcher on	Gamepad 2 Triangle	Yes
Launcher off	Gamepad 2 Cross (X)	Yes
Intake on	Gamepad 2 Right Trigger	Yes
Reverse intake	Gamepad 2 Left Trigger	Yes
Run feeder	Gamepad 2 Bumper (held)	Yes
     */


    private BioBuzzRobotChassis TrixWheels;


    private BaseContinuousServo leftloader;

    private BaseContinuousServo rightloader;

    private BasePowerMotor launcher;
    private double launcherPower = 0.465;

    private BasePowerMotor intake;

    private BaseContinuousServo feeder;

    private DumpArm flowerArm;


    // Ball tracker properties
  //  private BallTracker ballTracker;
    static final double KP_TURN = 0.004;
    static final double ASSIST_DRIVE_POWER = 0.4;
    static final double MAX_ASSIST_TURN = 0.4;
    static final int STOP_WIDTH = 120;

    @Override
    public void runOpMode() {

        // Execute initialization actions here

        TrixWheels = new BioBuzzRobotChassis(this);
        TrixWheels.InitializeChassisTeleop(.75,.3,.5);
        TrixWheels.FrontLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        initializeDevices();
        initializePositions();



    /*    while (!isStarted() && !isStopRequested()) {
            if (gamepad1.square) ballTracker.setAlliance(BallTracker.Alliance.BLUE);
            if (gamepad1.circle) ballTracker.setAlliance(BallTracker.Alliance.RED);
            telemetry.addData("Alliance (Square=blue, Circle=red)", ballTracker.getAlliance());
            telemetry.addData("HuskyLens connected", ballTracker.isConnected());
            telemetry.update();
        }*/

        waitForStart();

        while (opModeIsActive()) {
            manageDriverControls();
            manageManipulatorControls();
        }

    }

    private void manageDriverControls()
    {
     //   boolean assist = gamepad1.dpad_up;
     //   if (assist) {
         //   ballTracker.update();          // only read the camera while assisting
     //   }

     //   if (assist && ballTracker.hasTarget()) {
     //       if (ballTracker.isWithin(STOP_WIDTH)) {
     //           TrixWheels.stopMotors();
     //       } else {
     //           double turn = Range.clip(ballTracker.getXError() * KP_TURN,
     //                   -MAX_ASSIST_TURN, MAX_ASSIST_TURN);
     //           TrixWheels.DriveRobotCentric(ASSIST_DRIVE_POWER, 0, turn);
     //       }
     //   } else {
            TrixWheels.OptimizedTeleopDrive();
     //   }
    }

    private void manageManipulatorControls()
    {  /*
        Launcher on	Gamepad 2 Triangle	Yes
        Launcher off	Gamepad 2 Cross (X)	Yes
        Intake on	Gamepad 2 Right Trigger	Yes
        Reverse intake	Gamepad 2 Left Trigger	Yes
        Run feeder	Gamepad 2 Bumper (held)	Yes
            */

        if(gamepad2.triangle)
        {
            launcher.SetPower(launcherPower);
        }
        else if (gamepad2.cross)
        {
            launcher.Stop();
        }
        if (gamepad2.right_trigger > .4)
        {
            intake.SetPower(.85);
            rightloader.RunForward();
            leftloader.RunBackward();
        }
        else if (gamepad2.left_trigger > .4)
        {
            intake.SetPower(-.5);
            rightloader.Stop();
            leftloader.Stop();
        }
        else {
            intake.Stop();
            rightloader.Stop();
            leftloader.Stop();
        }
        if(gamepad2.right_bumper)
        {
            feeder.RunForward();
        }
        else if(gamepad2.left_bumper)
        {
            feeder.RunBackward();
        }
        else
        {
            feeder.Stop();
        }
  /*      if(gamepad2.dpadUpWasPressed())
        {
            flowerArm.MoveToDump();
        }
        if(gamepad2.dpadDownWasPressed())
        {
            flowerArm.MoveToRest();
        }

        flowerArm.Update();*/

    }



    private void initializeDevices()
    {
        leftloader = new BaseContinuousServo(this,"left_intake_servo", DcMotorSimple.Direction.FORWARD,0.9);
        leftloader.Initialize();

        rightloader = new BaseContinuousServo(this,"right_intake_servo", DcMotorSimple.Direction.FORWARD, 0.9);
        rightloader.Initialize();

        feeder = new BaseContinuousServo(this,"windmillServo", DcMotorSimple.Direction.REVERSE, 0.6);
        feeder.Initialize();

        launcher = new BasePowerMotor(this, "launcher", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.Initialize();

        intake = new BasePowerMotor(this, "intake", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        intake.Initialize();

    //    flowerArm = new DumpArm(this);
     //   flowerArm.Initialize();

      //  ballTracker = new BallTracker(hardwareMap, "huskylens", BallTracker.Alliance.BLUE);

    }

    private void initializePositions()
    {
    //    flowerArm.SnapToRest();
    }


}
