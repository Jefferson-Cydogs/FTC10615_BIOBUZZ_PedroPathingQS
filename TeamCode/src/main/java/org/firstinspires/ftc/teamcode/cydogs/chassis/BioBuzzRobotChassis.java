package org.firstinspires.ftc.teamcode.cydogs.chassis;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class BioBuzzRobotChassis {
    public DcMotor FrontLeftWheel;
    public DcMotor FrontRightWheel;
    public DcMotor BackLeftWheel;
    public DcMotor BackRightWheel;
    private float RightStickYValue;
    private float RightStickXValue;
    private float LeftStickYValue;
    private float LeftStickXValue;
    private float TriggersValue;
    private float BumpersValue;
    private double highSpeedDrive = BeethovenHardware.Drive.HIGH_SPEED;
    private double lowSpeedDrive = BeethovenHardware.Drive.LOW_SPEED;
    private double rotateSpeedDrive = BeethovenHardware.Drive.ROTATE_SPEED;
    private float rotateLowSpeedDrive = (float) BeethovenHardware.Drive.ROTATE_LOW_SPEED;
    private double FastStraight;
    private double Straight;
    private double FastStrafe;
    private double Strafe;
    private double FastRotate;
    private double Rotate;
    private double FrontLeftPower;
    private double FrontRightPower;
    private double BackLeftPower;
    private double BackRightPower;
    public LinearOpMode myOpMode;
    //public static final int OneTileMM = 610;

    /** This is the constructor for the class.  It takes a parameter for currentOp, which allows
     it to store and use the current op mode.  The four wheels' direction is initialized here. */
    public BioBuzzRobotChassis(LinearOpMode currentOp)
    {
        // The op mode is important code provided by first.  It has the hardwareMap, sleep function,
        //   and telemetry functions.
        myOpMode = currentOp;
        HardwareMap hardwareMap = myOpMode.hardwareMap;

        // This gets the devices from the configuration on the robot.
        //    This call basically says "Get me the thing called FrontRightWheel from the
        //    configuration, and trust me it can be mapped to the DCMotor class. If it's
        //    not a motor, then code later on will throw errors when it tries to do motor
        //    things with a non motor.
        FrontLeftWheel = hardwareMap.get(DcMotor.class, BeethovenHardware.Drive.FRONT_LEFT_NAME);
        FrontRightWheel = hardwareMap.get(DcMotor.class, BeethovenHardware.Drive.FRONT_RIGHT_NAME);
        BackLeftWheel = hardwareMap.get(DcMotor.class, BeethovenHardware.Drive.BACK_LEFT_NAME);
        BackRightWheel = hardwareMap.get(DcMotor.class, BeethovenHardware.Drive.BACK_RIGHT_NAME);

        FrontLeftWheel.setDirection(BeethovenHardware.Drive.FRONT_LEFT_DIRECTION);
        FrontRightWheel.setDirection(BeethovenHardware.Drive.FRONT_RIGHT_DIRECTION);
        BackLeftWheel.setDirection(BeethovenHardware.Drive.BACK_LEFT_DIRECTION);
        BackRightWheel.setDirection(BeethovenHardware.Drive.BACK_RIGHT_DIRECTION);
    }

    public void InitializeChassisTeleop(double highSpeed, double lowSpeed, double rotateSpeed)
    {
        highSpeedDrive = highSpeed;
        lowSpeedDrive = lowSpeed;
        rotateSpeedDrive = rotateSpeed;

        // Set motors' ZeroPower behavior
        FrontLeftWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_ZERO_POWER_BEHAVIOR);
        FrontRightWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_ZERO_POWER_BEHAVIOR);
        BackLeftWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_ZERO_POWER_BEHAVIOR);
        BackRightWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_ZERO_POWER_BEHAVIOR);

        // Clear Encoders of prior data; only necessary if motors will be configured to RUN_USING_ENCODER (for Velocity instead of Power)
        // or their encoders data will be read (even when using RUN_WITHOUT_ENCODER)
        //FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // > Set some motors' modes different from RUN_WITHOUT_ENCODER (default); suggested anyway if Auton was using RUN_TO_POSITION
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure motors are stopped, for predictable behavior and avoiding unintended motion
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }

    public void ChassisTeleopBrakeWheels()
    {
        FrontLeftWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_BRAKE_ZERO_POWER_BEHAVIOR);
        FrontRightWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_BRAKE_ZERO_POWER_BEHAVIOR);
        BackLeftWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_BRAKE_ZERO_POWER_BEHAVIOR);
        BackRightWheel.setZeroPowerBehavior(BeethovenHardware.Drive.TELEOP_BRAKE_ZERO_POWER_BEHAVIOR);
    }

    public void InitializeChassisAutonomous()
    {
        // Set motors' ZeroPower behavior
        FrontLeftWheel.setZeroPowerBehavior(BeethovenHardware.Drive.AUTON_ZERO_POWER_BEHAVIOR);
        FrontRightWheel.setZeroPowerBehavior(BeethovenHardware.Drive.AUTON_ZERO_POWER_BEHAVIOR);
        BackLeftWheel.setZeroPowerBehavior(BeethovenHardware.Drive.AUTON_ZERO_POWER_BEHAVIOR);
        BackRightWheel.setZeroPowerBehavior(BeethovenHardware.Drive.AUTON_ZERO_POWER_BEHAVIOR);

        // Clear Encoders of prior data
        FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the current position as initial one
        FrontLeftWheel.setTargetPosition(0);
        FrontRightWheel.setTargetPosition(0);
        BackLeftWheel.setTargetPosition(0);
        BackRightWheel.setTargetPosition(0);

        // Set some motors' modes different from RUN_WITHOUT_ENCODER (default)
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the PIDF coefficients per wheel motor; only applies for RUN_USING_ENCODER / RUN_TO_POSITION and after selecting one of them
        //    Need to calculate this for Beethoven, depends on each motor even with identical specs
        /*((DcMotorEx) FrontLeftWheel).setVelocityPIDFCoefficients(0,0,0,BeethovenHardware.Drive.FRONT_LEFT_VELOCITY_F);
        ((DcMotorEx) FrontRightWheel).setVelocityPIDFCoefficients(0,0,0,BeethovenHardware.Drive.FRONT_RIGHT_VELOCITY_F);
        ((DcMotorEx) BackLeftWheel).setVelocityPIDFCoefficients(0,0,0,BeethovenHardware.Drive.BACK_LEFT_VELOCITY_F);
        ((DcMotorEx) BackRightWheel).setVelocityPIDFCoefficients(0,0,0,BeethovenHardware.Drive.BACK_RIGHT_VELOCITY_F);*/

        // Ensure motors are stopped, for predictable behavior and avoiding unintended motion
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }

    public void TraditionalTeleopDrive()
    {
        RightStickYValue = -myOpMode.gamepad1.right_stick_y;
        RightStickXValue = myOpMode.gamepad1.right_stick_x;
        LeftStickYValue = -myOpMode.gamepad1.left_stick_y;
        LeftStickXValue = myOpMode.gamepad1.left_stick_x;
        TriggersValue = myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger;

        if (RightStickYValue != 0 || RightStickXValue != 0 || LeftStickYValue != 0 || LeftStickXValue != 0 || TriggersValue != 0) {
            // Set robot's fast move forward(+) or backwards(-) power
            FastStraight = highSpeedDrive * (0.75 * Math.pow(RightStickYValue, 3) + 0.25 * RightStickYValue);
            // Set robot's move forward(+) or backwards(-) power
            Straight = lowSpeedDrive * LeftStickYValue;
            // Set robot's fast strafe right(+) or left(-) power
            FastStrafe = highSpeedDrive * (0.75 * Math.pow(RightStickXValue, 3) + 0.25 * RightStickXValue);
            // Set robot's strafe right(+) or left(-) power
            Strafe = lowSpeedDrive * LeftStickXValue;
            // Set robot's clockwise(+) or counter-clockwise(-) rotation power
            Rotate = rotateSpeedDrive * (0.75 * Math.pow(TriggersValue, 3) + 0.25 * TriggersValue);
            // MOve all wheels based on the above calculations, using formulas for Mecanum wheels.
            FrontLeftWheel.setPower(FastStraight + Straight + FastStrafe + Strafe + Rotate);
            FrontRightWheel.setPower(FastStraight + Straight - FastStrafe - Strafe - Rotate);
            BackLeftWheel.setPower(FastStraight + Straight - FastStrafe - Strafe + Rotate);
            BackRightWheel.setPower(FastStraight + Straight + FastStrafe + Strafe - Rotate);
        }
        else {
            // Stop all motors if their controls are not touched
            FrontLeftWheel.setPower(0);
            FrontRightWheel.setPower(0);
            BackLeftWheel.setPower(0);
            BackRightWheel.setPower(0);
        }
    }

    public void OptimizedTeleopDrive()
    {
        RightStickYValue = -myOpMode.gamepad1.right_stick_y;
        RightStickXValue = myOpMode.gamepad1.right_stick_x;
        LeftStickYValue = -myOpMode.gamepad1.left_stick_y;
        LeftStickXValue = myOpMode.gamepad1.left_stick_x;
        TriggersValue = myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger;
        if (myOpMode.gamepad1.right_bumper) {
            BumpersValue = rotateLowSpeedDrive;
        }
        else if (myOpMode.gamepad1.left_bumper) {
            BumpersValue = -rotateLowSpeedDrive;
        }
        else {
            BumpersValue = 0;
        }

        // Use a third-degree polynomial function on fast movements for better control, less important on slow movements
        // Calculate each variable only when the joystick value is higher than the DeadZone number, otherwise make it 0
        FastStraight = abs(RightStickYValue) > BeethovenHardware.Drive.DEAD_ZONE ? (float)(highSpeedDrive * (0.75 * pow(RightStickYValue, 3) + 0.25 * RightStickYValue)) : 0;
        Straight = abs(LeftStickYValue) > BeethovenHardware.Drive.DEAD_ZONE ? (float)(lowSpeedDrive * LeftStickYValue) : 0;
        FastStrafe = abs(RightStickXValue) > BeethovenHardware.Drive.DEAD_ZONE ? (float)(highSpeedDrive * (0.75 * pow(RightStickXValue, 3) + 0.25 * RightStickXValue)) : 0;
        Strafe = abs(LeftStickXValue) > BeethovenHardware.Drive.DEAD_ZONE ? (float)(lowSpeedDrive * LeftStickXValue) : 0;
        FastRotate = abs(TriggersValue) > BeethovenHardware.Drive.DEAD_ZONE ? (float)(rotateSpeedDrive * (0.75 * pow(TriggersValue, 3) + 0.25 * TriggersValue)): 0;
        Rotate = BumpersValue;

        FrontLeftPower = FastStraight + Straight + FastStrafe + Strafe + FastRotate + Rotate;
        FrontRightPower = FastStraight + Straight - FastStrafe - Strafe - FastRotate - Rotate;
        BackLeftPower = FastStraight + Straight - FastStrafe - Strafe + FastRotate + Rotate;
        BackRightPower = FastStraight + Straight + FastStrafe + Strafe - FastRotate - Rotate;

        // If any power exceeds 1.0, determine the maximum between them then normalize all using that maximum
        double maxPower = max(abs(FrontLeftPower),
                max(abs(FrontRightPower),
                        max(abs(BackLeftPower), abs(BackRightPower))));
        if (maxPower > 1.0) {
            FrontLeftPower /= maxPower;
            FrontRightPower /= maxPower;
            BackLeftPower /= maxPower;
            BackRightPower /= maxPower;
        }

        FrontLeftWheel.setPower(FrontLeftPower);
        FrontRightWheel.setPower(FrontRightPower);
        BackLeftWheel.setPower(BackLeftPower);
        BackRightWheel.setPower(BackRightPower);
    }

    /** Move forward(+) or backwards(-) until reaching Position */
    public void MoveStraight(int mmToTarget, double VelocityPercentage, int WaitTime)
    {
        // Ticks to move = (distance to move / wheels circumference) * ticks for 1 full turn of wheel
        double TicksToTarget = (mmToTarget / (BeethovenHardware.Drive.WHEEL_DIAMETER_MM * Math.PI)) * BeethovenHardware.Drive.MOTOR_TICKS_PER_REV;
        // Ticks per second = Motor's revolutions per second * ticks for 1 full turn of wheel, then use a fraction/percentage of that
        double TicksPerSecond = VelocityPercentage * ((double)(BeethovenHardware.Drive.MOTOR_RPM / 60) * BeethovenHardware.Drive.MOTOR_TICKS_PER_REV);

        FrontLeftWheel.setTargetPosition((int)(FrontLeftWheel.getCurrentPosition() + TicksToTarget));
        FrontRightWheel.setTargetPosition((int)(FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int)(BackLeftWheel.getCurrentPosition() + TicksToTarget));
        BackRightWheel.setTargetPosition((int)(BackRightWheel.getCurrentPosition() + TicksToTarget));

        // The (DcMotorEx) is called casting.  It says take the FrontLeftWheel, and while we know it's a DcMotor
        //   treat it like a DcMotorEx.  DcMotorEx has more functionality than DcMotor (such as setVelocity).  Not all hardware
        //   can be treated as a DcMotorEx, but ours can.  DcMotorEx inherits from DcMotor, so it has all the
        //   functionality of DcMotor plus more.  We should figure out if we can just declare these as DcMotorEx in
        //   the first place.  After tournaments.
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (myOpMode.opModeIsActive() &&
                FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    /** Strafes right until reaching Position */
    public void StrafeRight(int mmToTarget, double VelocityPercentage, int WaitTime)
    {
        // Ticks to move = (distance to move / wheels circumference) * ticks for 1 full turn of wheel. Apply a compensation for friction impact to strafing
        double TicksToTarget = ((mmToTarget / (BeethovenHardware.Drive.WHEEL_DIAMETER_MM * Math.PI)) * BeethovenHardware.Drive.MOTOR_TICKS_PER_REV) * BeethovenHardware.Drive.STRAFE_COMPENSATION;
        // Ticks per second = Motor's revolutions per second * ticks for 1 full turn of wheel, then use a fraction/percentage of that
        double TicksPerSecond = VelocityPercentage * ((double)(BeethovenHardware.Drive.MOTOR_RPM / 60) * BeethovenHardware.Drive.MOTOR_TICKS_PER_REV);

        FrontLeftWheel.setTargetPosition((int)(FrontLeftWheel.getCurrentPosition() + TicksToTarget));
        FrontRightWheel.setTargetPosition((int)(FrontRightWheel.getCurrentPosition() - TicksToTarget));
        BackLeftWheel.setTargetPosition((int)(BackLeftWheel.getCurrentPosition() - TicksToTarget));
        BackRightWheel.setTargetPosition((int)(BackRightWheel.getCurrentPosition() + TicksToTarget));

        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        //  myOpMode.telemetry.addData()

        while (myOpMode.opModeIsActive() &&
                FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    /** Strafes left until reaching Position */
    public void StrafeLeft(int mmToTarget, double VelocityPercentage, int WaitTime)
    {
        StrafeRight(-mmToTarget, VelocityPercentage, WaitTime);
    }

    /** Rotates left until reaching Position */
    public void RotateLeft(double degrees, double VelocityPercentage, int WaitTime)
    {
        // Convert degrees to a distance in mm
        double mmToTarget = degrees * (BeethovenHardware.Drive.MM_PER_90_DEGREES_ROTATION / 90.0);
        // Uses the formula we've always had for rotation
        // Ticks to move = (distance to move / wheels circumference) * ticks for 1 full turn of wheel
        double TicksToTarget = (mmToTarget / (BeethovenHardware.Drive.WHEEL_DIAMETER_MM * Math.PI)) * BeethovenHardware.Drive.MOTOR_TICKS_PER_REV;
        // Ticks per second = Motor's revolutions per second * ticks for 1 full turn of wheel, then use a fraction/percentage of that
        double TicksPerSecond = VelocityPercentage * ((double)(BeethovenHardware.Drive.MOTOR_RPM / 60) * BeethovenHardware.Drive.MOTOR_TICKS_PER_REV);

        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() - TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() - TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));

        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (myOpMode.opModeIsActive() &&
                FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    /** Rotates right until reaching Position */
    public void RotateRight(double degree, double VelocityPercentage, int WaitTime)
    {
        RotateLeft(-1*degree, VelocityPercentage, WaitTime);
    }

    public void setTurnPower(double power)
    {
        FrontLeftWheel.setPower(-power);
        BackLeftWheel.setPower(-power);
        FrontRightWheel.setPower(power);
        BackRightWheel.setPower(power);
    }

    // Stop all motors
    public void stopMotors()
    {
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }

    // this function added to work with husky lens for ball tracking but can be used for other command driven movement.
    public void DriveRobotCentric(double forward, double strafe, double rotate)
    {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double maxPower = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
        if (maxPower > 1.0) { fl /= maxPower; fr /= maxPower; bl /= maxPower; br /= maxPower; }

        FrontLeftWheel.setPower(fl);
        FrontRightWheel.setPower(fr);
        BackLeftWheel.setPower(bl);
        BackRightWheel.setPower(br);
    }

}