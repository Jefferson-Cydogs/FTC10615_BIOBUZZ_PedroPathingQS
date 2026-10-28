package org.firstinspires.ftc.teamcode.cydogs.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * BeethovenHardware: the single configuration interface between the physical robot and the code.
 *
 * Everything that is a hardcoded property of the robot lives here: hardware-map names,
 * directions, zero-power behavior, limits, preset positions, and preset speeds. Mechanism
 * classes should reference these constants and never hold a literal name or number of their own.
 *
 * One static nested class per device or mechanism, e.g. BeethovenHardware.Launcher.MOTOR_NAME.
 *
 * All values below were taken from the code as it stood in the repo when this file was written.
 * Where two files disagreed about the same device, the conflict is called out with "CONFLICT".
 */
public final class BeethovenHardware {

    private BeethovenHardware() {} // constants only, never instantiated

    // ------------------------------------------------------------------
    // Drivetrain (mecanum) - used by BioBuzzRobotChassis
    // ------------------------------------------------------------------
    public static final class Drive {
        private Drive() {}

        public static final String FRONT_LEFT_NAME  = "FrontLeftWheel";
        public static final String FRONT_RIGHT_NAME = "FrontRightWheel";
        public static final String BACK_LEFT_NAME   = "BackLeftWheel";
        public static final String BACK_RIGHT_NAME  = "BackRightWheel";

        public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION   = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION  = DcMotorSimple.Direction.FORWARD;

        // Zero-power behavior: coast while driving in teleop, brake in autonomous
        public static final DcMotor.ZeroPowerBehavior TELEOP_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
        public static final DcMotor.ZeroPowerBehavior AUTON_ZERO_POWER_BEHAVIOR   = DcMotor.ZeroPowerBehavior.BRAKE;
        public static final DcMotor.ZeroPowerBehavior TELEOP_BRAKE_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE; // ChassisTeleopBrakeWheels

        // Physical measurements (GoBilda Yellow Jacket 312 RPM, 537.7 PPR; 104 mm mecanum wheels)
        public static final int    WHEEL_DIAMETER_MM = 104;
        public static final int    MOTOR_RPM = 312;
        public static final double MOTOR_TICKS_PER_REV = 537.7;
        public static final double MM_PER_90_DEGREES_ROTATION = 785; // depends on chassis size
        public static final double STRAFE_COMPENSATION = 1.081;      // friction correction when strafing

        // Per-wheel velocity PIDF "F" term (P, I, D are 0), used in autonomous RUN_TO_POSITION
        public static final double FRONT_LEFT_VELOCITY_F  = 11.754006;
        public static final double FRONT_RIGHT_VELOCITY_F = 10.175712;
        public static final double BACK_LEFT_VELOCITY_F   = 11.752998;
        public static final double BACK_RIGHT_VELOCITY_F  = 11.398002;

        // Teleop speeds and joystick shaping
        public static final double DEAD_ZONE = 0.05;
        public static final double HIGH_SPEED = 0.7;
        public static final double LOW_SPEED = 0.3;
        public static final double ROTATE_SPEED = 0.6;
        public static final double ROTATE_LOW_SPEED = 0.3; // bumper rotation
    }

    // ------------------------------------------------------------------
    // Intake: roller motor plus two continuous-rotation side rotators (Intake.java)
    // ------------------------------------------------------------------
    public static final class Intake {
        private Intake() {}

        public static final String LEFT_SERVO_NAME  = "LeftIntakeServo";
        public static final String RIGHT_SERVO_NAME = "RightIntakeServo";
        public static final String MOTOR_NAME       = "IntakeMotor";

        public static final DcMotorSimple.Direction LEFT_SERVO_DIRECTION  = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction RIGHT_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction MOTOR_DIRECTION       = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

        public static final double SERVO_POWER         = 0.8;
        public static final double MOTOR_INTAKE_POWER  = 0.75;
        public static final double MOTOR_REVERSE_POWER = -0.4;
    }

    // ------------------------------------------------------------------
    // Launcher: velocity-controlled flywheel plus a speed LED (Launcher.java)
    // ------------------------------------------------------------------
    public static final class Launcher {
        private Launcher() {}

        public static final String MOTOR_NAME = "LauncherMotor";
        public static final String LED_NAME   = "LauncherSpeedLED";

        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

        // Specs for typical launcher motor: GoBILDA 5203 Series Yellow Jacket 312 RPM, 537.7 PPR
        //    Without gearbox, theoretical 6,000RPM, empirical needs to be calculated.
        //    PPR = Ticks/second is always 28 at the motor shaft for goBILDA Yellow Jacket motors
        // Max velocity = (RPM / 60) * ticks per rev
        public static final double MOTOR_RPM = 5334; //Calculated with code for specific motor
        public static final double MOTOR_TICKS_PER_REV = 28;
        public static final double MAX_TICKS_PER_SECOND = (MOTOR_RPM / 60) * MOTOR_TICKS_PER_REV;

        // Launch speed as a fraction of top speed, and the size of one D-pad adjustment
        public static final double LAUNCH_PERCENT = 0.4;
        public static final double PERCENT_STEP = 0.01;

        // Controlled spin-down
        public static final double SPINDOWN_TPS_PER_SEC = 35000;
        public static final double SPINDOWN_MIN_TPS = 1000;
    }

    // ------------------------------------------------------------------
    // Loader servo: pushes one element into the launcher (LoaderServo.java)
    // ------------------------------------------------------------------
    public static final class Loader {
        private Loader() {}

        public static final String NAME = "LaunchLoaderServo";
        public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        public static final double MIN_POSITION = 0.0;
        public static final double MAX_POSITION = 1.0;
        public static final double STARTING_POSITION = 0.5;

        public static final double LOAD_POSITION = 0.62;
        public static final double LAUNCH_POSITION = 0.96;

        public static final double LAUNCH_HOLD_SECONDS = 0.3;
    }

    // ------------------------------------------------------------------
    // Launcher size servo: switches between nectar and pollen (LauncherSizeServo.java)
    // ------------------------------------------------------------------
    public static final class LauncherSize {
        private LauncherSize() {}

        public static final String NAME = "LauncherSizeServo";
        public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        public static final double MIN_POSITION = 0.02;
        public static final double MAX_POSITION = 0.9;
        public static final double STARTING_POSITION = 0.5;

        public static final double NECTAR_POSITION = 0.16;
        public static final double POLLEN_POSITION = 0.64;
    }

    // ------------------------------------------------------------------
    // Turret servo (Turret.java)
    // ------------------------------------------------------------------
    public static final class Turret {
        private Turret() {}

        public static final String NAME = "TurretTurnerServo";
        public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        // Servo positions (safe range from testing)
        public static final double MIN_POSITION = 0.35;
        public static final double STRAIGHT_POSITION = 0.5;
        public static final double MAX_POSITION = 0.65;

        // Robot angles in degrees: 0 = intake, 180 = straight ahead for aiming
        public static final double STRAIGHT_DEGREES = 180.0;
        public static final double MAX_OFFSET_DEGREES = 50.0;
        public static final double MIN_DEGREES = STRAIGHT_DEGREES - MAX_OFFSET_DEGREES;
        public static final double MAX_DEGREES = STRAIGHT_DEGREES + MAX_OFFSET_DEGREES;
    }

    // ------------------------------------------------------------------
    // Conveyors that move elements through the robot
    // ------------------------------------------------------------------
    public static final class MoveToMiddle {
        private MoveToMiddle() {}

        public static final String NAME = "MoveToMiddleMotor";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
        public static final double RUN_POWER = 0.5;
    }

    public static final class MoveToLauncher {
        private MoveToLauncher() {}

        public static final String NAME = "MoveToLauncherMotor";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
        public static final double RUN_POWER = 0.1; // power while a bumper is held in BioBuzzTeleop (negative for reverse)
    }

    // ------------------------------------------------------------------
    // Flower mechanisms
    // ------------------------------------------------------------------

    /** Motor that lifts the U-channel to flower height (FlowerSlider.java). */
    public static final class FlowerSlider {
        private FlowerSlider() {}

        public static final String NAME = "FlowerSlider";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

        public static final int HOME_TICKS = 0;
        public static final int FLOWER_TICKS = 300;

        public static final double RAISE_POWER = 0.7;
        public static final double LOWER_POWER = 0.4;
    }

    /** Gate that opens to drop elements into the flower (FlowerScoringServo.java). */
    public static final class FlowerScoring {
        private FlowerScoring() {}

        public static final String NAME = "FlowerScoringServo";
        public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        public static final double MIN_POSITION = 0.0;
        public static final double MAX_POSITION = 1.0;

        public static final double CLOSED_POSITION = 0.5;
        public static final double OPEN_POSITION = 0.8;
    }

    /** Three-bar dump arm built from three positional servos (DumpArm.java). */
    public static final class DumpArm {
        private DumpArm() {}

        // CONFLICT: the old notes call these lower_flower_servo / medium_flower_servo / upper_flower_servo
        public static final String LOWER_NAME  = "lowerflower";
        public static final String MEDIUM_NAME = "midflower";
        public static final String UPPER_NAME  = "upperflower";

        public static final Servo.Direction LOWER_DIRECTION  = Servo.Direction.FORWARD;
        public static final Servo.Direction MEDIUM_DIRECTION = Servo.Direction.FORWARD;
        public static final Servo.Direction UPPER_DIRECTION  = Servo.Direction.FORWARD;

        // Degrees of travel for position 0.0 to 1.0 (180 for standard servos)
        public static final double SERVO_RANGE_DEGREES = 180.0;
        public static final double MIN_POSITION = 0.02;
        public static final double MAX_POSITION = 0.98;

        // Where 0 degrees sits on each servo (position 0.0 to 1.0): {lower, medium, upper}
        public static final double[] ZERO_POSITIONS = {0.2, 0.2, 0.2};

        // Joint angles in degrees: {lower, medium, upper}
        public static final double[] REST_ANGLES = {0, 35, 10};
        public static final double[] DUMP_ANGLES = {80, 110, 30};

        // Seconds for a full move
        public static final double RAISE_SECONDS = 1.5;
        public static final double LOWER_SECONDS = 1.5;
    }

    // ------------------------------------------------------------------
    // Vision
    // ------------------------------------------------------------------

    /** HuskyLens color tracking for pollen and nectar (BallTracker.java). */
    public static final class HuskyLens {
        private HuskyLens() {}

        public static final String NAME = "HuskyLens";

        // IDs match the order the colors were trained on the camera
        public static final int YELLOW_ID = 1;
        public static final int RED_ID = 2;
        public static final int BLUE_ID = 3;

        public static final int FRAME_WIDTH = 320;
        public static final int FRAME_HEIGHT = 240;
        public static final int CENTER_X = 160;

        public static final double READ_INTERVAL_MS = 50;
    }

    /** Color/distance sensors that identify the elements in the magazine (FlingyMag.java). */
    public static final class FlingyMag {
        private FlingyMag() {}

        public static final String INTAKE_SENSOR_RIGHT_NAME = "IntakeSensorRight";
        public static final String INTAKE_SENSOR_LEFT_NAME  = "IntakeSensorLeft";
        public static final String LAUNCHER_SENSOR_NAME     = "LauncherSensor";
    }


    /** AprilTag webcam (TagSight.java). */
    public static final class Webcam {
        private Webcam() {}

        public static final String NAME = "Webcam 1";
    }












}