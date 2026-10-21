package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseContinuousServo;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePowerMotor;

/**
 * Intake - the mechanism that pulls scoring elements into the robot.
 *
 * WHAT THE INTAKE IS MADE OF
 *   Three moving parts that must work together:
 *     1) A main ROLLER, spun by a motor that we control with plain power (no encoder counting).
 *     2) A LEFT side rotator, a continuous-rotation servo that turns inward toward the center.
 *     3) A RIGHT side rotator, another continuous-rotation servo that also turns inward.
 *   The two side rotators are mirror images, so one has to spin "the other way" in the
 *   robot's wiring in order for BOTH to push elements inward. That is why each part gets its own
 *   direction setting below.
 *
 * "IS-A" VERSUS "HAS-A" (the big new idea in this class)
 *   In LoaderServo and FlowerSlider, our class WAS a kind of another class (that is what the
 *   word "extends" means). An Intake is different: it is not a servo and it is not a motor. It
 *   HAS three parts, and each part is an object we already have a class for:
 *       - BaseContinuousServo (for each side rotator)
 *       - BasePowerMotor      (for the roller)
 *   So this class does NOT say "extends". Instead, it keeps the three parts in variables
 *   (programmers call those variables "properties" or "fields"), and its methods tell the three
 *   parts what to do. This is called COMPOSITION.
 *
 * WHAT WE ADD
 *   The settings for all three parts, plus FOUR methods: Initialize, RunIntake, ReverseIntake
 *   and StopIntake.
 *
 * WHY THE SETTINGS LIVE HERE
 *   The OpMode (the program the drivers run) should not have to know any of the settings, or
 *   that there are even three parts. It will only say "make an intake", "run it", "reverse it"
 *   and "stop it". If the wiring or a name ever changes, we fix it in this ONE file.
 *
 * HOW AN OPMODE WILL USE YOUR CLASS (you do not write this part - just understand it)
 *   1. It makes an Intake and hands it the OpMode (the only thing it passes in).
 *   2. It calls your Initialize method once.
 *   3. When the driver presses buttons, it calls RunIntake, ReverseIntake or StopIntake.
 *
 * YOUR JOB: finish Steps 1 through 6 below. Everything you write goes INSIDE the curly braces
 * of the class, in the spaces marked for each step. Do the steps in order and save after each
 * one.
 */
public class Intake {


    // =========================================================================================
    // STEP 1: THE SETTINGS (constants)
    // =========================================================================================
    // Make ten constants. A constant is a variable that never changes after it is created.
    // Putting a number or name in ONE place with a good label is much better than typing the
    // same mysterious value in several places (programmers call those "magic numbers").
    //
    // Every constant must be:
    //   - private  (only this class needs to see it)
    //   - static   (it belongs to the class itself, not to one particular Intake; this is also
    //               what lets you use it in Step 3)
    //   - final    (this is the word that makes it impossible to change later)
    // Name them in ALL_CAPITAL_LETTERS with underscores between words. That is the Java habit
    // for constants. Look at the top of DumpArm.java for examples of how they look.
    //
    // Make these ten, giving each a short comment explaining what it is:
    //
    // The three hardware names - each is a String (text) and must match, letter for letter and
    // capital for capital, the name in the Robot Configuration on the Driver Hub. If one is
    // misspelled the robot will crash at init and tell you it "unable to find a hardware
    // device with name ...".
    //   a) LEFT_SERVO_NAME   - the text must be exactly:  LeftIntakeServo
    //   b) RIGHT_SERVO_NAME  - the text must be exactly:  RightIntakeServo
    //   c) MOTOR_NAME        - the text must be exactly:  Intake
    //
    // The three directions. The type of the servos' constants is DcMotorSimple.Direction, and
    // so is the motor's (that is why we imported DcMotorSimple above). Each one is either
    // FORWARD or REVERSE.
    //   d) LEFT_SERVO_DIRECTION  - set it to REVERSE
    //   e) RIGHT_SERVO_DIRECTION - set it to FORWARD
    //   f) MOTOR_DIRECTION       - set it to REVERSE
    //
    //   g) MOTOR_ZERO_POWER_BEHAVIOR - the type is DcMotor.ZeroPowerBehavior (that is why we
    //      imported DcMotor above). Set it to FLOAT, which lets the roller coast to a stop
    //      when we stop giving it power.
    //
    // The three powers. Each is a double (a number with a decimal point). Power goes from -1
    // (full speed one way) through 0 (stopped) to 1 (full speed the other way).
    //   h) SERVO_POWER          - how fast BOTH side rotators turn. Set it to 0.7.
    //   i) MOTOR_INTAKE_POWER   - how fast the roller pulls elements IN. Set it to 0.6.
    //   j) MOTOR_REVERSE_POWER  - how fast the roller pushes elements OUT. Set it to -0.4.
    //      Notice it is NEGATIVE. A negative power means "run backwards".
    //
    // We will tune these numbers later by testing on the robot.
    //
    private final String LEFT_SERVO_NAME= "LeftIntakeServo";
    private final String RIGHT_SERVO_NAME= "RightIntakeServo";
    private final String MOTOR_NAME= "IntakeMotor";
    private final DcMotorSimple.Direction LEFT_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction RIGHT_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private final DcMotor.ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
    private final double SERVO_POWER = 0.7;
    private final double MOTOR_INTAKE_POWER = 0.6;
    private final double MOTOR_REVERSE_POWER = -0.4;

    // =========================================================================================
    // STEP 2: THE PROPERTIES (the three parts)
    // =========================================================================================
    // Now make the three variables that will hold the parts of the intake. These are NOT
    // constants and they are NOT static: each Intake gets its own three parts.
    //
    // Each one must be:
    //   - private  (only this class needs to touch them)
    //   - final    (once it is created, it is never swapped for a different one)
    //
    // Make these three, with a short comment on each:
    //   a) leftServo   - its type is BaseContinuousServo
    //   b) rightServo  - its type is BaseContinuousServo
    //   c) rollerMotor - its type is BasePowerMotor
    //
    // Variables like these start out EMPTY. Because they are marked final, Java will insist that
    // you fill each one in the constructor (Step 3), and it will show a red underline until you
    // do. That is Java protecting you from forgetting.
    //
    // (Write your three properties below this line.)
    private final BaseContinuousServo leftServo;
    private final BaseContinuousServo rightServo;
    private final BasePowerMotor rollerMotor;

    // =========================================================================================
    // STEP 3: THE CONSTRUCTOR
    // =========================================================================================
    // A constructor is the special block of code that runs when someone creates an Intake.
    // Look at the constructors inside BaseContinuousServo.java and BasePowerMotor.java to see
    // what they take, because you are about to build one of each.
    //
    // Your constructor must:
    //   - be public
    //   - have EXACTLY the same name as the class (including the capital letters), and NO
    //     return type, not even void
    //   - take ONE parameter: an OpMode (that is why we imported OpMode above). Name the
    //     parameter opMode.
    //
    // Inside the constructor, fill in your three properties from Step 2, one at a time. To make
    // a new part, use the Java word  new  followed by the part's class name and parentheses with
    // the things it needs inside them. The things go in exactly the order the part's own
    // constructor lists them.
    //
    //   - leftServo:   a new BaseContinuousServo, given these four things in order:
    //                    the opMode, LEFT_SERVO_NAME, LEFT_SERVO_DIRECTION, SERVO_POWER
    //   - rightServo:  a new BaseContinuousServo, given these four things in order:
    //                    the opMode, RIGHT_SERVO_NAME, RIGHT_SERVO_DIRECTION, SERVO_POWER
    //   - rollerMotor: a new BasePowerMotor, given these four things in order:
    //                    the opMode, MOTOR_NAME, MOTOR_DIRECTION, MOTOR_ZERO_POWER_BEHAVIOR
    //
    // Do NOT call Initialize on the parts here; that happens in Step 4.
    //
    // (Write your constructor below this line.)
    public Intake(OpMode opMode){
         leftServo = new BaseContinuousServo (opMode,LEFT_SERVO_NAME,LEFT_SERVO_DIRECTION,SERVO_POWER);
        rightServo = new BaseContinuousServo (opMode,RIGHT_SERVO_NAME,RIGHT_SERVO_DIRECTION,SERVO_POWER);
        rollerMotor = new BasePowerMotor (opMode,MOTOR_NAME,MOTOR_DIRECTION,MOTOR_ZERO_POWER_BEHAVIOR);
    }

    // =========================================================================================
    // STEP 4: THE Initialize METHOD
    // =========================================================================================
    // In LoaderServo we got Initialize for free by inheriting it. This class inherits nothing,
    // so it does not have an Initialize method unless YOU write one. The OpMode will call it.
    // Its job is to pass the message along: tell each of the three parts to get itself ready.
    //
    // Your method must:
    //   - be public
    //   - return nothing (the word for that is void)
    //   - be named Initialize  (capital I)
    //   - take NO parameters
    //
    // Inside it there are THREE lines, one for each part. On each of your three properties,
    // call that part's own Initialize method.
    //
    // If you leave one out, the part will crash the program the first time you try to use it,
    // with a message saying it was used before Initialize was called.
    //
    // (Write your Initialize method below this line.)
    public void Initialize(){
       leftServo.Initialize();
       rightServo.Initialize();
       rollerMotor.Initialize();

    }

    // =========================================================================================
    // STEP 5: THE RunIntake METHOD
    // =========================================================================================
    // This is the method that pulls elements in. Look at the Stop method inside
    // BaseContinuousServo.java to see what a method with no parameters looks like.
    //
    // Your method must:
    //   - be public
    //   - return nothing (void)
    //   - be named RunIntake
    //   - take NO parameters
    //
    // Inside it there are THREE lines, one for each part:
    //   - tell the left servo to run forward. BaseContinuousServo has a method for that with
    //     "Forward" in its name, and it needs nothing passed to it: the servo already knows its
    //     power from the constructor. (Forward just means "the direction we set up as forward";
    //     the DIRECTION constants are what make the two sides turn inward.)
    //   - do the same for the right servo
    //   - tell the roller motor to run at your MOTOR_INTAKE_POWER. The method that does this is
    //     in BasePowerMotor.java, and this one DOES need the power passed to it.
    //
    // Add a comment above the method in your own words saying what it does.
    //
    // (Write your RunIntake method below this line.)
    public void RunIntake(){
       leftServo.RunForward();
       rightServo.RunForward();
       rollerMotor.SetPower(MOTOR_INTAKE_POWER);
    }

    // =========================================================================================
    // STEP 6: THE ReverseIntake AND StopIntake METHODS
    // =========================================================================================
    // Two more methods, written the same way as RunIntake: public, void, no parameters.
    //
    // ReverseIntake - used to spit elements back out. Inside it there are THREE lines:
    //   - the left servo does NOT run at all: tell it to stop
    //   - the right servo does NOT run at all: tell it to stop
    //   - the roller motor runs BACKWARDS: tell it to run at your MOTOR_REVERSE_POWER
    // (The side rotators only know how to push elements inward, so for spitting out we turn
    // them off and let the roller do all the work.)
    //
    // StopIntake - used when the driver lets go of the button. Inside it there are THREE lines:
    // tell each of the three parts to stop. Each of the parts has a method for stopping.
    //
    // Add a comment above each method in your own words.
    //
    // (Write your ReverseIntake and StopIntake methods below this line.)
    public void ReverseIntake (){
        leftServo.Stop();
        rightServo.Stop();
        rollerMotor.SetPower(MOTOR_REVERSE_POWER);
    }

    public void StopIntake(){
        leftServo.Stop();
        rightServo.Stop();
        rollerMotor.Stop();
    }
    // =========================================================================================
    // CHECK YOUR WORK
    // =========================================================================================
    //   [ ] This class does NOT say "extends" anywhere. (It HAS its parts; it is not one of them.)
    //   [ ] You have ten constants, each marked private static final, in ALL_CAPS.
    //   [ ] You have three properties, each marked private final, in camelCase (first word
    //       starts small, like leftServo).
    //   [ ] The constructor is spelled exactly like the class and takes one OpMode, and it fills
    //       in all three properties using  new.
    //   [ ] Initialize, RunIntake, ReverseIntake and StopIntake each have no parameters and
    //       each call something on all three parts.
    //   [ ] Every line of code ends with a semicolon, and every opening { has a closing }.
    //   [ ] The program builds with no red underlines.
    //
    // Common mistakes:
    //   - A red underline on a property: it was never filled in inside the constructor.
    //   - A crash that says something was used "before Initialize()": one of the three lines in
    //     your Initialize method is missing, or the OpMode never called Initialize.
    //   - A crash that mentions a "null object reference": a property was declared but never
    //     given a value with  new.
    //   - "Cannot find symbol": a name is misspelled, or has the wrong capital letters. Java
    //     treats  runIntake  and  RunIntake  as two different words.
    //   - A side rotator pushes elements OUT: its DIRECTION constant is the opposite of what it
    //     should be.
    //   - The robot crashes at init: a hardware name does not match the Robot Configuration.
    //
    // OPTIONAL CHALLENGE (only after everything above works)
    //   Add a method called RunRollerOnly that runs ONLY the main roller inward and keeps both
    //   side rotators stopped. It is a mix of the lines you already wrote in RunIntake and
    //   ReverseIntake.
    public void RunRollerOnly(){
        rightServo.Stop();
        leftServo.Stop();
        rollerMotor.SetPower(MOTOR_INTAKE_POWER);
    }
}