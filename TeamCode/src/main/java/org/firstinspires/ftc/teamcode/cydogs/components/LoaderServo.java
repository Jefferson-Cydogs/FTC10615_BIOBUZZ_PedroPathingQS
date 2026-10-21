package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseContinuousServo;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;

/**
 * LoaderServo - the little servo that pushes a scoring ball into the launcher.
 *
 * WHAT THIS CLASS IS
 *   A LoaderServo is a special kind of BaseContinuousServo. BaseContinuousServo already knows how
 *   to talk to a continuous-rotation servo: start it, stop it, and even run it for a number of
 *   seconds and then stop it automatically. The word for this in Java is INHERITANCE: our class
 *   gets all of that for free, and we only add the things that are specific to the loader.
 *
 * WHAT WE ADD
 *   The robot's settings for THIS servo (its name, which way it spins, how fast) and ONE method
 *   called Load that pushes a ball into the launcher.
 *
 * WHY THE SETTINGS LIVE HERE
 *   The OpMode (the program the drivers run) should not have to know any of the settings. A
 *   driver's OpMode will only say "make a loader" and "load a ball". If the servo is ever
 *   renamed or re-wired, we fix it in this ONE file and every OpMode keeps working.
 *
 * HOW AN OPMODE WILL USE YOUR CLASS (you do not write this part - just understand it)
 *   1. It makes a LoaderServo and hands it the OpMode (that is the only thing it passes in).
 *   2. It calls Initialize once. You do NOT write Initialize - you inherit it.
 *   3. When the driver presses a button, it calls your Load method ONCE for that press. The
 *      servo starts, and a timer running in the background stops it when the time is up. The
 *      OpMode does not have to do anything else - it never has to stop the servo itself.
 *
 * YOUR JOB: finish Steps 1, 2 and 3 below. Everything you write goes INSIDE the curly braces of
 * the class, in the spaces marked for each step. Do the steps in order and save after each one.
 */
public class LoaderServo extends BaseServo {
        private static final String HARDWARE_NAME = "LaunchLoaderServo";
        private static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;
        // Safety limits: BaseServo never lets a position go outside this range.
        private static final double MIN_POSITION = 0.0;
        private static final double MAX_POSITION = 1.0;

        // The two gate positions, measured on the robot. Both must be between MIN and MAX.
        private static final double LOAD_POSITION = 0.26;
        private static final double LAUNCH_POSITION = 0.68;

        public LoaderServo(OpMode opMode)
        {
            super(opMode, HARDWARE_NAME,DIRECTION, MIN_POSITION, MAX_POSITION, .5);

        }

        public void Load()
        {
                this.SetPosition(LOAD_POSITION);
        }
        public void Launch()
        {
                this.SetPosition(LAUNCH_POSITION);
        }

        // How long to hold the Launch position before going back to Load (seconds). Tune on the robot.
        private static final double LAUNCH_HOLD_SECONDS = 0.8;

        private final ElapsedTime launchTimer = new ElapsedTime();
        private boolean launching = false;

        /** One push: go to Launch now, then Update() returns to Load after LAUNCH_HOLD_SECONDS.
         *  Ignored if a launch is already in progress. */
        public void LaunchOnce()
        {
                if (launching) {
                        return;
                }
                Launch();
                launchTimer.reset();
                launching = true;
        }

        /** Call every loop. Returns the servo to Load when the hold time is up. */
        public void Update()
        {
                if (launching && launchTimer.seconds() >= LAUNCH_HOLD_SECONDS) {
                        Load();
                        launching = false;
                }
        }

        public boolean IsLaunching()
        {
                return launching;
        }



    // =========================================================================================
    // STEP 1: THE SETTINGS (constants)
    // =========================================================================================
    // Make four constants. A constant is a variable that never changes after it is created.
    // Putting a number or name in ONE place with a good label is much better than typing the
    // same mysterious value in several places (programmers call those "magic numbers").
    //
    // Every constant must be:
    //   - private  (only this class needs to see it)
    //   - static   (it belongs to the class itself, not to one particular LoaderServo; this is
    //               also what lets you use it in Step 2)
    //   - final    (this is the word that makes it impossible to change later)
    // Name them in ALL_CAPITAL_LETTERS with underscores between words. That is the Java habit
    // for constants. Look at the top of DumpArm.java for examples of how they look.
    //
    // Make these four, giving each a short comment explaining what it is:
    //
    //   a) HARDWARE_NAME - a String (text). The text must be exactly:  LaunchLoaderServo
    //      It has to match, letter for letter and capital for capital, the name of the servo in
    //      the Robot Configuration on the Driver Hub. If it is misspelled the robot will crash
    //      at init and tell you it "unable to find a hardware device with name ...".
    //
    //   b) DIRECTION - the type is DcMotorSimple.Direction (that is why we imported
    //      DcMotorSimple above). Set it to REVERSE. This is what makes the servo's "forward"
    //      spin push the ball TOWARD the launcher instead of away from it.
    //
    //   c) RUN_POWER - a double (a number with a decimal point). Set it to 0.6. Power goes
    //      from 0 (stopped) to 1 (full speed).
    //
    //   d) LOAD_SECONDS - a double. Set it to 2.0. This is how long the servo runs each time we
    //      load a ball.
    //
    // (Write your four constants below this line.)


    // =========================================================================================
    // STEP 2: THE CONSTRUCTOR
    // =========================================================================================
    // A constructor is the special block of code that runs when someone creates a LoaderServo.
    // Open BaseContinuousServo.java and look at its constructor to see what one looks like.
    //
    // Your constructor must:
    //   - be public
    //   - have EXACTLY the same name as the class (including the capital letters), and NO
    //     return type, not even void
    //   - take ONE parameter: an OpMode (that is why we imported OpMode above). Name the
    //     parameter opMode.
    //
    // Inside the constructor there is only ONE thing to do, and it must be the very first
    // line: call the PARENT class's constructor using the Java word  super  followed by
    // parentheses. Inside those parentheses, pass these four things, in exactly this order,
    // separated by commas:
    //     1) the opMode that was handed to your constructor
    //     2) your HARDWARE_NAME constant
    //     3) your DIRECTION constant
    //     4) your RUN_POWER constant
    // Compare with the parameter list of the BaseContinuousServo constructor: your four things
    // must line up with its four parameters, in the same order.
    //
    // Do NOT put anything else in the constructor. In particular, do not call Initialize here;
    // the OpMode does that.
    //
    // (Write your constructor below this line.)


    // =========================================================================================
    // STEP 3: THE LOAD METHOD
    // =========================================================================================
    // Now write the method the driver's button will call. Look at the Stop method inside
    // BaseContinuousServo.java to see what a method with no parameters looks like.
    //
    // Your method must:
    //   - be public
    //   - return nothing (the word for that is void)
    //   - be named Load  (capital L, like the other methods in our robot code)
    //   - take NO parameters
    //
    // Inside it there is just ONE line of code. Call the method you INHERITED from
    // BaseContinuousServo that runs the servo FORWARD for a number of seconds. It is called
    // RunForwardForSecondsAsync. It needs one thing passed to it: how many seconds. Give it
    // your LOAD_SECONDS constant.
    //
    // "Async" is short for "asynchronous", which means "happens on its own, in the
    // background". You do not need to write any code that waits or stops the servo. The
    // inherited method starts the servo, sends a timer off to do the waiting in the
    // background, and returns right away. When the time is up, the timer stops the servo. That
    // is why the robot can keep driving while a ball is loading!
    //
    // One more thing to know: if Load is called while a load is already running, the extra
    // request is ignored. So the servo may seem to ignore a button press that happens during
    // the 2 seconds. That is on purpose, so one ball is loaded at a time.
    //
    // Before you move on, add a comment above the method in your own words saying what it does.
    //
    // (Write your Load method below this line.)


    // =========================================================================================
    // CHECK YOUR WORK
    // =========================================================================================
    //   [ ] The first line of the file says the class "extends BaseContinuousServo".
    //   [ ] You have four constants, each marked private static final, in ALL_CAPS.
    //   [ ] The constructor is spelled exactly like the class and takes one OpMode.
    //   [ ] The very first line inside the constructor is the call to super with 4 things.
    //   [ ] Load has no parameters and its only line passes LOAD_SECONDS to
    //       RunForwardForSecondsAsync.
    //   [ ] Every line of code ends with a semicolon, and every opening { has a closing }.
    //   [ ] The program builds with no red underlines.
    //
    // Common mistakes:
    //   - A red underline on super: it is not the first line in the constructor, or the four
    //     things are in a different order than the parent's constructor expects.
    //   - "Cannot find symbol": a name is misspelled, or has the wrong capital letters. Java
    //     treats  load  and  Load  as two different words.
    //   - The servo does nothing at all: the OpMode did not call Initialize, or the name in
    //     HARDWARE_NAME does not match the Robot Configuration. Those are not in this method.
    //
    // OPTIONAL CHALLENGE (only after everything above works)
    //   Add a second method called Unload that spins the servo BACKWARD for the same number of
    //   seconds, to back a stuck ball out. The inherited method you want is the opposite of the
    //   one you used in Load. Its name follows the same pattern.

}