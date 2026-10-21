package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePositionalMotor;

/**
 * FlowerSlider - the motor that lifts the U-channel up to flower height so we can dump scoring
 * elements into the flower, and lowers it back to its home position.
 *
 * A FlowerSlider IS a BasePositionalMotor, so it inherits Initialize, SetToMaximum,
 * SetToMinimum, IsBusy and the rest. This class only adds the settings for this motor and the
 * two methods RaiseToFlower and LowerToHome.
 *
 * How an OpMode uses it:
 *   1. Make one:            new FlowerSlider(this)
 *   2. Call Initialize()    once (inherited). This sets "zero ticks" to wherever the slider is
 *                           sitting at that moment, so the slider MUST be fully down when
 *                           Init is pressed.
 *   3. Call RaiseToFlower() and LowerToHome() from button presses.
 *
 * Both methods start the move and return right away. The motor controller drives to the target
 * and holds it, so the robot can keep driving while the slider moves.
 */
public class FlowerSlider extends BasePositionalMotor {

    // Must match the motor's name in the Robot Configuration on the Driver Hub.
    private static final String HARDWARE_NAME = "FlowerSlider";

    // Chosen so the slider goes UP when the tick count goes UP. Flip it if it goes the wrong way.
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    // BRAKE makes the motor hold its place so the slider doesn't sink under the load.
    private static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // Lowest position (zero ticks is wherever the slider sits when Initialize runs).
    private static final int HOME_TICKS = 0;

    // Ticks from home up to flower height, measured on the robot.
    private static final int FLOWER_TICKS = 300;

    // Going up fights gravity, so it gets more power. Going down gets gravity's help.
    private static final double RAISE_POWER = 0.7;
    private static final double LOWER_POWER = 0.4;

    /** HOME_TICKS and FLOWER_TICKS become the parent's min and max, so the slider can never leave that range. */
    public FlowerSlider(OpMode opMode) {
        super(opMode, HARDWARE_NAME, DIRECTION, ZERO_POWER_BEHAVIOR, HOME_TICKS, FLOWER_TICKS);
    }

    /** Drives the slider up to flower height. Returns immediately. */
    public void RaiseToFlower() {
        SetToMaximum(RAISE_POWER);
    }

    /** Drives the slider back down to its home position. Returns immediately. */
    public void LowerToHome() {
        SetToMinimum(LOWER_POWER);
    }
}