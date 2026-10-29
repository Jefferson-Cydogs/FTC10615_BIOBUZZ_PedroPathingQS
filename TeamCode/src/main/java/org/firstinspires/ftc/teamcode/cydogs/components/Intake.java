package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseContinuousServo;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePowerMotor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

/**
 * Intake - pulls scoring elements into the robot.
 *
 * It has three parts: a main roller motor and two continuous-rotation side servos that turn
 * inward. All names, directions and powers come from BeethovenHardware.Intake.
 *
 * Usage: make an Intake, call Initialize() once, then call RunIntake(), ReverseIntake(),
 * RunRollerOnly() or StopIntake() from button presses.
 */
public class Intake {

    // The three parts of the intake
    private final BasePowerMotor rollerMotor;

    // Creates the three parts using the settings in BeethovenHardware.Intake
    public Intake(OpMode opMode){
        rollerMotor = new BasePowerMotor (opMode,BeethovenHardware.Intake.MOTOR_NAME,BeethovenHardware.Intake.MOTOR_DIRECTION,BeethovenHardware.Intake.MOTOR_ZERO_POWER_BEHAVIOR);
    }

    // Gets all three parts ready. Call once before using the intake.
    public void Initialize(){
        rollerMotor.Initialize();

    }

    // Pulls elements in: both side servos and the roller run inward
    public void RunIntake(){
        rollerMotor.SetPower(BeethovenHardware.Intake.MOTOR_INTAKE_POWER);
    }

    // Spits elements out: side servos stop and the roller runs backwards
    public void ReverseIntake (){
        rollerMotor.SetPower(BeethovenHardware.Intake.MOTOR_REVERSE_POWER);
    }

    // Stops all three parts
    public void StopIntake(){
        rollerMotor.Stop();
    }

    // Runs only the roller inward; the side servos stay stopped
    public void RunRollerOnly(){
        rollerMotor.SetPower(BeethovenHardware.Intake.MOTOR_INTAKE_POWER);
    }
}