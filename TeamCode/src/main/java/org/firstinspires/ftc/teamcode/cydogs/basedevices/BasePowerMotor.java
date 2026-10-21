package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BasePowerMotor {
    private final OpMode opMode;
    private final String hardwareName;
    private final DcMotorSimple.Direction direction;
    private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;

    private DcMotor motor;

    public BasePowerMotor(OpMode opMode,
                          String hardwareName,
                          DcMotorSimple.Direction direction,
                          DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.opMode = opMode;
        this.hardwareName = hardwareName;
        this.direction = direction;
        this.zeroPowerBehavior = zeroPowerBehavior;
    }
    /** Gets the motor from the hardware map and applies direction and zero power behavior.*/
    public void Initialize() {
        motor = opMode.hardwareMap.get(DcMotor.class, hardwareName);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        motor.setPower(0);
    }

    /** Sets raw power, limited to [-1, 1]. */
    public void SetPower(double power) {
        checkInitialized();
        motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void Stop() {
        checkInitialized();
        motor.setPower(0);
    }
    public double GetPower() {
        checkInitialized();
        return motor.getPower();
    }
    private void checkInitialized() {
        if (motor == null) {
            throw new IllegalStateException(
                    "BasePowerMotor '" + hardwareName + "' used before Initialized () was called");
        }
    }

}