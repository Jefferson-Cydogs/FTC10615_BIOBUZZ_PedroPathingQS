package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePowerMotor;

public class MoveToMiddleMotor extends BasePowerMotor {
    /**
     * runPower is the speed used by RunForward/RunBackward, from 0 to 1 (sign is ignored).
     *
     * @param opMode
     * @param hardwareName
     * @param direction
     * @param runPower
     */

    private static final String hardwareName = "MoveToMiddleMotor";

    private static final DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    private static double runPower = 0.5;


    public MoveToMiddleMotor(OpMode opMode) {

        super(opMode, hardwareName, direction, DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
