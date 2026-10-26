package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePowerMotor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

public class MoveToMiddleMotor extends BasePowerMotor {

    public MoveToMiddleMotor(OpMode opMode) {

        super(opMode, BeethovenHardware.MoveToMiddle.NAME, BeethovenHardware.MoveToMiddle.DIRECTION,
                BeethovenHardware.MoveToMiddle.ZERO_POWER_BEHAVIOR);
    }
}