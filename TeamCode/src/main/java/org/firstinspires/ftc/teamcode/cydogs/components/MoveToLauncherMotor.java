package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BasePowerMotor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

public class MoveToLauncherMotor extends BasePowerMotor {

    public MoveToLauncherMotor(OpMode opMode) {

        super(opMode, BeethovenHardware.MoveToLauncher.NAME, BeethovenHardware.MoveToLauncher.DIRECTION,
                BeethovenHardware.MoveToLauncher.ZERO_POWER_BEHAVIOR);
    }
}