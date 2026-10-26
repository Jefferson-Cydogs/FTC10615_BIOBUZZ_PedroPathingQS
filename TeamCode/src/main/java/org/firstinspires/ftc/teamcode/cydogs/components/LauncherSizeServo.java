package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

public class LauncherSizeServo extends BaseServo {

    public LauncherSizeServo(OpMode opMode) {
        super(opMode, BeethovenHardware.LauncherSize.NAME, BeethovenHardware.LauncherSize.DIRECTION,
                BeethovenHardware.LauncherSize.MIN_POSITION, BeethovenHardware.LauncherSize.MAX_POSITION,
                BeethovenHardware.LauncherSize.STARTING_POSITION);

    }

    public void SetNectar(){SetPosition(BeethovenHardware.LauncherSize.NECTAR_POSITION);}
    public void SetPollen(){SetPosition(BeethovenHardware.LauncherSize.POLLEN_POSITION);}

}