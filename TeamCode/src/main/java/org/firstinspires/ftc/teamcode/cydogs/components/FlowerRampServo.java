package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

import com.qualcomm.robotcore.hardware.Servo;
public class FlowerRampServo extends BaseServo {
    public FlowerRampServo (OpMode opMode)
    {
        super(    opMode,
                "FlowerRampServo",
                Servo.Direction.FORWARD,    0,
                1,
                BeethovenHardware.FlowerRamp.STARTING_POSITION);
    }

    public void MoveUp()
    {
        SetPosition(BeethovenHardware.FlowerRamp.UP_POSITION);
    }

    public void MoveDown() {
        SetPosition(BeethovenHardware.FlowerRamp.DOWN_POSITION);
    }




}


