package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;
import com.qualcomm.robotcore.hardware.Servo;
public class FlowerRampServo extends BaseServo {
    public FlowerRampServo (OpMode opMode)
    {
        super(    opMode,
                "FlowerRampServo",
                Servo.Direction.FORWARD,    0,
                1,
                0.96 );
    }

    public void MoveUp()
    {
        SetPosition(0.81);
    } //changed to 81 from 96 after recalibrating

    public void MoveDown() {
        SetPosition(0.32); //changed to 32 from 39 after recalibration
    }




}


