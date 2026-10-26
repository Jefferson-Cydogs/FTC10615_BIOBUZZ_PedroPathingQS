package org.firstinspires.ftc.teamcode.cydogs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;

@TeleOp(name = "Beethoven TeleOp BLUE", group = "TeleOp")
public class BioBuzzTeleopBLUE extends BioBuzzTeleop {

    public BioBuzzTeleopBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}