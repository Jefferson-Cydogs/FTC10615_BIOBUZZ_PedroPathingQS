package org.firstinspires.ftc.teamcode.cydogs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;

@TeleOp(name = "Beethoven TeleOp RED", group = "TeleOp")
public class BioBuzzTeleopRED extends BioBuzzTeleop {

    public BioBuzzTeleopRED()
    {
        super(TagSight.Alliance.RED);
    }
}