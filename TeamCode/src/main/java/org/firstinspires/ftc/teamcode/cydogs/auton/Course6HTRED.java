package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Disabled
@Autonomous(name = "Course 6 HT RED", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp RED")
public class Course6HTRED extends Course6HT {
    public Course6HTRED()
    {
        super(TagSight.Alliance.RED);
    }
}