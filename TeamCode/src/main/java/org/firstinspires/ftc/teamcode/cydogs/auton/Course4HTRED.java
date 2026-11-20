package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Disabled
@Autonomous(name = "Course 4 HT RED", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp RED")
public class Course4HTRED extends Course4HT {
    public Course4HTRED()
    {
        super(TagSight.Alliance.RED);
    }
}