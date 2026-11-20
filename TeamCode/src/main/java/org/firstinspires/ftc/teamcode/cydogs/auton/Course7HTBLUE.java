package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Disabled
@Autonomous(name = "Course 7 HT BLUE", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp BLUE")
public class Course7HTBLUE extends Course7HT {
    public Course7HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}