package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 6 HT BLUE", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp BLUE")
public class Course6HTBLUE extends Course6HT {
    public Course6HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}