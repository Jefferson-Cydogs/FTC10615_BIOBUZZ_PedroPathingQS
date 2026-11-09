package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 5 HT BLUE", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp BLUE")
public class Course5HTBLUE extends Course5HT {
    public Course5HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}