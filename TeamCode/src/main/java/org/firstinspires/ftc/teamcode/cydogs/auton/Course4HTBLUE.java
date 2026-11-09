package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 4 HT BLUE", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp BLUE")
public class Course4HTBLUE extends Course4HT {
    public Course4HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}