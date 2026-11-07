package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 1 HT BLUE", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp BLUE")
public class Course1HTBLUE extends Course1HT {
    public Course1HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}