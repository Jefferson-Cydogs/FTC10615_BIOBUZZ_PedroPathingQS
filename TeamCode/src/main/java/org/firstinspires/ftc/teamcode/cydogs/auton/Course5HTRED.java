package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 5 HT RED", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp RED")
public class Course5HTRED extends Course5HT {
    public Course5HTRED()
    {
        super(TagSight.Alliance.RED);
    }
}