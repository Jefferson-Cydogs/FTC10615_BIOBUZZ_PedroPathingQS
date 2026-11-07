package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 1 HT RED", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp RED")
public class Course1HTRED extends Course1HT {
    public Course1HTRED()
    {
        super(TagSight.Alliance.RED);
    }
}