package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Course 8 HT RED", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp RED")
public class Course8HTRED extends Course8HT {
    public Course8HTRED()
    {
        super(TagSight.Alliance.RED);
    }
}