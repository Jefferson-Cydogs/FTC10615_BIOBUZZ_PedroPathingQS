package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;


@Autonomous(name = "Hive Down Short BLUE", group = "Autonomous", preselectTeleOp = "Beethoven TeleOp BLUE")
public class Course8HTBLUE extends Course8HT {
    public Course8HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}