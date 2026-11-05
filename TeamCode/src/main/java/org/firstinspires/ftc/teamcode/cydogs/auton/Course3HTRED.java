package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;

@Autonomous(name = "Course 3 HT RED", group = "Autonomous")
public class Course3HTRED extends Course3HT {

    public Course3HTRED()
    {
        super(TagSight.Alliance.RED);
    }
}