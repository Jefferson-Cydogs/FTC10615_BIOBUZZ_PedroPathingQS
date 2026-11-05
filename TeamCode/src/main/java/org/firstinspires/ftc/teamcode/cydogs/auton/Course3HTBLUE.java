package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;

@Autonomous(name = "Course 1 HT BLUE", group = "Autonomous")
public class Course3HTBLUE extends Course3HT {

    public Course3HTBLUE()
    {
        super(TagSight.Alliance.BLUE);
    }
}