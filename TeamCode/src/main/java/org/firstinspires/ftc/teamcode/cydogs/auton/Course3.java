package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.cydogs.auton.AutoSkills;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Course3", group = "Autonomous")
public class Course3 extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 132.6875, 270);
    private final Pose shoot1 = poseFactory.of(56, 127.5, 270);
    private final Pose flower = poseFactory.of(48, 127.6875, -90);
    private final Pose flowerControl1 = poseFactory.of(47.8983, 112.5803, 0);
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                waitMs(1000),
                follow(follower, flower()),
                waitMs(1500),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        skills = new AutoSkills( this);
        skills.initialize();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            skills.update();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path flower() {
        return Paths.curve(shoot1, flowerControl1, flower).linear(shoot1, flower);
    }

    public Path park() {
        return Paths.line(flower, park).linear(flower, park);
    }
}
