package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Course Test", group = "Autonomous")
public class CourseTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Poses
    private final Pose start = poseFactory.of(60, 9.8125, 90);
    private final Pose score1 = poseFactory.of(60, 11, 90);
    private final Pose collectFromGarden = poseFactory.of(9, 12.5625, 90);
    private final Pose score2 = poseFactory.of(60, 128.75, 270);
    private final Pose score2Control1 = poseFactory.of(24, 108, 0);
    private final Pose park = poseFactory.of(9.8125, 96, 0);

    //Paths
    public Path score1() {
        return Paths.line(start, score1).linear(start, score1);
    }
    public Path collectFromGarden() {
        return Paths.line(score1, collectFromGarden).linear(score1, collectFromGarden);
    }
    public Path score2() {
        return Paths.curve(collectFromGarden, score2Control1, score2).linear(collectFromGarden, score2);
    }
    public Path park() {
        return Paths.line(score2, park).linear(score2, park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, score1()),
                waitMs(1000),
                follow(follower, collectFromGarden()),
                waitMs(1500),
                follow(follower, score2()),
                waitMs(1000),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

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
}
