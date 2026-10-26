package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.ivy.Scheduler.schedule;
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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Course2", group = "Autonomous")
public class Course2 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9.8125, 90);
    private final Pose shoot = poseFactory.of(56, 14, 90);
    private final Pose colectfromGarden = poseFactory.of(9, 12.8125, 90);
    private final Pose colectfromGardenControl1 = poseFactory.of(25.6113, 21.2454, 0);
    private final Pose shoot2 = poseFactory.of(56, 14, 90);
    private final Pose flower = poseFactory.of(14.8125, 48, 0);
    private final Pose shoot3 = poseFactory.of(56, 14, 90);
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                follow(follower, colectfromGarden()),
                follow(follower, shoot2()),
                follow(follower, flower()),
                follow(follower, shoot3()),
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

    public Path shoot() {
        return Paths.line(start, shoot).linear(start, shoot);
    }

    public Path colectfromGarden() {
        return Paths.curve(shoot, colectfromGardenControl1, colectfromGarden).linear(shoot, colectfromGarden);
    }

    public Path shoot2() {
        return Paths.line(colectfromGarden, shoot2).linear(colectfromGarden, shoot2);
    }

    public Path flower() {
        return Paths.line(shoot2, flower).linear(shoot2, flower);
    }

    public Path shoot3() {
        return Paths.line(flower, shoot3).linear(flower, shoot3);
    }

    public Path park() {
        return Paths.line(shoot3, park).linear(shoot3, park);
    }
}
