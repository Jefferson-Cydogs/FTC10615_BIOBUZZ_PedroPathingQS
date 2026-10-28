package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
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

import org.firstinspires.ftc.teamcode.cydogs.components.Intake;
import org.firstinspires.ftc.teamcode.cydogs.components.Launcher;
import org.firstinspires.ftc.teamcode.cydogs.components.LauncherSizeServo;
import org.firstinspires.ftc.teamcode.cydogs.components.LoaderServo;
import org.firstinspires.ftc.teamcode.cydogs.components.MoveToLauncherMotor;
import org.firstinspires.ftc.teamcode.cydogs.vision.Aimer;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Course 1", group = "Autonomous")
public class Course1 extends LinearOpMode {
    private Follower follower;
    private AutoSkills skills;


    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose shoot = poseFactory.of(60.25, 14, 270);
    private final Pose gardenApproach = poseFactory.of(33, 10, 270);
    private final Pose gardenCollect = poseFactory.of(10, 10, 270);
    private final Pose shoot2 = poseFactory.of(60.25, 129, 90);
    private final Pose shoot2Control1 = poseFactory.of(26.043, 104.375, 0);
    private final Pose flowerCollect = poseFactory.of(48, 125.6875, 90);
    private final Pose flowerCollectControl1 = poseFactory.of(47.7961, 123.7883, 0);
    private final Pose shoot3 = poseFactory.of(60.25, 129, 90);
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);

    // Paths
    public Path shoot() {
        return Paths.line(start, shoot).linear(start, shoot);
    }
    public Path gardenApproach() {
        return Paths.curve(shoot, gardenApproach).linear(shoot ,gardenApproach);
    }
    public Path gardenCollect() {
        return Paths.curve(gardenApproach, gardenCollect).linear(gardenApproach,gardenCollect);
    }
    public Path shoot2() {
        return Paths.curve(gardenCollect, shoot2Control1, shoot2).linear(gardenCollect, shoot2);
    }
    public Path flowerCollect() {
        return Paths.curve(shoot2, flowerCollectControl1, flowerCollect).linear(shoot2, flowerCollect);
    }
    public Path shoot3() {
        return Paths.line(flowerCollect, shoot3).linear(flowerCollect, shoot3);
    }
    public Path park() {
        return Paths.line(shoot3, park).linear(shoot3, park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                skills.shootFourPollen(),
                follow(follower, gardenApproach()),

                follow(follower,gardenCollect()),
                follow(follower, shoot2()),
                waitMs(1000),
                follow(follower, flowerCollect()),
                waitMs(1500),
                follow(follower, shoot3()),
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
        skills = new AutoSkills( this);
        skills.initialize();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            skills.update();
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
