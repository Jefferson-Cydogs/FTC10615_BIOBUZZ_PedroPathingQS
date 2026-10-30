package org.firstinspires.ftc.teamcode.cydogs.auton;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.Scheduler.schedule;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "Course 1", group = "Autonomous")
public class Course1 extends LinearOpMode {
    private Follower follower;
    private AutoSkills skills;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose Start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose Shoot1 = poseFactory.of(60.25, 12, 270);
    private final Pose GardenApproach = poseFactory.of(28, 8.75, 180);
    private final Pose GardenApproachControl1 = poseFactory.of(35, 20, 0);
    private final Pose GardenCollect = poseFactory.of(12.5, 8.75, 180);
    private final Pose FlowerCollectBadControl1 = poseFactory.of(6.77, 71.73, 0);
    private final Pose FlowerParkControl1 = poseFactory.of(37.1, 92.64, 0);
    private final Pose Shoot2 = poseFactory.of(61.25, 126, 90);
    private final Pose Shoot2Control1 = poseFactory.of(35, 105, 0);
    private final Pose FlowerCollect = poseFactory.of(48, 127, 90);
    private final Pose FlowerCollectControl1 = poseFactory.of(50, 122, 0);
    private final Pose Shoot3 = poseFactory.of(61.25, 126, 90);
    private final Pose Park = poseFactory.of(9.8125, 94.5, 0);

    // Paths
    public Path Shoot1() {
        return Paths.line(Start, Shoot1).linear(Start, Shoot1);
    }
    public Path GardenApproach() {
        return Paths.curve(Shoot1, GardenApproachControl1, GardenApproach).linear(Shoot1, GardenApproach);
    }
    public Path GardenCollect() {
        return Paths.line(GardenApproach, GardenCollect).linear(GardenApproach, GardenCollect);
    }
    public Path GardenReverse() {
        return Paths.line(GardenCollect,Shoot1).linear(GardenCollect, Shoot1);
    }
    public Path FlowerCollectBad() {
        return Paths.curve(Shoot1,FlowerCollectBadControl1,FlowerCollect).linear(Shoot1,FlowerCollect);
    }
    public Path FlowerPark() {
        return Paths.curve(FlowerCollect,FlowerParkControl1,Park).linear(FlowerCollect,Park);
    }
    public Path Shoot2() {
        return Paths.curve(GardenCollect, Shoot2Control1, Shoot2).linear(GardenCollect, Shoot2);
    }
    public Path FlowerCollect() {
        return Paths.curve(Shoot2, FlowerCollectControl1, FlowerCollect).linear(Shoot2, FlowerCollect);
    }
    public Path Shoot3() {
        return Paths.line(FlowerCollect, Shoot3).linear(FlowerCollect, Shoot3);
    }
    public Path Park() {
        return Paths.line(Shoot3, Park).linear(Shoot3, Park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, Shoot1()),
                skills.shootFourPollen(),
                follow(follower, GardenApproach()),
                skills.startIntake(),
                follow(follower, GardenCollect()),
                skills.stopIntake(),
                follow(follower, Shoot2()),
                skills.shootFourPollen(),
                follow(follower, FlowerCollect()),
                waitMs(1500),
                follow(follower, Shoot3()),
                skills.shootFourPollen(),
                follow(follower, Park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(Start);
        follower.update();
        skills = new AutoSkills( this);
        skills.initialize();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            skills.update();
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
