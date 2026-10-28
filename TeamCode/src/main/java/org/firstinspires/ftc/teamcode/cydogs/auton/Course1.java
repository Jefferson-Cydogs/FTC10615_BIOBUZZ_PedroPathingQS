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
    private LauncherSizeServo launcherSize;
    private Launcher launcher;
    private LoaderServo loader;
   // private Aimer aimer;
    private MoveToLauncherMotor moveToLauncher;
    private Intake intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose shoot = poseFactory.of(60.25, 14, 270);
    private final Pose gardenCollect = poseFactory.of(8, 12, 270);
    private final Pose gardenCollectControl1 = poseFactory.of(27.7305, 22.4141, 0);
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
    public Path gardenCollect() {
        return Paths.curve(shoot, gardenCollectControl1, gardenCollect).linear(shoot ,gardenCollect);
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
                shootFourPollen(),
                follow(follower, gardenCollect()),
                intakeGarden(),
                follow(follower, shoot2()),
                waitMs(1000),
                follow(follower, flowerCollect()),
                waitMs(1500),
                follow(follower, shoot3()),
                waitMs(1000),
                follow(follower, park())
        );
    }

    private static final long SPIN_UP_MS=2000;
    private static final long POP_MS=500;
    private static final long FEED_MS=350;
    private static final long INTAKE_MS=400;
    private static final long INTAKE_MS_GARDEN=2000;
    private static final double FEED_POWER=0.4;

    private Command popOnce(){
      return sequential(
              instant(()->loader.LaunchOnce()),
                    waitMs(POP_MS));
    }

    private Command feedOnce(){
        return sequential(
                instant(()->moveToLauncher.SetPower(FEED_POWER)),
                waitMs(FEED_MS),
                instant(()->moveToLauncher.Stop()));
    }

    private Command intakeOnce(){
        return sequential(
                instant(()->intake.RunIntake()),
                waitMs(INTAKE_MS),
                instant(()->intake.StopIntake()));
    }

    private Command intakeGarden(){
        return sequential(
                instant(()->intake.RunIntake()),
                waitMs(INTAKE_MS_GARDEN),
                instant(()->intake.StopIntake()));
    }
    private Command shootFourPollen(){
        return sequential(
                instant(()->launcher.StartLauncher()),
                waitMs(SPIN_UP_MS),

                popOnce(),
                feedOnce(),
                intakeOnce(),
                waitMs(400),
                popOnce(),
                feedOnce(),
                intakeOnce(),
                waitMs(400),
                popOnce(),
                feedOnce(),
                intakeOnce(),
                waitMs(400),
                popOnce(),

                instant(()->{
                    intake.StopIntake();
                    moveToLauncher.Stop();
                    launcher.StopLauncher();
                })
        );
    }






















    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
// Shooting Hardware
        launcherSize = new LauncherSizeServo(this);
        launcher = new Launcher(this);
        loader = new LoaderServo(this);
         moveToLauncher = new MoveToLauncherMotor(this);
        intake = new Intake(this);
      //  aimer =new Aimer(this, TagSight.Alliance.BLUE);
//Initialize
        launcherSize.Initialize();
        launcher.Initialize();
        loader.Initialize();
        moveToLauncher.Initialize();
        intake.Initialize();
        launcherSize.SetPollen();
       // aimer.preset(0);

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
            launcher.Update();
            loader.Update();

            telemetry.update();
        }
    }
}
