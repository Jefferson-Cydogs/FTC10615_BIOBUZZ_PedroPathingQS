package org.firstinspires.ftc.teamcode.cydogs.auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

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
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Course 1 with Heads / Tails after the garden.
 *
 * HEAD (always):
 *   Shoot1 → hold → shoot → garden approach → intake on → garden collect → intake off
 *   → checkForTip (look 20° left of center)
 *
 * TAIL A — hiveTipped == true  (no lock → treat as tipped): rest of normal route
 * TAIL B — hiveTipped == false (still see basket): contingency (empty for now)
 */
@Autonomous(name = "Course 1 HT", group = "Autonomous")
public class Course1HT extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Alliance for tip check (change to RED if needed)
    private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;

    // Turret: center = 180. 20° left of center (use 200 if left is the other way)
    private static final double TIP_LOOK_DEGREES = 180.0 + 30.0;

    // Poses
    private final Pose Start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose Shoot1 = poseFactory.of(60.25, 12, 270);
    private final Pose GardenApproach = poseFactory.of(27, 9, 180);
    private final Pose GardenApproachControl1 = poseFactory.of(30, 20, 0);
    private final Pose GardenCollect = poseFactory.of(12, 9, 180);
    private final Pose FlowerCollectBad = poseFactory.of(48, 127.5, 90);
    private final Pose FlowerCollectBadControl1 = poseFactory.of(6.77, 71.73, 0);
    private final Pose FlowerParkControl1 = poseFactory.of(37.1, 92.64, 0);
    private final Pose Shoot2 = poseFactory.of(60.25, 127, 90);
    private final Pose Shoot2Control1 = poseFactory.of(25, 110, 0);
    private final Pose FlowerCollect = poseFactory.of(48, 127.5, 90);
    private final Pose FlowerCollectControl1 = poseFactory.of(50, 122, 0);
    private final Pose Shoot3 = poseFactory.of(60.25, 127, 90);
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
        return Paths.curve(Shoot1,FlowerCollectBadControl1,FlowerCollectBad).linear(Shoot1,FlowerCollectBad);
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

    // ---------- HEAD (always runs) ----------
    private Command head() {
        return sequential(
                follow(follower, Shoot1()),
                hold(follower),                 // hold pose while spinning up / shooting
                skills.shootFourPollen(),

                follow(follower, GardenApproach()),
                skills.startIntake(),
                follow(follower, GardenCollect()),
                skills.stopIntake(),

                // Lock = basket still up → no tip. No lock → tipped.
                skills.checkForTip(ALLIANCE, TIP_LOOK_DEGREES)
        );
    }

    // ---------- TAIL A: tipped → finish normal route ----------
    private Command tailA_HiveTipped() {
        return sequential(
                follow(follower, Shoot2()),
                hold(follower),
                skills.shootFourPollen(),

                follow(follower, FlowerCollect()),
                waitMs(1500),

                follow(follower, Shoot3()),
                hold(follower),
                skills.shootFourPollen(),

                follow(follower, Park())
        );
    }

    // ---------- TAIL B: still see basket → contingency (empty for now) ----------
// ---------- TAIL B: still see basket (no tip) → reverse, reshoot, then park ----------
    private Command tailB_NoTip() {
        return sequential(
                follow(follower, GardenReverse()),   // garden → Shoot1
                hold(follower),
                skills.shootFourPollen(),

                follow(follower, FlowerCollectBad()), // Shoot1 → flower
                // optional: skills.startIntake() / stop around a collect path if you need it

                follow(follower, FlowerPark())        // flower → park
        );
    }

    /**
     * After tip check, run exactly one tail.
     * hiveTipped() true  → Tail A
     * hiveTipped() false → Tail B
     */
    private Command chooseTail() {
        return Command.build()
                .setStart(() -> {
                    if (skills.hiveTipped()) {
                        schedule(tailA_HiveTipped());
                    } else {
                        schedule(tailB_NoTip());
                    }
                })
                .setDone(() -> true);
    }

    public Command autoRoutine() {
        return sequential(
                head(),
                chooseTail()
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(Start);
        follower.update();

        skills = new AutoSkills(this);
        skills.initialize();
        skills.prepareVision(TagSight.Alliance.BLUE);  // or RED

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            skills.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("hiveTipped", skills.hiveTipped());
            telemetry.addData("follower mode", follower.mode());

            if (follower.currentPath() != null) {
                telemetry.addData("path remaining", follower.distanceToEndpoint());
                telemetry.addData("path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}