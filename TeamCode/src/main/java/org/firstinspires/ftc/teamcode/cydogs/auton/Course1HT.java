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

import org.firstinspires.ftc.teamcode.cydogs.auton.AutoSkills;
import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Course 1 with Heads / Tails after the garden.
 *
 * HEAD (always):
 *   Shoot1 → shoot → garden approach → intake on → garden collect → intake off
 *   → checkForTip (look 30° left of center)
 *
 * TAIL A — hiveTipped == true  (no lock → treat as tipped): rest of normal route
 *   (same poses and paths as Course 1)
 * TAIL B — hiveTipped == false (still see basket): contingency
 *   reverse to Shoot1, reshoot, far flower, far shoot, park
 *
 * Flower sequence (both tails):
 *   approach → ramp down → intake on → collect → intake off → back up → ramp up → shoot
 *   (we back up BEFORE raising the ramp, otherwise the ramp comes up under the flower)
 */
@Autonomous(name = "Course 1 HT", group = "Autonomous")
public class Course1HT extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;
    // Pause at the flower after collect so the intake can finish pulling balls in (tune on field)
    private static final long FLOWER_COLLECT_WAIT_MS = 500;
    // How long into the second drive in before the feeder stops (ms)
    private static final long FEEDER_STOP_DELAY_MS = 500;

    // How long into the third drive in before the intake stops (ms). TUNE on the field.
    private static final long INTAKE_STOP_DELAY_MS = 1000;

    // How far each in-and-out pass at the flower backs up from FlowerCollect (inches
    private static final double FLOWER_NUDGE_INCHES = 4.0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Alliance for tip check (change to RED if needed)
    private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;

    // Turret: center = 180. 30° left of center (use 180.0 - 30.0 if left is the other way)
    private static final double TIP_LOOK_DEGREES = 180.0 + 30.0;

    // Poses (all shared poses are copied from Course 1, which is field-tuned)
    private final Pose Start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose Shoot1 = poseFactory.of(60.25, 12, 270);
    private final Pose GardenApproach = poseFactory.of(28, 8.75, 180);
    private final Pose GardenApproachControl1 = poseFactory.of(35, 20, 0);
    private final Pose GardenCollect = poseFactory.of(12.5, 8.75, 180);
    private final Pose Shoot2Half1 = poseFactory.of(36, 108, 90);
    private final Pose Shoot2Half2 = poseFactory.of(61.25, 126, 90);
    private final Pose Shoot2Half2Control1 = poseFactory.of(60, 108, 0);
    private final Pose FlowerApproach = poseFactory.of(48, 115, 90);
    private final Pose FlowerApproachControl1 = poseFactory.of(56, 110, 0);
    private final Pose FlowerCollect = poseFactory.of(46, 125.5, 90);
    private final Pose Shoot3 = poseFactory.of(61.25, 126, 90);
    private final Pose Park = poseFactory.of(9.8125, 94.5, 0);
    // Tail B only: bend point for the long curve up the left side from Shoot1 to FlowerApproach
    private final Pose FlowerApproachBadControl1 = poseFactory.of(6.77, 71.73, 0);
    // FLOWER_NUDGE_INCHES back from FlowerCollect, along the line toward FlowerApproach
    private final Pose FlowerNudgeBack = flowerNudgeBack();
    private Pose flowerNudgeBack() {
        double dx = FlowerApproach.x() - FlowerCollect.x();
        double dy = FlowerApproach.y() - FlowerCollect.y();
        double len = Math.hypot(dx, dy);
        return poseFactory.of(
                FlowerCollect.x() + dx / len * FLOWER_NUDGE_INCHES,
                FlowerCollect.y() + dy / len * FLOWER_NUDGE_INCHES,
                90);
    }



    // Paths
    public Path Shoot1() {
        return Paths.line(Start, Shoot1).linear(Start, Shoot1);
    }

    public Path GardenApproach() {
        return Paths.curve(Shoot1, GardenApproachControl1, GardenApproach).linear(Shoot1, GardenApproach);
    }

    public Path GardenCollect() {
        return Paths.line(GardenApproach, GardenCollect)
                .linear(GardenApproach, GardenCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    public Path Shoot2Half1() {
        return Paths.line(GardenCollect, Shoot2Half1).linear(GardenCollect, Shoot2Half1);
    }

    public Path Shoot2Half2() {
        return Paths.curve(Shoot2Half1, Shoot2Half2Control1, Shoot2Half2).linear(Shoot2Half1, Shoot2Half2);
    }

    // Full speed: the 40% cap lives on FlowerCollect, the leg that actually touches the flower
    public Path FlowerApproach() {
        return Paths.curve(Shoot2Half2, FlowerApproachControl1, FlowerApproach)
                .linear(Shoot2Half2, FlowerApproach);
    }

    public Path FlowerCollect() {
        return Paths.line(FlowerApproach, FlowerCollect)
                .linear(FlowerApproach, FlowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.3));
    }

    // Reverse of FlowerCollect. Ramp must stay down until this finishes.
    public Path FlowerBackUp() {
        return Paths.line(FlowerCollect, FlowerApproach)
                .linear(FlowerCollect, FlowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Starts at FlowerApproach (after back-up), not FlowerCollect
    public Path Shoot3() {
        return Paths.line(FlowerApproach, Shoot3).linear(FlowerApproach, Shoot3);
    }

    public Path Park() {
        return Paths.line(Shoot3, Park).linear(Shoot3, Park);
    }

    // Tail B paths
    public Path GardenReverse() {
        return Paths.line(GardenCollect, Shoot1).linear(GardenCollect, Shoot1);
    }

    // Shoot1 → FlowerApproach (Tail B only)
    public Path FlowerApproachBad() {
        return Paths.curve(Shoot1, FlowerApproachBadControl1, FlowerApproach)
                .linear(Shoot1, FlowerApproach);
    }
    // Short in-and-out passes at the flower
    public Path FlowerNudgeBack() {
        return Paths.line(FlowerCollect, FlowerNudgeBack)
                .linear(FlowerCollect, FlowerNudgeBack)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.3));
    }
    public Path FlowerNudgeIn() {
        return Paths.line(FlowerNudgeBack, FlowerCollect)
                .linear(FlowerNudgeBack, FlowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.3));
    }

    // ---------- shared flower pickup (both tails) ----------
    /*private Command doFlower() {
        return sequential(
                skills.flowerRampDown(),
                skills.startIntake(),
                follow(follower, FlowerCollect()),
                waitMs(FLOWER_COLLECT_WAIT_MS),
                skills.stopIntake(),
                follow(follower, FlowerBackUp()),
                skills.flowerRampUp()
        );
    }*/

    // Three passes into the flower with the ramp down, intake + feeder running:
    //   pass 1 releases the balls, pass 2 runs over 3 (maybe 4), pass 3 gets the last one.
    private Command doFlower() {
        return sequential(
                skills.flowerRampDown(),
                skills.startIntakeAndFeed(),
                follow(follower, FlowerCollect()),        // pass 1
                waitMs(FLOWER_COLLECT_WAIT_MS),
                follow(follower, FlowerNudgeBack()),
                skills.stopFeedAfter(FEEDER_STOP_DELAY_MS),
                follow(follower, FlowerNudgeIn()),        // pass 2 (feeder stops ~0.5 s in)
                follow(follower, FlowerNudgeBack()),
                skills.stopIntakeAfter(INTAKE_STOP_DELAY_MS),
                follow(follower, FlowerNudgeIn()),        // pass 3
                skills.stopIntakeAndFeed(),
                follow(follower, FlowerBackUp()),
                skills.flowerRampUp()
        );
    }

    // ---------- HEAD (always runs) ----------
    private Command head() {
        return sequential(
                skills.startLauncher(),
                follow(follower, Shoot1()),
                skills.shootFourPollen(),

                follow(follower, GardenApproach()),
                skills.startIntakeAndFeed(),
                skills.stopFeedAfter(FEEDER_STOP_DELAY_MS),
                //skills.startIntake(),
                follow(follower, GardenCollect()),
                skills.stopIntakeAndFeed(),
                //skills.stopIntake(),

                // Lock = basket still up → no tip. No lock → tipped.
                skills.checkForTip(ALLIANCE, TIP_LOOK_DEGREES)
        );
    }

    // ---------- TAIL A: tipped → finish normal route (same route as Course 1) ----------
    private Command tailA_HiveTipped() {
        return sequential(
                follow(follower, Shoot2Half1()),
                follow(follower, Shoot2Half2()),
                skills.turretStraight(),
                skills.shootFourPollen(),

                follow(follower, FlowerApproach()),
                doFlower(),

                follow(follower, Shoot3()),
                skills.shootFourPollen(),

                follow(follower, Park())
        );
    }

    // ---------- TAIL B: still see basket (no tip) ----------
    // Why this route differs from Tail A: we are not coming from the garden's far end,
    // we reverse back to the near shoot pose (Shoot1), reshoot the garden balls there,
    // and only then head across. From Shoot1 we run one long curve up the left side
    // straight to FlowerApproach (south of the flower), so we never skip past it, and
    // there is no far-side shot BEFORE the flower (the balls were already shot at Shoot1).
    // After FlowerApproach it shares the flower pickup, Shoot3, and Park with Tail A.
    private Command tailB_NoTip() {
        return sequential(
                follow(follower, GardenReverse()),   // garden → Shoot1
                skills.turretStraight(),
                skills.shootFourPollen(),

                follow(follower, FlowerApproachBad()), // Shoot1 → FlowerApproach
                doFlower(),

                follow(follower, Shoot3()),
                skills.shootFourPollen(),              // half a tip on their side

                follow(follower, Park())
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
        skills.prepareVision(ALLIANCE);

        // INIT loop: confirm the camera is streaming and sees the hive tag before the match
        while (!isStarted() && !isStopRequested()) {
            skills.updateInitVision();
            telemetry.update();
        }

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