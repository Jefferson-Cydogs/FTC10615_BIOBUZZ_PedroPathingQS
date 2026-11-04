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
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pause at the flower after collect so the intake can finish pulling balls in (tune on field)
    private static final long FLOWER_COLLECT_WAIT_MS = 500;
    // How long into the second drive in before the feeder stops (ms)
    private static final long FEEDER_STOP_DELAY_MS = 500;

    // How long into the third drive in before the intake stops (ms). TUNE on the field.
    private static final long INTAKE_STOP_DELAY_MS = 1000;

    // How far each in-and-out pass at the flower backs up from FlowerCollect (inches
    private static final double FLOWER_NUDGE_INCHES = 4.0;

    // Alliance for tip check (change to RED if needed)
    private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;

    // Turret: center = 180. 30° left of center (use 180.0 - 30.0 if left is the other way)
    private static final double TIP_LOOK_DEGREES = 180.0 + 30.0;

    // Poses (all shared poses are copied from Course 1, which is field-tuned)
    private final Pose Start = poseFactory.of(35.5, 9.8125, 270);
    private final Pose Shoot1 = poseFactory.of(28.5, 15, 235);
    private final Pose GardenApproach = poseFactory.of(28, 8.75, 180);
    private final Pose GardenApproachControl1 = poseFactory.of(35, 20, 0);
    private final Pose GardenCollect = poseFactory.of(12.5, 8.75, 180);
    private final Pose Shoot2 = poseFactory.of(28.5, 126.5, 125);
    private final Pose Shoot2Control1 = poseFactory.of(28.5, 72, 0);
    private final Pose FlowerApproach = poseFactory.of(49, 115, 90);
    private final Pose FlowerCollect = poseFactory.of(49, 124.5, 90);
    private final Pose FlowerCollectShake = poseFactory.of(49.5, 124.5, 85);
    private final Pose Shoot3 = poseFactory.of(28.5, 126.5, 125);
    // Tail B only: bend point for the long curve up the left side from Shoot1 to FlowerApproach
    private final Pose FlowerApproachBadControl1 = poseFactory.of(6, 72, 0);
    // FLOWER_NUDGE_INCHES back from FlowerCollect, along the line toward FlowerApproach
    private final Pose FlowerNudgeBack = flowerNudgeBack();
    private Pose flowerNudgeBack() {
        double dx = FlowerApproach.x() - FlowerCollect.x();
        double dy = FlowerApproach.y() - FlowerCollect.y();
        double len = Math.hypot(dx, dy);
        return poseFactory.of(
                FlowerCollect.x() + dx / len * FLOWER_NUDGE_INCHES,
                FlowerCollect.y() + (dy - 1) / len * FLOWER_NUDGE_INCHES,
                90);
    }
    private final Pose Park = poseFactory.of(9.8125, 94.5, 0);

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
                .with(Constants.foresightConfig.maxPathSpeed.at(0.3));
    }
    public Path Shoot2() {
        return Paths.curve(GardenCollect, Shoot2Control1, Shoot2).linear(GardenCollect, Shoot2);
    }
    public Path FlowerApproach() {
        return Paths.line(Shoot2, FlowerApproach).linear(Shoot2, FlowerApproach);
    }
    public Path FlowerCollect() {
        return Paths.line(FlowerApproach, FlowerCollect)
                .linear(FlowerApproach, FlowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }
    public Path FlowerCollectShake() {
        return Paths.line(FlowerCollect, FlowerCollectShake).linear(FlowerCollect, FlowerCollectShake);
    }
    // Reverse of FlowerCollect. Ramp must stay down until this finishes.
    public Path FlowerBackUp() {
        return Paths.line(FlowerCollect, FlowerApproach)
                .linear(FlowerCollect, FlowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }
    // Starts at FlowerApproach (after back-up), not FlowerCollect
    public Path Shoot3() {
        return Paths.line(FlowerApproach, Shoot3).linear(FlowerApproach, Shoot3);
    }
    // Tail B paths
    public Path GardenReverse() {
        return Paths.line(GardenCollect, Shoot1).linear(GardenCollect, Shoot1);
    }
    // Shoot1 → FlowerApproach (Tail B only)
    public Path FlowerApproachBad() {
        return Paths.curve(Shoot1, FlowerApproachBadControl1, FlowerApproach).linear(Shoot1, FlowerApproach);
    }
    // Short in-and-out passes at the flower
    public Path FlowerNudgeBack() {
        return Paths.line(FlowerCollect, FlowerNudgeBack)
                .linear(FlowerCollect, FlowerNudgeBack)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }
    public Path FlowerNudgeIn() {
        return Paths.line(FlowerNudgeBack, FlowerCollect)
                .linear(FlowerNudgeBack, FlowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.3));
    }
    public Path Park() {
        return Paths.line(Shoot3, Park).linear(Shoot3, Park);
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

    // Two passes into the flower with the ramp down, intake + feeder running:
    //   pass 1 + shake releases the balls, pass 2 gets the last one.
    private Command doFlower() {
        return sequential(
                skills.flowerRampDown(),
                skills.startIntakeAndFeed(),
                follow(follower, FlowerCollect()),        // pass 1
                waitMs(FLOWER_COLLECT_WAIT_MS),
                follow(follower, FlowerCollectShake()),   // shake ramp
                follow(follower, FlowerCollect()),
                follow(follower, FlowerNudgeBack()),
                skills.stopFeedAfter(FEEDER_STOP_DELAY_MS),
                skills.stopIntakeAfter(INTAKE_STOP_DELAY_MS),
                follow(follower, FlowerNudgeIn()),        // pass 2
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
                skills.emptyMagazineWhileDriving(),

                follow(follower, GardenApproach()),
                skills.startIntakeAndFeed(),
                skills.stopFeedAfter(FEEDER_STOP_DELAY_MS),
                //skills.startIntake(),
                follow(follower, GardenCollect()),
                //skills.stopIntakeAndFeed(),
                //skills.stopIntake(),

                // Lock = basket still up → no tip. No lock → tipped.
                skills.checkForTip(ALLIANCE, TIP_LOOK_DEGREES),
                skills.stopIntakeAndFeed()
        );
    }
    // ---------- TAIL A: tipped → finish normal route (same route as Course 1) ----------
    private Command tailA_HiveTipped() {
        return sequential(
                follow(follower, Shoot2()),
                skills.turretStraight(),
                skills.shootFourPollen(),
                skills.emptyMagazineWhileDriving(),

                follow(follower, FlowerApproach()),
                doFlower(),

                follow(follower, Shoot3()),
                skills.turretStraight(),
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
                follow(follower, GardenReverse()),   // Garden → Shoot1
                skills.turretStraight(),
                skills.shootFourPollen(),
                skills.emptyMagazineWhileDriving(),

                follow(follower, FlowerApproachBad()), // Shoot1 → FlowerApproach
                doFlower(),

                follow(follower, Shoot3()),
                skills.turretStraight(),
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
