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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cydogs.vision.TagSight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Course3HT — Second Shot Triple Tip + bail-out tail.
 *
 * We start on the FAR side (intake against the wall, turret/back toward the hive),
 * with 4 pollen preloaded. Our partner shoots first.
 *
 * Seesaw rule: we only see the tag when OUR side is up.
 *   At the start our side is down, so we see no tag.
 *   When the partner tips, our side comes up and we see the tag (lock).
 *
 * HEAD: start → far shoot pose → wait for lock (timeout PARTNER_TIMEOUT_MS)
 *
 * Tail A (tailA_partnerTipped) — lock before timeout:
 *   shoot 4 (preload) → far flower (approach, ramp, intake, collect, back up)
 *   → shoot 4 (second tip) → park
 *   Same poses, paths and speed caps as Course 1's far side.
 *
 * Tail B (tailB_partnerMissed) — timeout, no lock:
 *   cross to the near side by Course 1's tuned left-side route in reverse
 *   (far shoot → Shoot2Half1 → garden approach → near shoot pose),
 *   shoot the preloaded 4 (enough to tip the first hive, it already holds 3 nectar),
 *   then park.
 */
//@Autonomous(name = "Course3HT", group = "Autonomous")
//public class Course3HT extends LinearOpMode {
public abstract class Course3HT extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Alliance for the tag (change to RED if needed)
    //private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;
    //private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.RED;
    private final TagSight.Alliance alliance;
    protected Course3HT(TagSight.Alliance alliance) {
        this.alliance = alliance;
    }

    private static final long FLOWER_COLLECT_WAIT_MS = 500;
    // How long into the second drive in before the feeder stops (ms)
    private static final long FEEDER_STOP_DELAY_MS = 500;
    // How long into the third drive in before the intake stops (ms). TUNE on the field.
    private static final long INTAKE_STOP_DELAY_MS = 1000;
    // How far each in-and-out pass at the flower backs up from FlowerCollect (inches
    private static final double FLOWER_NUDGE_INCHES = 4.0;
    // Turret: 180 = straight ahead, which is right at the hive from the far shoot pose
    private static final double TIP_LOOK_DEGREES = 180.0;
    // How long we wait for the partner's tip before going to help (tune on field)
    private static final long PARTNER_TIMEOUT_MS = 10000;
    // Pause at the flower after collect so the intake can finish (same as other courses)


    // -------------------------------------------------------------------------
    // --- Poses (shared poses copied from Course 1, which is field-tuned) ---
    // -------------------------------------------------------------------------
    private final Pose start = poseFactory.of(35.5, 130.5, 90);
    private final Pose shoot1 = poseFactory.of(28.5, 126.5, 127);
    // Tail A: far flower (Course 1)
    private final Pose FlowerApproach = poseFactory.of(49, 115, 90);
    private final Pose FlowerCollect = poseFactory.of(49, 124.5, 90);
    private final Pose FlowerCollectShake = poseFactory.of(49.5, 124.5, 85);
    private final Pose FlowerNudgeBack = flowerNudgeBack();
    private Pose flowerNudgeBack() {
        double dx = FlowerApproach.x() - FlowerCollect.x();
        double dy = FlowerApproach.y() - FlowerCollect.y();
        double len = Math.hypot(dx, dy);
        return poseFactory.of(
                FlowerCollect.x() + dx / len * FLOWER_NUDGE_INCHES,
                FlowerCollect.y() + (dy -1) / len * FLOWER_NUDGE_INCHES,
                90
        );
    }
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);
    // Tail B: Course 1 route in reverse
    private final Pose shoot2 = poseFactory.of(28.5, 15, 239);


    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------
    /** Start → far shoot pose (short, straight out from the wall). */
    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }
    // Tail A (same as Course 1's far side)
    public Path FlowerApproach() {
        return Paths.line(shoot1, FlowerApproach).linear(shoot1, FlowerApproach);
    }
    public Path FlowerCollect() {
        return Paths.line(FlowerApproach, FlowerCollect)
                .linear(FlowerApproach, FlowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.3));
    }
    public Path FlowerCollectShake() {
        return Paths.line(FlowerCollect, FlowerCollectShake).linear(FlowerCollect, FlowerCollectShake);
    }
    // Reverse of FlowerCollect. Ramp must stay down until this finishes.
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
    public Path FlowerBackUp() {
        return Paths.line(FlowerCollect, FlowerApproach)
                .linear(FlowerCollect, FlowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }
    public Path PostFlowerShoot() {
        return Paths.line(FlowerApproach, shoot1).linear(FlowerApproach, shoot1);
    }
    // Tail B (Course 1's route to the far side, driven in reverse)
    /** Far shoot pose → half-way point (reverse of Course 1's Shoot2Half2). */
    public Path shoot2() {
        return Paths.line(shoot1, shoot2).linear(shoot1, shoot2);
    }
    public Path parkTailA() {
        return Paths.line(shoot1, park).linear(shoot1, park);
    }
    public Path parkTailB() {
        return Paths.line(shoot2, park).linear(shoot2, park);
    }


    // -------------------------------------------------------------------------
    // Command pieces
    // -------------------------------------------------------------------------
    /**
     * Same flower sequence as Course 1 HT:
     * ramp down → intake on → collect → wait → intake off → back up → ramp up.
     * (Back up BEFORE raising the ramp, otherwise it comes up under the flower.)
     */
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
    /**
     * Head: drive out to the far shoot pose, then wait for the tag.
     * Lock before timeout   → hiveTipped false → partner tipped → Tail A
     * Timeout (no lock)     → hiveTipped true  → partner missed  → Tail B
     */
    private Command head() {
        return sequential(
                follow(follower, shoot1()),
                skills.waitForPartnerTipOrTimeout(alliance, TIP_LOOK_DEGREES, PARTNER_TIMEOUT_MS)
        );
    }
    /** Tail A — partner tipped: shoot, far flower, shoot again (our tip), park. */
    private Command tailA_partnerTipped() {
        return sequential(
                skills.turretStraight(),
                skills.startLauncher(),
                waitMs(3000),
                skills.shootFourPollen(),
                skills.emptyMagazineWhileDriving(),
                follow(follower, FlowerApproach()),
                doFlower(),
                follow(follower, PostFlowerShoot()),
                skills.turretStraight(),
                skills.shootFourPollen(),
                follow(follower, parkTailA())
        );
    }
    /**
     * Tail B — partner never tipped: go to their side and shoot our preloaded 4
     * (enough to tip the first hive), then park.
     */
    private Command tailB_partnerMissed() {
        return sequential(
                skills.startLauncher(),
                follow(follower, shoot2()),
                skills.turretStraight(),
                skills.shootFourPollen(),
                follow(follower, parkTailB())
        );
    }


    // -------------------------------------------------------------------------
    // Main routine
    // -------------------------------------------------------------------------
    public Command autoRoutine() {
        return sequential(
                head(),
                // branchOnTip(tipped slot, locked slot): timeout = "tipped" slot
                skills.branchOnTip(
                        tailB_partnerMissed(),   // timeout, no lock → bail out
                        tailA_partnerTipped()    // lock → happy path
                )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        skills = new AutoSkills(this);
        skills.initialize();
        skills.prepareVision(alliance);

        // INIT loop: confirm the camera is streaming (check Camera = STREAMING and fps).
        // NOTE: in this course our side starts DOWN, so "NO TAG IN VIEW" is NORMAL at init.
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

            /*telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());*/
            telemetry.addData("hiveTipped", skills.hiveTipped());

            if (follower.currentPath() != null) {
                telemetry.addData("path remaining", follower.distanceToEndpoint());
                telemetry.addData("path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

}
