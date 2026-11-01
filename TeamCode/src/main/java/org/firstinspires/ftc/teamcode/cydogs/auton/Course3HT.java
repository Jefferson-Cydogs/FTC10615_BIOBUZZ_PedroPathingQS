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
@Autonomous(name = "Course3HT", group = "Autonomous")
public class Course3HT extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Alliance for the tag (change to RED if needed)
    private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;
    // Turret: 180 = straight ahead, which is right at the hive from the far shoot pose
    private static final double TIP_LOOK_DEGREES = 180.0;
    // How long we wait for the partner's tip before going to help (tune on field)
    private static final long PARTNER_TIMEOUT_MS = 10000;
    // Pause at the flower after collect so the intake can finish (same as other courses)
    private static final long FLOWER_COLLECT_WAIT_MS = 1500;

    // --- Poses (shared poses copied from Course 1, which is field-tuned) ---
    private final Pose start = poseFactory.of(56, 132.6875, 90);
    private final Pose shoot = poseFactory.of(61.25, 126, 90);

    // Tail A: far flower (Course 1)
    private final Pose flowerApproach = poseFactory.of(48, 115, 90);
    private final Pose flowerApproachControl1 = poseFactory.of(56, 110, 0);
    private final Pose flowerCollect = poseFactory.of(48, 127, 90);
    private final Pose shoot3 = poseFactory.of(61.25, 126, 90);
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);

    // Tail B: Course 1 route in reverse
    private final Pose shoot2Half1 = poseFactory.of(36, 108, 90);
    private final Pose shoot2Half2Control1 = poseFactory.of(60, 108, 0);
    private final Pose gardenApproach = poseFactory.of(28, 8.75, 180);
    private final Pose gardenApproachControl1 = poseFactory.of(35, 20, 0);
    private final Pose nearShoot = poseFactory.of(60.25, 12, 270);
    private final Pose nearPark = poseFactory.of(9.8125, 94.5, 0);

    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------

    /** Start → far shoot pose (short, straight out from the wall). */
    public Path shoot() {
        return Paths.line(start, shoot).linear(start, shoot);
    }

    // Tail A (same as Course 1's far side)

    // Full speed: the 40% cap lives on flowerCollect, the leg that touches the flower
    public Path flowerApproach() {
        return Paths.curve(shoot, flowerApproachControl1, flowerApproach)
                .linear(shoot, flowerApproach);
    }

    public Path flowerCollect() {
        return Paths.curve(flowerApproach, flowerCollect)
                .linear(flowerApproach, flowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Reverse of flowerCollect. Ramp must stay down until this finishes.
    public Path flowerBackUp() {
        return Paths.line(flowerCollect, flowerApproach)
                .linear(flowerCollect, flowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Starts at flowerApproach (after back-up), not flowerCollect
    public Path shoot3() {
        return Paths.line(flowerApproach, shoot3).linear(flowerApproach, shoot3);
    }

    public Path park() {
        return Paths.line(shoot3, park).linear(shoot3, park);
    }

    // Tail B (Course 1's route to the far side, driven in reverse)

    /** Far shoot pose → half-way point (reverse of Course 1's Shoot2Half2). */
    public Path tailB_ToHalfway() {
        return Paths.curve(shoot, shoot2Half2Control1, shoot2Half1)
                .linear(shoot, shoot2Half1);
    }

    /** Half-way point → end of the garden approach (same line as Course2HT's bail-out, reversed). */
    public Path tailB_ToGardenApproach() {
        return Paths.line(shoot2Half1, gardenApproach).linear(shoot2Half1, gardenApproach);
    }

    /** End of the garden approach → near shoot pose (reverse of Course 1's GardenApproach). */
    public Path tailB_ToNearShoot() {
        return Paths.curve(gardenApproach, gardenApproachControl1, nearShoot)
                .linear(gardenApproach, nearShoot);
    }

    public Path tailB_Park() {
        return Paths.line(nearShoot, nearPark).linear(nearShoot, nearPark);
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
                skills.startIntake(),
                follow(follower, flowerCollect()),
                waitMs(FLOWER_COLLECT_WAIT_MS),
                skills.stopIntake(),
                follow(follower, flowerBackUp()),
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
                follow(follower, shoot()),
                skills.waitForPartnerTipOrTimeout(ALLIANCE, TIP_LOOK_DEGREES, PARTNER_TIMEOUT_MS)
        );
    }

    /** Tail A — partner tipped: shoot, far flower, shoot again (our tip), park. */
    private Command tailA_partnerTipped() {
        return sequential(
                skills.turretStraight(),
                skills.shootFourPollen(),

                follow(follower, flowerApproach()),
                doFlower(),

                follow(follower, shoot3()),
                skills.shootFourPollen(),

                follow(follower, park())
        );
    }

    /**
     * Tail B — partner never tipped: go to their side and shoot our preloaded 4
     * (enough to tip the first hive), then park.
     */
    private Command tailB_partnerMissed() {
        return sequential(
                skills.turretStraight(),
                follow(follower, tailB_ToHalfway()),
                follow(follower, tailB_ToGardenApproach()),
                follow(follower, tailB_ToNearShoot()),
                skills.shootFourPollen(),
                follow(follower, tailB_Park())
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
        skills.prepareVision(ALLIANCE);

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

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("hiveTipped", skills.hiveTipped());

            if (follower.currentPath() != null) {
                telemetry.addData("path remaining", follower.distanceToEndpoint());
                telemetry.addData("path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}