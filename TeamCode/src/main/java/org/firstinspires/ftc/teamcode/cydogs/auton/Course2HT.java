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
 * Course2HT — First Shot Triple Tip + tails, tip check at the NEAR SHOOT pose.
 *
 * Head: near shoot → garden collect → drive back to near shoot pose → checkForTip
 *   Lock    = our basket is up (seesaw on our side) → we did NOT tip → Tail 1
 *   No lock = our basket is down (we tipped) → wait for the tag to come back up
 *             (partner tipped) or timeout
 *
 * Tails (exactly one runs after the head):
 *   Tail 1 (tailA_weMissedFirstTip) — hive UP at first look: we missed.
 *            Shoot, then park. No third tip in this scenario.
 *   Tail 2 (tailB_partnerMissed)    — hive DOWN, never came up before timeout:
 *            we tipped, partner failed. Go to their side, shoot, collect their
 *            flower, shoot again, park.
 *   Tail 3 (tailC_bothTipped)       — hive DOWN, then came back UP before timeout:
 *            we tipped, partner tipped. Shoot, near flower, back to near shoot
 *            pose, shoot again (third tip), park.
 *
 * Flower sequence (both flowers):
 *   approach → ramp down → intake on → collect → wait → intake off → back up → ramp up → shoot
 *   (back up BEFORE raising the ramp, otherwise the ramp comes up under the flower)
 *
 * Seesaw rule (from the team): we only see the tag when our side is up.
 *   No tag = we are down (tipped).  Tag again = partner tipped.
 *
 * AutoSkills (same as Course1HT):
 *   checkForTip(alliance, lookDegrees)
 *   branchOnTip(tipped / no lock, still locked)
 *   waitForPartnerTipOrTimeout(alliance, lookDegrees)
 *   flowerRampDown / flowerRampUp, startIntake / stopIntake, shootFourPollen
 *   turretStraight
 */
@Autonomous(name = "Course2HT", group = "Autonomous")
public class Course2HT extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Alliance for tip checks (change to RED if needed)
    private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;
    // Turret: 180 = straight ahead. We shoot from the back, so we look straight too.
    private static final double TIP_LOOK_DEGREES = 180.0;

    // --- Near shot (Course 1 coordinates) ---
    private final Pose start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose shoot = poseFactory.of(60.25, 12, 270);

    // --- Garden (Course 1 split) ---
    private final Pose gardenApproach = poseFactory.of(28, 8.75, 180);
    private final Pose gardenApproachControl1 = poseFactory.of(35, 20, 0);
    private final Pose gardenCollect = poseFactory.of(12.5, 8.75, 180);

    // --- Near shoot again (tip check happens here) ---
    private final Pose shoot2 = poseFactory.of(60.25, 12, 270);

    // --- Near flower (Course 2 flower, split) ---
    private final Pose flowerApproach = poseFactory.of(22, 48, 0);
    private final Pose flower = poseFactory.of(14.8125, 48, 0);

    private final Pose shoot3 = poseFactory.of(60.25, 12, 270);
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);

    // --- Tail 2: Course 1 post-garden (far side) ---
    private final Pose shoot2Half1 = poseFactory.of(36, 108, 90);
    private final Pose shoot2Half2 = poseFactory.of(61.25, 126, 90);
    private final Pose shoot2Half2Control1 = poseFactory.of(60, 108, 0);
    private final Pose farFlowerApproach = poseFactory.of(48, 115, 90);
    private final Pose farFlowerApproachControl1 = poseFactory.of(56, 110, 0);
    private final Pose farFlowerCollect = poseFactory.of(48, 127, 90);
    private final Pose farShoot3 = poseFactory.of(61.25, 126, 90);
    private final Pose farPark = poseFactory.of(9.8125, 94.5, 0);

    // Pause at the flower after collect so the intake can finish pulling balls in (tune on field)
    private static final long FLOWER_COLLECT_WAIT_MS = 1500;

    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------

    public Path shoot() {
        return Paths.line(start, shoot).linear(start, shoot);
    }

    public Path GardenApproach() {
        return Paths.curve(shoot, gardenApproachControl1, gardenApproach)
                .linear(shoot, gardenApproach);
    }

    public Path GardenCollect() {
        return Paths.line(gardenApproach, gardenCollect)
                .linear(gardenApproach, gardenCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    /** Back to near shoot from garden (end of head; tip check happens here). */
    public Path shoot2() {
        return Paths.line(gardenCollect, shoot2).linear(gardenCollect, shoot2);
    }

    // Full speed: the 40% cap lives on flower(), the leg that touches the flower
    public Path flowerApproach() {
        return Paths.line(shoot2, flowerApproach)
                .linear(shoot2, flowerApproach);
    }

    public Path flower() {
        return Paths.line(flowerApproach, flower)
                .linear(flowerApproach, flower)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Reverse of flower(). Ramp must stay down until this finishes.
    public Path flowerBackUp() {
        return Paths.line(flower, flowerApproach)
                .linear(flower, flowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Starts at flowerApproach (after back-up), not flower
    public Path shoot3() {
        return Paths.line(flowerApproach, shoot3).linear(flowerApproach, shoot3);
    }

    public Path park() {
        return Paths.line(shoot3, park).linear(shoot3, park);
    }

    public Path parkFromShoot2() {
        return Paths.line(shoot2, park).linear(shoot2, park);
    }

    // Tail 2 now starts at the near shoot pose, goes out to the end of the
    // garden approach (safe lane, same as Course 1), then crosses to their side.

    /** Near shoot pose → end of garden approach (same curve as the head's approach). */
    public Path tailB_ToGardenApproach() {
        return Paths.curve(shoot2, gardenApproachControl1, gardenApproach)
                .linear(shoot2, gardenApproach);
    }

    /** End of garden approach → far side half-way point. */
    public Path tailB_Shoot2Half1() {
        return Paths.line(gardenApproach, shoot2Half1).linear(gardenApproach, shoot2Half1);
    }

    public Path tailB_Shoot2Half2() {
        return Paths.curve(shoot2Half1, shoot2Half2Control1, shoot2Half2)
                .linear(shoot2Half1, shoot2Half2);
    }

    // Full speed: the 40% cap lives on the collect leg
    public Path tailB_FarFlowerApproach() {
        return Paths.curve(shoot2Half2, farFlowerApproachControl1, farFlowerApproach)
                .linear(shoot2Half2, farFlowerApproach);
    }

    public Path tailB_FarFlowerCollect() {
        return Paths.curve(farFlowerApproach, farFlowerCollect)
                .linear(farFlowerApproach, farFlowerCollect)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Reverse of tailB_FarFlowerCollect. Ramp must stay down until this finishes.
    public Path tailB_FarFlowerBackUp() {
        return Paths.line(farFlowerCollect, farFlowerApproach)
                .linear(farFlowerCollect, farFlowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    // Starts at farFlowerApproach (after back-up), not farFlowerCollect
    public Path tailB_FarShoot3() {
        return Paths.line(farFlowerApproach, farShoot3).linear(farFlowerApproach, farShoot3);
    }

    public Path tailB_FarPark() {
        return Paths.line(farShoot3, farPark).linear(farShoot3, farPark);
    }

    // -------------------------------------------------------------------------
    // Command pieces
    // -------------------------------------------------------------------------

    /**
     * Head: first shot, garden, then drive BACK to the near shoot pose and
     * check the tip from there (turret straight, 180).
     * After this we are parked at the near shoot pose for every branch.
     *
     * checkForTip: Lock = basket up (NO tip). No lock = tipped.
     */
    private Command head() {
        return sequential(
                follow(follower, shoot()),
                skills.shootFourPollen(),

                follow(follower, GardenApproach()),
                skills.startIntake(),
                follow(follower, GardenCollect()),
                skills.stopIntake(),

                follow(follower, shoot2()),
                skills.checkForTip(ALLIANCE, TIP_LOOK_DEGREES)
        );
    }

    private Command doNearFlower() {
        return sequential(
                follow(follower, flowerApproach()),
                skills.flowerRampDown(),
                skills.startIntake(),
                follow(follower, flower()),
                waitMs(FLOWER_COLLECT_WAIT_MS),
                skills.stopIntake(),
                follow(follower, flowerBackUp()),
                skills.flowerRampUp()
        );
    }

    private Command doFarFlower() {
        return sequential(
                follow(follower, tailB_FarFlowerApproach()),
                skills.flowerRampDown(),
                skills.startIntake(),
                follow(follower, tailB_FarFlowerCollect()),
                waitMs(FLOWER_COLLECT_WAIT_MS),
                skills.stopIntake(),
                follow(follower, tailB_FarFlowerBackUp()),
                skills.flowerRampUp()
        );
    }

    /**
     * Tail 1 — hive UP at the first look = we did NOT tip (or partner was
     * very fast). We are already at the near shoot pose: shoot, then park.
     * No third tip in this scenario.
     */
    private Command tailA_weMissedFirstTip() {
        return sequential(
                skills.turretStraight(),
                skills.shootFourPollen(),
                follow(follower, parkFromShoot2())
        );
    }

    /**
     * Tail 2 — hive DOWN, never came back up (timeout): we tipped, partner failed.
     * From the near shoot pose: out to the end of the garden approach (safe lane),
     * across to their side, shoot, collect their flower, shoot again, park.
     */
    private Command tailB_partnerMissed() {
        return sequential(
                skills.turretStraight(),
                follow(follower, tailB_ToGardenApproach()),
                follow(follower, tailB_Shoot2Half1()),
                follow(follower, tailB_Shoot2Half2()),
                skills.shootFourPollen(),
                doFarFlower(),
                follow(follower, tailB_FarShoot3()),
                skills.shootFourPollen(),
                follow(follower, tailB_FarPark())
        );
    }

    /**
     * Tail 3 — hive DOWN, then came back UP before timeout: we tipped and
     * the partner tipped. Shoot, near flower, back to the near shoot pose,
     * shoot again (third tip), park.
     */
    private Command tailC_bothTipped() {
        return sequential(
                skills.turretStraight(),
                skills.shootFourPollen(),
                doNearFlower(),
                follow(follower, shoot3()),
                skills.shootFourPollen(),
                follow(follower, park())
        );
    }

    /**
     * We already tipped (no lock at the first look).
     * Stay at the near shoot pose and wait for the tag to come back (partner tipped) or timeout.
     *   Timeout (hiveTipped stays true)      → Tail 2 (their side)
     *   Lock before timeout (hiveTipped false) → Tail 3 (happy triple)
     *
     * branchOnTip(tippedCmd, stillLockedCmd): tipped = no lock = timeout.
     */
    private Command afterWeTipped() {
        return sequential(
                skills.waitForPartnerTipOrTimeout(ALLIANCE, TIP_LOOK_DEGREES),
                skills.branchOnTip(
                        tailB_partnerMissed(),   // timeout, no lock → partner failed
                        tailC_bothTipped()       // lock again → partner tipped
                )
        );
    }

    // -------------------------------------------------------------------------
    // Main routine
    // -------------------------------------------------------------------------

    public Command autoRoutine() {
        return sequential(
                head(),
                // After garden + back at near shoot + checkForTip:
                //   Lock (hive up, we missed) → Tail 1
                //   No lock (we tipped)       → wait for partner → Tail 2 or Tail 3
                skills.branchOnTip(
                        afterWeTipped(),           // no lock → we tipped
                        tailA_weMissedFirstTip()   // lock → we missed
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
        skills.prepareVision(ALLIANCE);   // open camera at init, same as Course1HT

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

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}