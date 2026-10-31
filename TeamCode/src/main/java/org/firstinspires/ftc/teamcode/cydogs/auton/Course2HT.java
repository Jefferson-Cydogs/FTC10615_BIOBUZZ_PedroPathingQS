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
 * Course2HT — First Shot Triple Tip + tails, tip checks at the garden.
 *
 * Head: near shoot → garden collect → checkForTip (same meaning as Course1HT)
 *   Lock  = our basket still up → we did NOT tip → Tail A (go back, shoot, park)
 *   No lock = we tipped → wait for partner tip (see hive again) or timeout → Tail B / happy
 *
 * Both tails start after garden collect (easy shared entry pose).
 *
 * AutoSkills (same as Course1HT):
 *   checkForTip(alliance, lookDegrees)
 *   branchOnTip(ifNoLock / tipped, ifLock / no tip)  — match whatever Course1HT uses
 *   waitForPartnerTipOrTimeout() or equivalent
 *   flowerRampDown / flowerRampUp, startIntake / stopIntake, shootFourPollen
 */
@Autonomous(name = "Course2HT", group = "Autonomous")
public class Course2HT extends LinearOpMode {

    private Follower follower;
    private AutoSkills skills;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Alliance for tip checks (change to RED if needed)
    private static final TagSight.Alliance ALLIANCE = TagSight.Alliance.BLUE;
    private static final double TIP_LOOK_DEGREES = 0.0; // preset aim while checking, same idea as C1HT

    // --- Near shot (Course 1 coordinates) ---
    private final Pose start = poseFactory.of(60.25, 9.8125, 270);
    private final Pose shoot = poseFactory.of(60.25, 12, 270);

    // --- Garden (Course 1 split) ---
    private final Pose gardenApproach = poseFactory.of(28, 8.75, 180);
    private final Pose gardenApproachControl1 = poseFactory.of(35, 20, 0);
    private final Pose gardenCollect = poseFactory.of(12.5, 8.75, 180);

    // --- Near shoot again ---
    private final Pose shoot2 = poseFactory.of(60.25, 12, 270);

    // --- Near flower (Course 2 flower, split) ---
    private final Pose flowerApproach = poseFactory.of(22, 48, 0);
    private final Pose flower = poseFactory.of(14.8125, 48, 0);

    private final Pose shoot3 = poseFactory.of(60.25, 12, 270);
    private final Pose park = poseFactory.of(9.8125, 94.5, 0);

    // --- Tail B: Course 1 post-garden (far side) — from gardenCollect ---
    private final Pose shoot2Half1 = poseFactory.of(36, 108, 90);
    private final Pose shoot2Half2 = poseFactory.of(61.25, 126, 90);
    private final Pose shoot2Half2Control1 = poseFactory.of(60, 108, 0);
    private final Pose farFlowerApproach = poseFactory.of(48, 115, 90);
    private final Pose farFlowerApproachControl1 = poseFactory.of(56, 110, 0);
    private final Pose farFlowerCollect = poseFactory.of(48, 127, 90);
    private final Pose farShoot3 = poseFactory.of(61.25, 126, 90);
    private final Pose farPark = poseFactory.of(9.8125, 94.5, 0);

    private static final long FLOWER_COLLECT_MS = 1500;

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

    /** Back to near shoot from garden (Tail A / happy second shot). */
    public Path shoot2() {
        return Paths.line(gardenCollect, shoot2).linear(gardenCollect, shoot2);
    }

    public Path flowerApproach() {
        return Paths.line(shoot2, flowerApproach)
                .linear(shoot2, flowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    public Path flower() {
        return Paths.line(flowerApproach, flower)
                .linear(flowerApproach, flower)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    public Path shoot3() {
        return Paths.line(flower, shoot3).linear(flower, shoot3);
    }

    public Path park() {
        return Paths.line(shoot3, park).linear(shoot3, park);
    }

    public Path parkFromShoot2() {
        return Paths.line(shoot2, park).linear(shoot2, park);
    }

    // Tail B from gardenCollect (same entry as after head)
    public Path tailB_Shoot2Half1() {
        return Paths.curve(gardenCollect, shoot2Half1).linear(gardenCollect, shoot2Half1);
    }

    public Path tailB_Shoot2Half2() {
        return Paths.curve(shoot2Half1, shoot2Half2Control1, shoot2Half2)
                .linear(shoot2Half1, shoot2Half2);
    }

    public Path tailB_FarFlowerApproach() {
        return Paths.curve(shoot2Half2, farFlowerApproachControl1, farFlowerApproach)
                .linear(shoot2Half2, farFlowerApproach)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    public Path tailB_FarFlowerCollect() {
        return Paths.curve(farFlowerApproach, farFlowerCollect)
                .linear(farFlowerApproach, farFlowerCollect);
    }

    public Path tailB_FarShoot3() {
        return Paths.line(farFlowerCollect, farShoot3).linear(farFlowerCollect, farShoot3);
    }

    public Path tailB_FarPark() {
        return Paths.line(farShoot3, farPark).linear(farShoot3, farPark);
    }

    // -------------------------------------------------------------------------
    // Command pieces
    // -------------------------------------------------------------------------

    /**
     * Head: first shot, then straight to garden, then tip check (Course1HT style).
     * After this we are parked at gardenCollect for every branch.
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

                skills.checkForTip(ALLIANCE, TIP_LOOK_DEGREES)
        );
    }

    private Command doNearFlower() {
        return sequential(
                follow(follower, flowerApproach()),
                skills.flowerRampDown(),
                skills.startIntake(),
                follow(follower, flower()),
                waitMs(FLOWER_COLLECT_MS),
                skills.stopIntake(),
                skills.flowerRampUp()
        );
    }

    private Command doFarFlower() {
        return sequential(
                follow(follower, tailB_FarFlowerApproach()),
                skills.flowerRampDown(),
                skills.startIntake(),
                follow(follower, tailB_FarFlowerCollect()),
                waitMs(FLOWER_COLLECT_MS),
                skills.stopIntake(),
                skills.flowerRampUp()
        );
    }

    /**
     * Tail A — still locked on our hive at garden = we did NOT tip.
     * Go back to near shoot, shoot, park.
     */
    private Command tailA_weMissedFirstTip() {
        return sequential(
                follow(follower, shoot2()),
                skills.shootFourPollen(),
                follow(follower, parkFromShoot2())
        );
    }

    /**
     * Tail B — partner never tipped (timeout while waiting at garden).
     * Course 1 after-garden style from gardenCollect → their side.
     */
    private Command tailB_partnerMissed() {
        return sequential(
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
     * We already tipped (no lock at garden).
     * Stay at garden: wait until we see the hive again (partner tipped) or timeout.
     *   Seen again → near shoot → flower → shoot → park (happy triple)
     *   Timeout   → Tail B (their side)
     *
     * waitForPartnerTipOrTimeout() should set the same tip flag checkForTip uses,
     * or expose branchOnPartnerTip(yes, no) — mirror Course1HT naming if different.
     */
    private Command afterWeTipped() {
        return sequential(
                skills.waitForPartnerTipOrTimeout(ALLIANCE, TIP_LOOK_DEGREES),
                skills.branchOnTip(
                        // Partner tipped (hive visible again as tipped state / your C1HT rule)
                        sequential(
                                follow(follower, shoot2()),
                                skills.shootFourPollen(),
                                doNearFlower(),
                                follow(follower, shoot3()),
                                skills.shootFourPollen(),
                                follow(follower, park())
                        ),
                        tailB_partnerMissed()
                )
        );
    }

    // -------------------------------------------------------------------------
    // Main routine
    // -------------------------------------------------------------------------

    public Command autoRoutine() {
        return sequential(
                head(),
                // After garden + checkForTip:
                //   Lock (no tip)  → Tail A
                //   No lock (tip)  → wait for partner / timeout → happy or Tail B
                //
                // Match Course1HT branch polarity: lock = no tip = first arg or second?
                // Course1HT comment: Lock = basket still up → no tip.
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