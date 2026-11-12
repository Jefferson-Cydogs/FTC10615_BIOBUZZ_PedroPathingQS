package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Telemetry menu for picking the park side during auton INIT.
 * Works like the Servo Config menu:
 *
 *   D-pad Up/Down  move the >> pointer between Left and Right
 *   A              select the side the pointer is on
 *   A (again)      confirm it      (B = go back instead)
 *   B              after confirming, unlock it to change your pick
 *
 * If nobody confirms a side before START, getSelection() returns ParkSide.NONE
 * and the auton decides what to do.
 *
 * How to use it in an auton's runOpMode():
 *
 *   ParkSideSelector parkSelector = new ParkSideSelector(this);
 *
 *   while (!isStarted() && !isStopRequested()) {
 *       parkSelector.update();       // adds the menu lines to telemetry
 *       skills.updateInitVision();   // existing camera check still works
 *       telemetry.update();          // the init loop still calls update() itself
 *   }
 *
 *   waitForStart();
 *   ParkSide parkSide = parkSelector.getSelection();   // LEFT, RIGHT or NONE
 *
 * update() never calls telemetry.update() and never waits, so it can share the
 * init loop with other code.
 */
public class ParkSideSelector {

    /** Where the drive team is in the menu. */
    private enum Step { CHOOSING, CONFIRMING, LOCKED }

    /** The choices in the menu, top to bottom. */
    private static final ParkSide[] OPTIONS = { ParkSide.LEFT, ParkSide.RIGHT };

    private final OpMode opMode;

    private Step step = Step.CHOOSING;
    private int pointerIndex = 0;              // which option the >> is on
    private ParkSide selection = ParkSide.NONE; // only set after confirm

    // Last loop's buttons, so each press counts only once
    private boolean prevDpadUp, prevDpadDown, prevA, prevB;

    /** Pass in the OpMode (in a LinearOpMode, pass "this"). */
    public ParkSideSelector(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Call once per INIT loop. Reads gamepad1 and adds the menu to telemetry. */
    public void update() {
        Gamepad gp = opMode.gamepad1;

        boolean upPressed = gp.dpad_up && !prevDpadUp;
        boolean downPressed = gp.dpad_down && !prevDpadDown;
        boolean aPressed = gp.a && !prevA;
        boolean bPressed = gp.b && !prevB;

        prevDpadUp = gp.dpad_up;
        prevDpadDown = gp.dpad_down;
        prevA = gp.a;
        prevB = gp.b;

        switch (step) {
            case CHOOSING:
                if (upPressed) {
                    pointerIndex = (pointerIndex - 1 + OPTIONS.length) % OPTIONS.length;
                } else if (downPressed) {
                    pointerIndex = (pointerIndex + 1) % OPTIONS.length;
                } else if (aPressed) {
                    step = Step.CONFIRMING;
                }
                break;

            case CONFIRMING:
                if (aPressed) {
                    selection = OPTIONS[pointerIndex];
                    step = Step.LOCKED;
                } else if (bPressed) {
                    step = Step.CHOOSING;
                }
                break;

            case LOCKED:
                if (bPressed) {
                    selection = ParkSide.NONE;
                    step = Step.CHOOSING;
                }
                break;
        }

        showTelemetry();
    }

    /** LEFT or RIGHT once confirmed, otherwise NONE. */
    public ParkSide getSelection() {
        return selection;
    }

    /** True when a side has been confirmed. */
    public boolean isConfirmed() {
        return selection != ParkSide.NONE;
    }

    private void showTelemetry() {
        Telemetry t = opMode.telemetry;

        switch (step) {
            case CHOOSING:
                t.addLine("=== PARK SIDE: NOT SELECTED ===");
                t.addLine("D-pad Up/Down to move, A to select");
                for (int i = 0; i < OPTIONS.length; i++) {
                    String marker = (i == pointerIndex) ? ">> " : "    ";
                    t.addLine(marker + OPTIONS[i].label());
                }
                break;

            case CONFIRMING:
                t.addLine("=== PARK SIDE: CONFIRM ===");
                t.addLine("Park on " + OPTIONS[pointerIndex].label().toUpperCase() + "?");
                t.addLine("A = confirm,  B = go back");
                break;

            case LOCKED:
                t.addLine("=== PARK SIDE: " + selection.label().toUpperCase() + " (confirmed) ===");
                t.addLine("B to change");
                break;
        }
        t.addLine(); // blank line before the next section (camera status, etc.)
    }
}