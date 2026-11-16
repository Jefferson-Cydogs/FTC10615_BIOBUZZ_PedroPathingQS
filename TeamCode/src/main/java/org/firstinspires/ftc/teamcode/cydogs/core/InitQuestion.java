package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One multiple-choice question asked on telemetry during auton INIT.
 * ParkSideSelector, HelpPartnerSelector and SecondShotSelector are all built on this.
 *
 * Buttons (gamepad1), same as the Servo Config menu:
 *   D-pad Up/Down  move the >> pointer
 *   A              select the answer the pointer is on
 *   A (again)      confirm it      (B = go back instead)
 *   B              after confirming, unlock it to change the answer
 *
 * Until an answer is confirmed, getSelection() returns the "unanswered" value
 * the selector was built with (for example ParkSide.NONE).
 *
 * One question by itself: call update() in the INIT loop.
 * Two or more questions: add them to an AutonQuestions and call its update() instead,
 * so one button press doesn't answer every question at once.
 *
 * @param <T> the type of answer (an enum, Boolean, ...)
 */
public abstract class InitQuestion<T> {

    /** Where the drive team is on this question. */
    private enum Step { CHOOSING, CONFIRMING, LOCKED }

    private final OpMode opMode;
    private final String title;       // short name for headers and the summary, e.g. "Park side"
    private final String question;    // full question shown to the drive team
    private final T[] options;        // the choices, top to bottom
    private final T unansweredValue;  // returned if nobody confirms before START

    private Step step = Step.CHOOSING;
    private int pointerIndex = 0;     // which option the >> is on
    private T selection;              // the confirmed answer, or unansweredValue
    private boolean backRequested = false;

    // Last loop's buttons, so each press counts only once
    private boolean prevDpadUp, prevDpadDown, prevA, prevB;

    protected InitQuestion(OpMode opMode, String title, String question,
                           T[] options, T unansweredValue) {
        this.opMode = opMode;
        this.title = title;
        this.question = question;
        this.options = options;
        this.unansweredValue = unansweredValue;
        this.selection = unansweredValue;
    }

    /** Text shown for one answer, e.g. "Left" or "Yes". Each selector fills this in. */
    protected abstract String label(T value);

    // -------------------------------------------------------------------------
    // Public: what an auton uses
    // -------------------------------------------------------------------------

    /** Use this when the auton asks only this one question. Call once per INIT loop. */
    public void update() {
        readButtons();
        showTelemetry();
    }

    /** The confirmed answer, or the unanswered value if nobody confirmed one. */
    public T getSelection() {
        return selection;
    }

    /** True when an answer has been confirmed. */
    public boolean isConfirmed() {
        return step == Step.LOCKED;
    }

    /** Short name of this question, e.g. "Park side". */
    public String getTitle() {
        return title;
    }

    /** The current answer as text, e.g. "Left". */
    public String getSelectionLabel() {
        return label(selection);
    }

    // -------------------------------------------------------------------------
    // Package-private: used by AutonQuestions
    // -------------------------------------------------------------------------

    /** Reads gamepad1 and moves through choose → confirm → locked. No telemetry. */
    void readButtons() {
        Gamepad gp = opMode.gamepad1;

        boolean upPressed = gp.dpad_up && !prevDpadUp;
        boolean downPressed = gp.dpad_down && !prevDpadDown;
        boolean aPressed = gp.a && !prevA;
        boolean bPressed = gp.b && !prevB;
        syncButtons();

        backRequested = false;

        switch (step) {
            case CHOOSING:
                if (upPressed) {
                    pointerIndex = (pointerIndex - 1 + options.length) % options.length;
                } else if (downPressed) {
                    pointerIndex = (pointerIndex + 1) % options.length;
                } else if (aPressed) {
                    step = Step.CONFIRMING;
                } else if (bPressed) {
                    backRequested = true;   // AutonQuestions uses this to go to the previous question
                }
                break;

            case CONFIRMING:
                if (aPressed) {
                    selection = options[pointerIndex];
                    step = Step.LOCKED;
                } else if (bPressed) {
                    step = Step.CHOOSING;
                }
                break;

            case LOCKED:
                if (bPressed) {
                    reopen();
                }
                break;
        }
    }

    /** Adds this question's menu lines to telemetry. Never calls telemetry.update(). */
    void showTelemetry() {
        Telemetry t = opMode.telemetry;
        String name = title.toUpperCase();

        switch (step) {
            case CHOOSING:
                t.addLine("=== " + name + ": NOT ANSWERED ===");
                t.addLine(question);
                for (int i = 0; i < options.length; i++) {
                    String marker = (i == pointerIndex) ? ">> " : "    ";
                    t.addLine(marker + label(options[i]));
                }
                t.addLine("D-pad Up/Down to move, A to select");
                t.addLine("If not answered: " + label(unansweredValue));
                break;

            case CONFIRMING:
                t.addLine("=== " + name + ": CONFIRM ===");
                t.addLine(title + " = " + label(options[pointerIndex]).toUpperCase() + "?");
                t.addLine("A = confirm,  B = go back");
                break;

            case LOCKED:
                t.addLine("=== " + name + ": " + label(selection).toUpperCase() + " (confirmed) ===");
                t.addLine("B to change");
                break;
        }
        t.addLine(); // blank line before the next section (camera status, etc.)
    }

    /** True for one loop after B is pressed while choosing. */
    boolean wasBackRequested() {
        return backRequested;
    }

    /** True while the pointer is being moved (not confirming, not locked). */
    boolean isChoosing() {
        return step == Step.CHOOSING;
    }

    /** Clears the answer and goes back to choosing. */
    void reopen() {
        step = Step.CHOOSING;
        selection = unansweredValue;
    }

    /**
     * Remembers the buttons as they are right now, so a button that is still held
     * from the previous question doesn't count as a new press on this one.
     */
    void syncButtons() {
        Gamepad gp = opMode.gamepad1;
        prevDpadUp = gp.dpad_up;
        prevDpadDown = gp.dpad_down;
        prevA = gp.a;
        prevB = gp.b;
    }
}