package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Asks a list of InitQuestions one at a time during auton INIT.
 * Each auton adds only the questions it needs, in the order it wants them asked.
 *
 *   A on a confirmed answer moves to the next question.
 *   B while choosing goes back to the previous question.
 *   When every question is answered, a summary shows all the answers.
 *
 * How to use it in an auton's runOpMode():
 *
 *   ParkSideSelector parkSelector = new ParkSideSelector(this);
 *   HelpPartnerSelector helpSelector = new HelpPartnerSelector(this);
 *
 *   AutonQuestions questions = new AutonQuestions(this);
 *   questions.add(parkSelector);
 *   questions.add(helpSelector);
 *
 *   while (!isStarted() && !isStopRequested()) {
 *       questions.update();          // asks the current question
 *       skills.updateInitVision();   // camera check still works
 *       telemetry.update();
 *   }
 *
 *   waitForStart();
 *   ParkSide parkSide = parkSelector.getSelection();      // LEFT, RIGHT or NONE
 *   boolean help = helpSelector.shouldHelpPartner();      // false if not answered
 *
 * Don't also call update() on the selectors themselves; AutonQuestions does that.
 */
public class AutonQuestions {

    private final OpMode opMode;
    private final List<InitQuestion<?>> questions = new ArrayList<>();
    private int currentIndex = 0;

    public AutonQuestions(OpMode opMode) {
        this.opMode = opMode;
    }

    /** Adds a question. Questions are asked in the order they are added. */
    public void add(InitQuestion<?> question) {
        questions.add(question);
    }

    /** Call once per INIT loop. Reads gamepad1 and adds the menu to telemetry. */
    public void update() {
        if (questions.isEmpty()) {
            return;
        }

        InitQuestion<?> current = questions.get(currentIndex);
        current.readButtons();

        if (current.isConfirmed() && currentIndex < questions.size() - 1) {
            // Answered: move on to the next question
            currentIndex++;
            questions.get(currentIndex).syncButtons();
        } else if (current.wasBackRequested() && currentIndex > 0) {
            // B while choosing: go back and re-answer the previous question
            currentIndex--;
            InitQuestion<?> previous = questions.get(currentIndex);
            previous.reopen();
            previous.syncButtons();
        }

        showTelemetry();
    }

    /** True when every question has a confirmed answer. */
    public boolean allAnswered() {
        for (InitQuestion<?> question : questions) {
            if (!question.isConfirmed()) {
                return false;
            }
        }
        return true;
    }

    private void showTelemetry() {
        Telemetry t = opMode.telemetry;

        if (allAnswered()) {
            t.addLine("=== AUTON QUESTIONS: ALL ANSWERED ===");
            for (InitQuestion<?> question : questions) {
                t.addLine(question.getTitle() + ": " + question.getSelectionLabel());
            }
            t.addLine("B to change the last answer");
            t.addLine();
            return;
        }

        InitQuestion<?> current = questions.get(currentIndex);
        t.addLine("=== AUTON QUESTION " + (currentIndex + 1) + " of " + questions.size() + " ===");
        if (currentIndex > 0 && current.isChoosing()) {
            t.addLine("(B = back to previous question)");
        }
        current.showTelemetry();
    }
}