package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Asks "Should the 2nd shot be from the normal spot or the corner?"
 * If nobody confirms an answer, getSelection() returns SecondShotSpot.NORMAL.
 *
 * By itself: call update() in the INIT loop.
 * With other questions: add it to an AutonQuestions instead.
 */
public class SecondShotSelector extends InitQuestion<SecondShotSpot> {

    public SecondShotSelector(OpMode opMode) {
        super(opMode,
                "2nd shot",
                "Should the 2nd shot be from the normal spot or the corner?",
                new SecondShotSpot[] { SecondShotSpot.NORMAL, SecondShotSpot.CORNER },
                SecondShotSpot.NORMAL);
    }

    @Override
    protected String label(SecondShotSpot value) {
        return value.label();
    }
}