package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Asks which side of the loading zone to park on: Left or Right.
 * If nobody confirms an answer, getSelection() returns ParkSide.NONE
 * and the auton decides what to do.
 *
 * By itself: call update() in the INIT loop.
 * With other questions: add it to an AutonQuestions instead.
 */
public class ParkSideSelector extends InitQuestion<ParkSide> {

    public ParkSideSelector(OpMode opMode) {
        super(opMode,
                "Park side",
                "Which side of the loading zone should we park on?",
                new ParkSide[] { ParkSide.LEFT, ParkSide.RIGHT },
                ParkSide.NONE);
    }

    @Override
    protected String label(ParkSide value) {
        return value.label();
    }
}