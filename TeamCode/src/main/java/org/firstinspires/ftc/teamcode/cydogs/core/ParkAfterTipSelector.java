package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Asks "Should we park after getting the tip on our side?" Yes or No.
 * If nobody confirms an answer, shouldPark() returns false.
 *
 * By itself: call update() in the INIT loop.
 * With other questions: add it to an AutonQuestions instead.
 */
public class ParkAfterTipSelector extends InitQuestion<Boolean> {

    public ParkAfterTipSelector(OpMode opMode) {
        super(opMode,
                "Park After Tip",
                "Should we park after getting the tip on our side?",
                new Boolean[] { true, false },
                false);
    }

    /** True = go park after the tip, false = don't (also false if not answered). */
    public boolean shouldPark() {
        return getSelection();
    }

    @Override
    protected String label(Boolean value) {
        return value ? "Yes" : "No";
    }
}