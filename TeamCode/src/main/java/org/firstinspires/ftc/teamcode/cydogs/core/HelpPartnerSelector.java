package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Asks "Should we go help our partner after shooting on our side?" Yes or No.
 * If nobody confirms an answer, shouldHelpPartner() returns false.
 *
 * By itself: call update() in the INIT loop.
 * With other questions: add it to an AutonQuestions instead.
 */
public class HelpPartnerSelector extends InitQuestion<Boolean> {

    public HelpPartnerSelector(OpMode opMode) {
        super(opMode,
                "Help partner",
                "Should we go help our partner after shooting on our side?",
                new Boolean[] { true, false },
                false);
    }

    /** True = go help our partner, false = don't (also false if not answered). */
    public boolean shouldHelpPartner() {
        return getSelection();
    }

    @Override
    protected String label(Boolean value) {
        return value ? "Yes" : "No";
    }
}