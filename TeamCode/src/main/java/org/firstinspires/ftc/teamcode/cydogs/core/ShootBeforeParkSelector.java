package org.firstinspires.ftc.teamcode.cydogs.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Asks "Should we shoot before parking" Yes or No.
 * If nobody confirms an answer, shouldShoot() returns false.
 *
 * By itself: call update() in the INIT loop.
 * With other questions: add it to an AutonQuestions instead.
 */
public class ShootBeforeParkSelector extends InitQuestion<Boolean> {

    public ShootBeforeParkSelector(OpMode opMode) {
        super(opMode,
                "Shoot Before Park",
                "Should we shoot before parking?",
                new Boolean[] { true, false },
                false);
    }

    /** True = shoot before parking, false = don't (also false if not answered). */
    public boolean shouldShoot() {
        return getSelection();
    }

    @Override
    protected String label(Boolean value) {
        return value ? "Yes" : "No";
    }
}