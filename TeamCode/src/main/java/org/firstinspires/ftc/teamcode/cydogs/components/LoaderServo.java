package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseServo;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

/**
 * LoaderServo - the servo that pushes a scoring element into the launcher.
 *
 * It is a BaseServo, so it inherits Initialize and SetPosition. Its name, direction, limits and
 * positions come from BeethovenHardware.Loader.
 *
 * Usage: make a LoaderServo, call Initialize() once, then call LaunchOnce() on a button press and
 * Update() every loop so the servo returns to Load by itself.
 */
public class LoaderServo extends BaseServo {

        // Sets up the servo using the settings in BeethovenHardware.Loader
        public LoaderServo(OpMode opMode)
        {
                super(opMode, BeethovenHardware.Loader.NAME, BeethovenHardware.Loader.DIRECTION,
                        BeethovenHardware.Loader.MIN_POSITION, BeethovenHardware.Loader.MAX_POSITION,
                        BeethovenHardware.Loader.STARTING_POSITION);
        }

        // Moves to the position that holds the next element, ready to be pushed
        public void Load()
        {
                this.SetPosition(BeethovenHardware.Loader.LOAD_POSITION);
        }

        // Moves to the position that pushes the element into the launcher
        public void Launch()
        {
                this.SetPosition(BeethovenHardware.Loader.LAUNCH_POSITION);
        }

        // Times how long the servo has been holding the Launch position
        private final ElapsedTime launchTimer = new ElapsedTime();
        private boolean launching = false;

        /** One push: go to Launch now, then Update() returns to Load after LAUNCH_HOLD_SECONDS.
         *  Ignored if a launch is already in progress. */
        public void LaunchOnce()
        {
                if (launching) {
                        return;
                }
                Launch();
                launchTimer.reset();
                launching = true;
        }

        /** Call every loop. Returns the servo to Load when the hold time is up. */
        public void Update()
        {
                if (launching && launchTimer.seconds() >= BeethovenHardware.Loader.LAUNCH_HOLD_SECONDS) {
                        Load();
                        launching = false;
                }
        }

        // True while a launch push is in progress
        public boolean IsLaunching()
        {
                return launching;
        }

}