package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseLED;
import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseVelocityMotor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;

/**
 * Launcher - the spinning motor that throws scoring elements, plus an LED that shows whether
 * it has reached speed (RED = not yet, GREEN = ready).
 *
 * A Launcher IS a BaseVelocityMotor (so it inherits Initialize, SetVelocity, Stop and
 * IsAtVelocity) and HAS a BaseLED. The motor holds a target speed counted in encoder ticks per
 * second, so its encoder cable must be plugged in.
 *
 * The flywheel is heavy, so it keeps a lot of momentum. StopLauncher() does not just cut power:
 * it starts a controlled spin-down that brakes the flywheel at a steady rate. The spin-down runs
 * inside Update(), so there are no threads and it can be cancelled at any time.
 *
 * How an OpMode uses it:
 *   1. Make one:                 new Launcher(this)   (inside runOpMode/init, not as a field
 *                                initializer, because the LED connects to the hardware map as
 *                                soon as it is created)
 *   2. Call Initialize()         once (inherited).
 *   3. StartLauncher() / StopLauncher() from button presses.
 *   4. Call Update()             EVERY loop. Without it the spin-down never progresses.
 *   5. IncreaseLaunchPercent() / DecreaseLaunchPercent() to tune the speed in 1% steps. If the
 *      launcher is running, the new speed takes effect immediately.
 *   6. IsLauncherAtSpeed()       every loop. The LED only changes color when this runs.
 */
public class Launcher extends BaseVelocityMotor {

    // The launcher is always in exactly one of these states.
    private enum State {
        IDLE,           // stopped, nothing running
        RUNNING,        // holding the launch speed
        SPINNING_DOWN   // being braked toward zero by Update()
    }

    private State state = State.IDLE;

    // Measures the time between Update() calls so the spin-down rate does not depend on loop speed.
    private final ElapsedTime spinDownTimer = new ElapsedTime();

    // The speed indicator light.
    private final BaseLED speedLed;

    // Current launch speed as a fraction of top speed. Starts at BeethovenHardware.Launcher.LAUNCH_PERCENT and can be adjusted.
    private double launchVelocityPercent = BeethovenHardware.Launcher.LAUNCH_PERCENT;

    public Launcher(OpMode opMode) {
        super(opMode, BeethovenHardware.Launcher.MOTOR_NAME, BeethovenHardware.Launcher.DIRECTION, BeethovenHardware.Launcher.ZERO_POWER_BEHAVIOR);
        speedLed = new BaseLED(opMode.hardwareMap, BeethovenHardware.Launcher.LED_NAME);
    }

    /** Spins the launcher up to the current launch speed. Returns immediately.
     *  Also cancels a spin-down that is in progress. */
    public void StartLauncher() {
        state = State.RUNNING;
        SetVelocity(BeethovenHardware.Launcher.MAX_TICKS_PER_SECOND * launchVelocityPercent);
    }

    /**
     * Begins a controlled spin-down and turns the LED off. Returns immediately; Update() does the
     * braking over the next second or so. Safe to call every loop (for example while a button is
     * held): once a spin-down has started, extra calls do nothing. Does nothing if already stopped.
     */
    public void StopLauncher() {
        if (state != State.RUNNING) {
            return;
        }
        spinDownTimer.reset();
        state = State.SPINNING_DOWN;
        speedLed.setOff();
    }

    /** Cuts power right now with no controlled spin-down (the flywheel coasts). Use for emergencies. */
    public void StopLauncherNow() {
        Stop();
        state = State.IDLE;
        speedLed.setOff();
    }

    /**
     * Call this once every loop. While the launcher is spinning down, it sets the target speed
     * just below the flywheel's MEASURED speed, so the braking stays gentle and steady no matter
     * how much momentum the flywheel has. Does nothing in any other state.
     */
    public void Update() {
        if (state != State.SPINNING_DOWN) {
            return;
        }

        double secondsSinceLastUpdate = spinDownTimer.seconds();
        spinDownTimer.reset();

        double nextTargetTps = Math.abs(GetVelocity()) - BeethovenHardware.Launcher.SPINDOWN_TPS_PER_SEC * secondsSinceLastUpdate;

        if (nextTargetTps < BeethovenHardware.Launcher.SPINDOWN_MIN_TPS) {
            Stop();
            state = State.IDLE;
        } else {
            SetVelocity(nextTargetTps);
        }
    }

    /** True while the controlled spin-down is in progress (for telemetry, or to wait before parking). */
    public boolean IsSpinningDown() {
        return state == State.SPINNING_DOWN;
    }

    /**
     * True when the launcher is running and within 5% of its target speed. Also sets the LED:
     * green when at speed, red when not (including when stopped or spinning down).
     * Call this every loop.
     */
    public boolean IsLauncherAtSpeed() {
        boolean atSpeed = (state == State.RUNNING) && IsAtVelocity();
        if (atSpeed) {
            speedLed.setGreen();
        } else {
            speedLed.setRed();
        }
        return atSpeed;
    }

    /** Raises the launch speed by one step. If the launcher is running, applies it right away. */
    public void IncreaseLaunchPercent() {
        adjustLaunchPercent(BeethovenHardware.Launcher.PERCENT_STEP);
    }

    /** Lowers the launch speed by one step. If the launcher is running, applies it right away. */
    public void DecreaseLaunchPercent() {
        adjustLaunchPercent(-BeethovenHardware.Launcher.PERCENT_STEP);
    }

    /** The ACTUAL (measured) launch speed as a fraction of top speed (0.5 = 50%), for telemetry. */
    public double GetLaunchVelocityPercent() {
        return GetVelocity() / BeethovenHardware.Launcher.MAX_TICKS_PER_SECOND;
    }

    /** The TARGET launch speed as a fraction of top speed (what the D-pad adjusts), for telemetry. */
    public double GetTargetLaunchPercent() {
        return launchVelocityPercent;
    }

    private void adjustLaunchPercent(double change) {
        launchVelocityPercent = Math.max(0.0, Math.min(1.0, launchVelocityPercent + change));
        // Only re-apply the speed while running. During a spin-down or when stopped, just remember
        // the new percent; otherwise a D-pad press would restart the launcher by accident.
        if (state == State.RUNNING) {
            StartLauncher();
        }
    }
}