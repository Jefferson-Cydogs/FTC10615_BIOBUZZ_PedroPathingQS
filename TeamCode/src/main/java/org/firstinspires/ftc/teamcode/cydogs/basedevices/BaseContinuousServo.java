package org.firstinspires.ftc.teamcode.cydogs.basedevices;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BaseContinuousServo {

    private final OpMode opMode;
    private final String hardwareName;
    private final DcMotorSimple.Direction direction;
    private final double runPower;
    private CRServo servo;
    private double currentRunningPower = 0;

    private int runToken = 0;          // changes every time the servo gets a new power command
    private int timedRunToken = -1;    // the runToken value that belongs to the timed run in progress


    /** runPower is the speed used by RunForward/RunBackward, from 0 to 1 (sign is ignored). */
    public BaseContinuousServo(OpMode opMode,
                               String hardwareName,
                               DcMotorSimple.Direction direction,
                               double runPower) {
        this.opMode = opMode;
        this.hardwareName = hardwareName;
        this.direction = direction;
        this.runPower = Math.min(1.0, Math.abs(runPower));
    }

    /** Gets the servo from the hardware map and applies direction. Call once before using the servo. */
    public void Initialize() {
        servo = opMode.hardwareMap.get(CRServo.class, hardwareName);
        servo.setDirection(direction);
        applyPower(0);
    }

    public void RunForward() {
        applyPower(runPower);
    }

    public void RunBackward() {
        applyPower(-runPower);
    }

    public void RunForward(double power) {
        applyPower(Math.abs(power));
    }

    public void RunBackward(double power) {
        applyPower(-Math.abs(power));
    }

    public void Stop() {
        applyPower(0);
    }

    /** Returns the last commanded power, from -1 to 1 (not a sensed value). */
    public double GetRunningPower() {
        return currentRunningPower;
    }

    /** Single place where power is sent to the servo, so the tracked value always matches. */
    private synchronized void applyPower(double power) {
        checkInitialized();
        runToken++;     // any new power command cancels a timed run in progress
        currentRunningPower = Math.max(-1.0, Math.min(1.0, power));
        servo.setPower(currentRunningPower);
    }

    private void checkInitialized() {
        if (servo == null) {
            throw new IllegalStateException(
                    "BaseContinuousServo '" + hardwareName + "' used before Initialize() was called");
        }
    }

    /**
     * Runs forward at the normal run power for the given number of seconds, then stops by itself.
     * Returns immediately; a background timer does the stopping. If a timed run is already in
     * progress, the request is ignored.
     */
    public synchronized void RunForwardForSecondsAsync(double seconds) {
        if (IsTimedRunActive()) return;
        RunForward();
        stopAfterSecondsAsync(seconds);
    }

    /** Same as RunForwardForSecondsAsync, but backward. */
    public synchronized void RunBackwardForSecondsAsync(double seconds) {
        if (IsTimedRunActive()) return;
        RunBackward();
        stopAfterSecondsAsync(seconds);
    }

    /** True while a timed run has started and has not yet finished or been cancelled. */
    public synchronized boolean IsTimedRunActive() {
        return timedRunToken == runToken;
    }

    private void stopAfterSecondsAsync(double seconds) {
        final int myToken = runToken;
        timedRunToken = myToken;
        final long waitMs = (long) (Math.max(0, seconds) * 1000);

        Thread timer = new Thread(() -> {
            try {
                Thread.sleep(waitMs);
                synchronized (this) {
                    // Only stop if nothing else has commanded the servo since this run began
                    if (runToken == myToken) {
                        Stop();
                    }
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();   // OpMode is shutting down; the SDK stops it
            } catch (RuntimeException e) {
                // OpMode already ended and the hardware is gone; nothing left to stop
            }
        });
        timer.setDaemon(true);
        timer.start();
    }
}