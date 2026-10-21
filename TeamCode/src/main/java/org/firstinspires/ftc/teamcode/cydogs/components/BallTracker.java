package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BallTracker {
    public enum Alliance { RED, BLUE }

    // IDs match the order you trained the colors on the camera
    public static final int YELLOW_ID = 1, RED_ID = 2, BLUE_ID = 3;
    public static final int CENTER_X = 160;   // HuskyLens frame is 320x240

    private HuskyLens huskyLens;
    private ElapsedTime timer = new ElapsedTime();
    private double intervalMs;

    private Alliance alliance;
    private HuskyLens.Block target = null;

    public BallTracker(HardwareMap hardwareMap, String deviceName,
                        Alliance alliance, double intervalMs) {
        this.huskyLens = hardwareMap.get(HuskyLens.class, deviceName);
        this.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        this.alliance = alliance;
        this.intervalMs = intervalMs;
    }

    public BallTracker(HardwareMap hardwareMap, String deviceName, Alliance alliance) {
        this(hardwareMap, deviceName, alliance, 50);
    }

    /** Call every loop. Only reads the camera every intervalMs. */
    public void update() {
        if (timer.milliseconds() < intervalMs) return;
        timer.reset();
        target = pickTarget(huskyLens.blocks());
    }

    private HuskyLens.Block pickTarget(HuskyLens.Block[] blocks) {
        int allianceId = (alliance == Alliance.RED) ? RED_ID : BLUE_ID;
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : blocks) {
            if (b.id != YELLOW_ID && b.id != allianceId) continue;
            if (best == null || area(b) > area(best)) best = b;
        }
        return best;
    }

    private static int area(HuskyLens.Block b) { return b.width * b.height; }

    public void setAlliance(Alliance alliance) { this.alliance = alliance; }
    public Alliance getAlliance() { return alliance; }

    public boolean isConnected() { return huskyLens.knock(); }
    public boolean hasTarget() { return target != null; }

    /** Pixels from frame center. Negative = ball is left, positive = right. */
    public double getXError() { return target == null ? 0 : target.x - CENTER_X; }

    /** Block width in px, a rough proxy for distance. 0 if no target. */
    public int getTargetWidth() { return target == null ? 0 : target.width; }

    public int getTargetId() { return target == null ? -1 : target.id; }

    public boolean isWithin(int widthThresholdPx) {
        return target != null && target.width >= widthThresholdPx;
    }
}
