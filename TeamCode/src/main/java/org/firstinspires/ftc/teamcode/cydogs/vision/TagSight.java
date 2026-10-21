package org.firstinspires.ftc.teamcode.cydogs.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * TagSight = the robot's "eyes" for BIOBUZZ AprilTags.
 * Hive cells use a 4-tag CLUSTER under each cell. The SDK only needs ONE
 * member tag fully visible to report the whole cluster. Pose aims near the
 * center of the cell opening (not the center of one sticker).
 * Filters:
 *   - Alliance color in the cluster/tag NAME (red or blue)
 *   - Upright face: |roll| < 90 degrees (scorable cell, not the down side)
 * Camera starts OFF. Call openCamera() for TRACK/SEARCH, stopCamera() for
 * PRESET/HOLD. Call close() when the OpMode ends.
 * How to use:
 *   1. new TagSight(hardwareMap, TagSight.Alliance.RED)
 *   2. openCamera() when aiming
 *   3. eyes.update() each loop
 *   4. Read hasTarget, bearingDegrees, rangeInches, etc.
 * Webcam config name must be "Webcam 1".
 */
public class TagSight {

    /** Which alliance tags/clusters we accept. */
    public enum Alliance {
        RED,   // name must contain "red"
        BLUE,  // name must contain "blue"
        BOTH   // any color
    }

    private static final String WEBCAM_NAME = "Webcam 1";

    // |roll| below this = right-side-up / scorable face
    private static final double UPRIGHT_LIMIT = 90.0;

    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private final Alliance alliance;

    private boolean cameraOpen = false;

    // ----- Results after update() -----

    /** True if we have a valid upright target for our alliance */
    public boolean hasTarget = false;

    /** Left/right angle to aim (degrees). 0 = centered */
    public double bearingDegrees = 0;

    /** Distance to the target (inches) — useful later for shot power */
    public double rangeInches = 0;

    /** Roll (degrees). Used to decide upright vs upside-down */
    public double rollDegrees = 0;

    /** True when |roll| says upright */
    public boolean isUpright = false;

    /** Full or short name from the SDK (telemetry) */
    public String targetName = "";

    /**
     * How much of the 4-tag cluster is visible (25, 50, 75, or 100).
     * 0 if not a cluster / unknown. Partial is OK — do not require 100.
     */
    public int percentClusterFound = 0;

    /** Single-tag id, or -1 for a cluster */
    public int tagId = -1;

    /**
     * @param hardwareMap from the OpMode
     * @param alliance    RED, BLUE, or BOTH
     */
    public TagSight(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // BIOBUZZ library: 4-tag clusters per hive cell
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(processor)
                .build();

        // Start with camera OFF to save CPU (Aimer turns it on for track/search)
        stopCamera();
    }

    /** Start the webcam + AprilTag pipeline. Call for TRACK or SEARCH. */
    public void openCamera() {
        if (portal == null) {
            return;
        }
        if (!cameraOpen) {
            portal.resumeStreaming();
            cameraOpen = true;
        }
    }

    /** Stop streaming frames. Call for PRESET or HOLD. */
    public void stopCamera() {
        if (portal == null) {
            return;
        }
        if (cameraOpen || portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            portal.stopStreaming();
        }
        cameraOpen = false;
        clearResults();
    }

    public boolean isCameraOpen() {
        return cameraOpen;
    }

    /**
     * Read the latest detections and pick the best valid target.
     * Safe to call when the camera is off (hasTarget stays false).
     */
    public void update() {
        clearResults();

        if (!cameraOpen) {
            return;
        }

        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return;
        }

        // Among all valid targets, keep the one closest to camera center
        double bestAbsBearing = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {

            // ----- BIOBUZZ hive: 4-tag cluster (main path) -----
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.ftcPose == null || cluster.metadata == null) {
                    continue;
                }

                // Alliance from FULL name (e.g. "RED SCORING"), not shortName only
                String fullName = cluster.metadata.name != null ? cluster.metadata.name : "";
                String displayName = fullName;
                if (cluster.metadata.shortName != null && !cluster.metadata.shortName.isEmpty()) {
                    displayName = cluster.metadata.shortName;
                }

                if (!matchesAlliance(fullName)) {
                    continue;
                }
                if (!isUprightRoll(cluster.ftcPose.roll)) {
                    continue;
                }

                // Partial cluster is OK (one visible tag can still give cell pose)
                double absBearing = Math.abs(cluster.ftcPose.bearing);
                if (absBearing < bestAbsBearing) {
                    bestAbsBearing = absBearing;
                    hasTarget = true;
                    bearingDegrees = cluster.ftcPose.bearing;
                    rangeInches = cluster.ftcPose.range;
                    rollDegrees = cluster.ftcPose.roll;
                    isUpright = true;
                    targetName = displayName;
                    percentClusterFound = cluster.percentClusterFound;
                    tagId = -1;
                }
                continue;
            }

            // ----- Single tags (practice tags only; hive field tags are clusters) -----
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.ftcPose == null) {
                    continue;
                }

                String name = "id " + single.id;
                if (single.metadata != null && single.metadata.name != null) {
                    name = single.metadata.name;
                }

                if (!matchesAlliance(name)) {
                    continue;
                }
                if (!isUprightRoll(single.ftcPose.roll)) {
                    continue;
                }

                double absBearing = Math.abs(single.ftcPose.bearing);
                if (absBearing < bestAbsBearing) {
                    bestAbsBearing = absBearing;
                    hasTarget = true;
                    bearingDegrees = single.ftcPose.bearing;
                    rangeInches = single.ftcPose.range;
                    rollDegrees = single.ftcPose.roll;
                    isUpright = true;
                    targetName = name;
                    percentClusterFound = 0;
                    tagId = single.id;
                }
            }
        }
    }

    private void clearResults() {
        hasTarget = false;
        bearingDegrees = 0;
        rangeInches = 0;
        rollDegrees = 0;
        isUpright = false;
        targetName = "";
        percentClusterFound = 0;
        tagId = -1;
    }

    /** RED/BLUE = name contains that word; BOTH = always true. */
    private boolean matchesAlliance(String name) {
        if (alliance == Alliance.BOTH) {
            return true;
        }
        if (name == null || name.isEmpty()) {
            return false;
        }
        String lower = name.toLowerCase();
        if (alliance == Alliance.RED) {
            return lower.contains("red");
        }
        if (alliance == Alliance.BLUE) {
            return lower.contains("blue");
        }
        return true;
    }

    private boolean isUprightRoll(double roll) {
        return Math.abs(roll) < UPRIGHT_LIMIT;
    }

    /** Fully release the camera when the OpMode is done. */
    public void close() {
        stopCamera();
        if (portal != null) {
            portal.close();
        }
    }
}