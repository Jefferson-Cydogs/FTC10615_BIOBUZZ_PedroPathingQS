package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseColorSensor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;
import org.firstinspires.ftc.teamcode.cydogs.core.TargetColor;

public class FlingyMag {
    public BaseColorSensor ball0ScannerRight;
    public BaseColorSensor ball0ScannerLeft;
    public BaseColorSensor ball0ScannerMid;
    public String flingyReady = "none";   // what the launcher sensor sees
    public String flingyNext = "none";    // what the intake sensors see (left and right combined)

    // What each intake sensor saw the last time it was read
    private String leftColor = "none";
    private String rightColor = "none";

    // Constructor: creates the three color sensors (names come from BeethovenHardware.FlingyMag)
    public FlingyMag(OpMode opMode)
    {
        ball0ScannerRight = new BaseColorSensor(opMode,BeethovenHardware.FlingyMag.INTAKE_SENSOR_RIGHT_NAME);
        ball0ScannerLeft = new BaseColorSensor(opMode,BeethovenHardware.FlingyMag.INTAKE_SENSOR_LEFT_NAME);
        ball0ScannerMid = new BaseColorSensor(opMode,BeethovenHardware.FlingyMag.LAUNCHER_SENSOR_NAME);
    }

    // ---------- Public ----------

    /** Reads all three sensors and updates flingyReady and flingyNext. */
    public void IdentifyAll()
    {
        IdentifyLauncher();
        IdentifyRight();
        IdentifyLeft();
    }

    /**
     * True if the launcher sensor sees a ball of any color (yellow, red or blue).
     * Reads only the launcher sensor, fresh, each time it is called.
     */
    public boolean IsBallReadyForLaunch()
    {
        IdentifyLauncher();
        return !flingyReady.equals("none");
    }

    /**
     * True if either intake sensor sees a ball of any color.
     * Reads only the two intake sensors, fresh, each time it is called.
     */
    public boolean IsBallInMidPosition()
    {
        IdentifyRight();
        IdentifyLeft();
        return !flingyNext.equals("none");
    }

    // ---------- Private: one method per sensor ----------

    private void IdentifyLauncher()
    {
        flingyReady = readColor(ball0ScannerMid);
    }

    private void IdentifyRight()
    {
        rightColor = readColor(ball0ScannerRight);
        updateNext();
    }

    private void IdentifyLeft()
    {
        leftColor = readColor(ball0ScannerLeft);
        updateNext();
    }

    // ---------- Private: helpers ----------

    /** Reads one sensor. Checks yellow, then red, then blue, and stops at the first match. */
    private String readColor(BaseColorSensor sensor)
    {
        if (sensor.SeeArtifactColor(TargetColor.YELLOW))
        {
            return "yellow";
        }
        if (sensor.SeeArtifactColor(TargetColor.RED))
        {
            return "red";
        }
        if (sensor.SeeArtifactColor(TargetColor.BLUE))
        {
            return "blue";
        }
        return "none";
    }

    /** Combines the two intake sensors into flingyNext (yellow wins, then red, then blue). */
    private void updateNext()
    {
        if (rightColor.equals("yellow") || leftColor.equals("yellow"))
        {
            flingyNext = "yellow";
        }
        else if (rightColor.equals("red") || leftColor.equals("red"))
        {
            flingyNext = "red";
        }
        else if (rightColor.equals("blue") || leftColor.equals("blue"))
        {
            flingyNext = "blue";
        }
        else
        {
            flingyNext = "none";
        }
    }
}