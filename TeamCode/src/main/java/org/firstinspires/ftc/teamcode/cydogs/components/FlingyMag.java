package org.firstinspires.ftc.teamcode.cydogs.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.cydogs.basedevices.BaseColorSensor;
import org.firstinspires.ftc.teamcode.cydogs.chassis.BeethovenHardware;
import org.firstinspires.ftc.teamcode.cydogs.core.TargetColor;

public class FlingyMag {
    public BaseColorSensor ball0ScannerRight;
    public BaseColorSensor ball0ScannerLeft;
    public BaseColorSensor ball0ScannerMid;
    public String flingyReady;
    public String flingyNext;
    // Constructor: creates the three color sensors (names come from BeethovenHardware.FlingyMag)
    public FlingyMag(OpMode opMode)
    {
        ball0ScannerRight = new BaseColorSensor(opMode,BeethovenHardware.FlingyMag.INTAKE_SENSOR_RIGHT_NAME);
        ball0ScannerLeft = new BaseColorSensor(opMode,BeethovenHardware.FlingyMag.INTAKE_SENSOR_LEFT_NAME);
        ball0ScannerMid = new BaseColorSensor(opMode,BeethovenHardware.FlingyMag.LAUNCHER_SENSOR_NAME);
    }
    public boolean Identify()
    {
        boolean leftYellow = ball0ScannerLeft.SeeArtifactColor(TargetColor.ARTIFACTGREEN);
        boolean leftRed = ball0ScannerLeft.SeeArtifactColor(TargetColor.RED);
        boolean leftBlue = ball0ScannerLeft.SeeArtifactColor(TargetColor.BLUE);
        boolean rightYellow = ball0ScannerRight.SeeArtifactColor(TargetColor.ARTIFACTGREEN);
        boolean rightRed = ball0ScannerRight.SeeArtifactColor(TargetColor.RED);
        boolean rightBlue = ball0ScannerRight.SeeArtifactColor(TargetColor.BLUE);
        boolean launcherYellow = ball0ScannerMid.SeeArtifactColor(TargetColor.ARTIFACTGREEN);
        boolean launcherRed = ball0ScannerMid.SeeArtifactColor(TargetColor.RED);
        boolean launcherBlue = ball0ScannerMid.SeeArtifactColor(TargetColor.BLUE);

        if (launcherYellow)
        {
            flingyReady = "yellow";
        }
        else if(launcherRed)
        {
            flingyReady = "red";
        }
        else if(launcherBlue)
        {
            flingyReady = "blue";
        }
        else
        {
            flingyReady = "none";
        }



        if (rightYellow || leftYellow)
        {
            flingyNext = "yellow";
        }
        else if(rightRed || leftRed)
        {
            flingyNext = "red";
        }
        else if(rightBlue || leftBlue)
        {
            flingyNext = "blue";
        }
        else
        {
            flingyNext = "none";
        }
        return true;
    }
}