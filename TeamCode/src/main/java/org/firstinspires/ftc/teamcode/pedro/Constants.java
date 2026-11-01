package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FrontLeftWheel");
                c.frontRightName.set("FrontRightWheel");
                c.backLeftName.set("BackLeftWheel");
                c.backRightName.set("BackRightWheel");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("Pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(6.96850394);
                c.yPodOffset.set(0.03937008);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                /*
                // End Constraints, with default values. Only enable if follower is taking too long to finish paths.
                c.parametricTConstraint.set(0.025);
                c.velocityConstraint.set(0.1);
                c.translationalConstraint.set(0.1);
                c.headingConstraint.set(0.007);
                c.timeoutConstraint.set(100.0);
                // Path Constraints, with default values. Adding default values is harmless, but adding finite acceleration/velocity/deceleration constraints is an actual behavior change.
                c.brakeAggression.set(1.0);
                c.maxBrakingPower.set(0.2);
                c.maxAccelerationConstraint.set(ForesightConfig.Constraint.NONE);
                c.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
                c.maxDecelerationConstraint.set(ForesightConfig.Constraint.NONE);
                c.maxDecelerationScale.set(ForesightConfig.Constraint.NONE);
                c.coastDownToVelocity.set(0.0);
                // Foresight Constraints, with default values.
                c.headingDriveRatio.set(0.5);
                c.brakeAtEnd.set(true);
                c.translationalDeviationTolerance.set(2.5);
                c.headingDeviationTolerance.set(Math.toRadians(11.25));
                c.cosineScale.set(false);
                */
                Controller primaryTranslationalForward = Controller.proportional(0.31373366008474174);
                Controller secondaryTranslationalForward = Controller.proportional(0.11591620428352051);
                Controller primaryTranslationalLateral = Controller.proportional(0.49702205863857224);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18363636999295196);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018219738653513333));
                c.brake.set(Controller.proportionalFeedforward(0.015486777855486333));

                c.headingFeedback.set(Controller.proportional(4.104448001823589));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04179017793937523, 0.007249258120814468));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09218355660085838, 0.06063525224611366));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013068092557740353, 0.0016906794804317248));

                c.maxAchievableForwardVelocity.set(58.97072313325149);
                c.maxAchievableStrafeVelocity.set(45.15864360899799);
                c.naturalForwardDeceleration.set(35.51390692087908);
                c.naturalStrafeDeceleration.set(73.39581398800358);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
