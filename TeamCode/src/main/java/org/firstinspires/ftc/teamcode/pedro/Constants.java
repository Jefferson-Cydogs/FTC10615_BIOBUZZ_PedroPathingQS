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
                Controller primaryTranslationalForward = Controller.proportional(0.2863788296740714);
                Controller secondaryTranslationalForward = Controller.proportional(0.10580932538130823);
                Controller primaryTranslationalLateral = Controller.proportional(0.3927447731466221);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1451086993440615);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01778612078682873));
                c.brake.set(Controller.proportionalFeedforward(0.01511820266880442));

                c.headingFeedback.set(Controller.proportional(3.8181134783716195));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05012834316430095, 0.004365041651603953));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05326763195534439, 0.06142057682558323));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0023390176212197073, 0.0015321202654240906));

                c.maxAchievableForwardVelocity.set(61.14944908942139);
                c.maxAchievableStrafeVelocity.set(47.591469947674426);
                c.naturalForwardDeceleration.set(36.27397948903635);
                c.naturalStrafeDeceleration.set(68.74023828758513);
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
