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
                Controller primaryTranslationalForward = Controller.proportional(0.4012848190517527);
                Controller secondaryTranslationalForward = Controller.proportional(0.1482640181117779);
                Controller primaryTranslationalLateral = Controller.proportional(0.6351855852654);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23468410128414055);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017768828399754175));
                c.brake.set(Controller.proportionalFeedforward(0.015103504139791049));

                c.headingFeedback.set(Controller.proportional(5.796826607594298));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05296036786713781, 0.0034791883614576865));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07215007849818011, 0.05220257715454756));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001771759053035244, 0.0018102535420155557));

                c.maxAchievableForwardVelocity.set(59.51029864484011);
                c.maxAchievableStrafeVelocity.set(46.84134068859984);
                c.naturalForwardDeceleration.set(38.15444485175657);
                c.naturalStrafeDeceleration.set(64.58905721038217);
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
