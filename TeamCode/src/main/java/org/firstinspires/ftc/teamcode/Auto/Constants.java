package org.firstinspires.ftc.teamcode.Auto;

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
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.38968844676581);
        c.yPodOffset.set(1.6733252157376506);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.273921361802357);
                Controller secondaryTranslationalForward = Controller.proportional(0.10120662387238182);
                Controller primaryTranslationalLateral = Controller.proportional(0.4254571869687963);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15719506215955917);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011130860836533422));
                c.brake.set(Controller.proportionalFeedforward(0.009461231711053408));

                c.headingFeedback.set(Controller.proportional(4.134735700988736));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05505984159971028, 0.0038390793352799384));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1091092319200768, 0.058646076654587));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010293697040452178, 0.0017086590304443985));

                c.maxAchievableForwardVelocity.set(84.23036893636535);
                c.maxAchievableStrafeVelocity.set(70.49122550051248);
                c.naturalForwardDeceleration.set(26.817785577498935);
                c.naturalStrafeDeceleration.set(61.15262825855622);
            }
    );
}