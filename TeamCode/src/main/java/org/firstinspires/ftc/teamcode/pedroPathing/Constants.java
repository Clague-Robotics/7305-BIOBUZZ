package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2)
            .strafePodX(8.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(68)
            .yVelocity(62)
            .useBrakeModeInTeleOp(true);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration(-42.615171495439945)
            .lateralZeroPowerAcceleration(-48.016596200674314)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9,0,0.04,0.1))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5,0,0.03,0.025))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0009,0,1,0.021))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.005,0,1,0.015))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016)) // (kP, kLinear, kQuadratic)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.017, 0, 0.00005, 0.6, 0.01))
            .centripetalScaling(0)
            ;

    public static PathConstraints pathConstraints = new PathConstraints
            (0.99, 100, 1.4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
