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

/**
 * Robot settings for Pedro Pathing, used by every Pedro OpMode through
 * Constants.createFollower(hardwareMap).
 *
 * Most numbers here come from Pedro's tuning OpModes (Tuning.java). If the
 * robot changes (weight, wheels, odometry pods), run the tuners again and
 * update these. Units are inches unless noted.
 */
public class Constants {

    // Odometry: the goBILDA Pinpoint computer and its two tracking wheels ("pods").
    // forwardPodY / strafePodX = where each pod sits, measured from the robot's center.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2)
            .strafePodX(8.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // Drive motors. Names must match the Robot Configuration on the Driver Hub.
    // xVelocity / yVelocity = top speed forward and sideways (inches/second), from the velocity tuners.
    // NOTE: TeleOpDrive.java reverses the RIGHT side instead of the left.
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

    // How Pedro follows a path.
    //   mass                        robot mass in kilograms
    //   ...ZeroPowerAcceleration    how fast the robot slows when coasting (from the tuners)
    //   heading PIDF                keeps the robot pointed the right way
    //   translational PIDF          keeps the robot on the path sideways
    //   drive PIDF                  controls speed along the path
    //   "secondary" PIDFs           gentler versions used when the robot is close to the target
    // PIDF numbers are (P, I, D, F); the drive ones have one more filter value.
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

    // When a path counts as finished: (fraction of the path done, velocity,
    // translational error, heading error). See the Pedro Pathing docs.
    public static PathConstraints pathConstraints = new PathConstraints
            (0.99, 100, 1.4, 1);

    /** Builds the Follower that drives the robot along paths. Call once in init(). */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
