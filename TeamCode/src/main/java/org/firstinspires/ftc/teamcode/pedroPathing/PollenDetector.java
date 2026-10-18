package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Test auto: looks for a ball with the Limelight, works out where it is on
 * the field, and drives one straight path to it (and 6 inches past it, so the
 * intake can grab it). It only starts one path, then just keeps reporting.
 *
 * The distance comes from simple trigonometry: if we know how high the camera
 * is, how high the ball is, and the angle down to the ball (ty), then
 * forward distance = height difference / tan(angle).
 */
@Autonomous(name = "PollenDetector")
public class PollenDetector extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private boolean pathStarted;

    private static final String LIMELIGHT_NAME = "limelight";

    private static final Pose START_POSE = new Pose(55.504667444574096, 7.834889148191378, Math.toRadians(90));

    // Tune these for your robot and Limelight mount.
    // Measure these on the real robot. Wrong numbers = wrong ball position.
    private static final double CAMERA_HEIGHT_IN = 8.5;
    private static final double BALL_HEIGHT_IN = 3;
    private static final double CAMERA_PITCH_DEG = 0;
    private static final double MIN_TARGET_AREA = 0.0;
    private static final double DRIVE_THROUGH_BALL_IN = 6.0;
    private static final double CAMERA_FORWARD_OFFSET_IN = 0.0;
    private static final double CAMERA_STRAFE_OFFSET_IN = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.start();

        pathStarted = false;
    }

    @Override
    public void loop() {
        follower.update();

        LLResult result = limelight.getLatestResult();
        TargetReading target = getBestTarget(result);

        if (!pathStarted && target != null) {
            Pose ballPose = estimateBallPose(target, follower.getPose());

            if (ballPose != null) {
                PathChain pathToBall = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), ballPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), ballPose.getHeading())
                        .build();

                follower.followPath(pathToBall, true);
                pathStarted = true;
            }
        }

        telemetry.addData("Limelight connected", limelight.isConnected());
        telemetry.addData("Limelight valid", result != null && result.isValid());
        telemetry.addData("Pipeline", result == null ? "none" : result.getPipelineType());
        telemetry.addData("Main tx/ty/ta", result == null ? "none" : result.getTx() + ", " + result.getTy() + ", " + result.getTa());
        telemetry.addData("Color targets", result == null ? 0 : result.getColorResults().size());
        telemetry.addData("Detector targets", result == null ? 0 : result.getDetectorResults().size());
        telemetry.addData("Using target", target == null ? "none" : target.source + " tx=" + target.txDeg + " ty=" + target.tyDeg + " area=" + target.area);
        telemetry.addData("Path started", pathStarted);
        telemetry.addData("Robot pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Picks the biggest target (biggest usually means closest) from the color
     * and neural-detector results. If neither has one, falls back to the
     * Limelight's main target. Returns null if nothing is seen.
     */
    private TargetReading getBestTarget(LLResult result) {
        if (result == null) {
            return null;
        }

        TargetReading bestTarget = null;

        for (LLResultTypes.ColorResult colorResult : result.getColorResults()) {
            TargetReading target = new TargetReading(
                    colorResult.getTargetXDegrees(),
                    colorResult.getTargetYDegrees(),
                    colorResult.getTargetArea(),
                    "color"
            );
            bestTarget = keepLargerTarget(bestTarget, target);
        }

        for (LLResultTypes.DetectorResult detectorResult : result.getDetectorResults()) {
            TargetReading target = new TargetReading(
                    detectorResult.getTargetXDegrees(),
                    detectorResult.getTargetYDegrees(),
                    detectorResult.getTargetArea(),
                    "detector"
            );
            bestTarget = keepLargerTarget(bestTarget, target);
        }

        if (bestTarget != null) {
            return bestTarget;
        }

        if (result.isValid() && result.getTa() >= MIN_TARGET_AREA) {
            return new TargetReading(result.getTx(), result.getTy(), result.getTa(), "main");
        }

        return null;
    }

    /** Returns whichever of the two targets is bigger, skipping ones below MIN_TARGET_AREA. */
    private TargetReading keepLargerTarget(TargetReading bestTarget, TargetReading target) {
        if (target.area < MIN_TARGET_AREA) {
            return bestTarget;
        }

        if (bestTarget == null || target.area > bestTarget.area) {
            return target;
        }

        return bestTarget;
    }

    /**
     * Turns the camera angles (tx, ty) into a field position for the robot to drive to.
     * Steps: angle down -> forward distance; angle sideways -> sideways distance;
     * add the camera offsets; push the point DRIVE_THROUGH_BALL_IN past the ball;
     * then rotate from robot directions into field x and y using the robot's heading.
     * Returns null if the numbers don't make sense (for example, the ball looks above the camera).
     */
    private Pose estimateBallPose(TargetReading target, Pose robotPose) {
        double txRad = Math.toRadians(target.txDeg);
        double totalPitchRad = Math.toRadians(CAMERA_PITCH_DEG + target.tyDeg);
        double tangent = Math.tan(-totalPitchRad);

        if (Math.abs(tangent) < 0.001) {
            return null;
        }

        double forwardInches = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / tangent;

        if (forwardInches <= 0) {
            return null;
        }

        double cameraStrafeInches = forwardInches * Math.tan(txRad);
        double robotForwardInches = forwardInches + CAMERA_FORWARD_OFFSET_IN;
        double robotStrafeInches = cameraStrafeInches + CAMERA_STRAFE_OFFSET_IN;
        double ballDistanceInches = Math.hypot(robotForwardInches, robotStrafeInches);

        if (ballDistanceInches < 0.001) {
            return null;
        }

        double targetDistanceInches = ballDistanceInches + DRIVE_THROUGH_BALL_IN;
        double targetForwardInches = robotForwardInches * targetDistanceInches / ballDistanceInches;
        double targetStrafeInches = robotStrafeInches * targetDistanceInches / ballDistanceInches;
        double robotHeading = robotPose.getHeading();
        double targetHeading = normalizeRadians(robotHeading + Math.atan2(robotStrafeInches, robotForwardInches));

        double fieldX = robotPose.getX()
                + targetForwardInches * Math.cos(robotHeading)
                - targetStrafeInches * Math.sin(robotHeading);
        double fieldY = robotPose.getY()
                + targetForwardInches * Math.sin(robotHeading)
                + targetStrafeInches * Math.cos(robotHeading);

        return new Pose(fieldX, fieldY, targetHeading);
    }

    /** Keeps an angle between -pi and +pi radians (-180 to +180 degrees). */
    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    /** One thing the camera saw: its angles, its size, and which Limelight result it came from. */
    private static class TargetReading {
        final double txDeg;
        final double tyDeg;
        final double area;
        final String source;

        TargetReading(double txDeg, double tyDeg, double area, String source) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
            this.source = source;
        }
    }
}