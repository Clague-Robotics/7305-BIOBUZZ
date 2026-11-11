package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
 * AUTO plan when our alliance partner is team 10136.
 *   10136 makes TIP 1, we make TIP 2, 10136 tries for TIP 3, then everyone parks.
 *
 *   1. Start on the left (down) side. Spin up the shooter.
 *   2. Wait until the HIVE tips (from 10136) OR 7 seconds, whichever comes first.
 *   3. Shoot the 4 preloads for up to 5 s, feeding ONLY while the shooter is at 2350 RPM or more.
 *   4. Drive to the FLOWER and intake its 4 POLLEN.
 *   5. Drive back and shoot them (same RPM rule) to cause TIP 2.
 *   6. Park on the wall side.
 *   Safety: at 25 s we stop whatever we are doing and go park, so we never miss PARK points.
 *
 * Tip detection: each CELL has AprilTags on its bottom face (manual section 9.9).
 *   Red CELL, far side from the audience: IDs 0-3.  Red CELL, audience side: IDs 4-7.
 *   During INIT we remember which red CELL's tags the Limelight sees (that one is face-up).
 *   When the OTHER red CELL's tags show up for a few loops in a row, the HIVE has tipped.
 *   If the camera saw nothing during INIT, we just use the 7 s timer.
 *
 * Every Pose marked TUNE is a starting guess. Measure them on a real field.
 */
@Autonomous(name = "Down Left - 10136 partner (TIP 2)", group = "HiveDown")
public class HiveDownPartnerTipAuto extends OpMode {

    // ---------- Numbers to tune ----------
    private static final double READY_RPM = 2350.0;      // only feed balls at or above this
    private static final double COMMAND_RPM = 2400.0;    // ask a bit higher so we actually reach READY_RPM
    private static final double TICKS_PER_REV = 28.0;    // shooter motor encoder (same as the other autos)

    private static final double MAX_WAIT_FOR_TIP = 7.0;  // s, shoot preloads after this even with no tip
    private static final double SHOOT_TIME = 5.0;        // s, length of each shooting window
    private static final double INTAKE_TIME = 1.5;       // s, sit at the FLOWER with the intake on
    private static final double PARK_BY = 25.0;          // s after start, drop everything and park
    private static final int TIP_CONFIRM_LOOPS = 5;      // loops in a row the new CELL must be seen

    private static final double INTAKE_IN = 1.0;
    private static final double KICKER_FEED = -0.40;     // pushes balls into the shooter
    private static final double KICKER_HOLD = 0.1;       // holds balls back while intaking

    // ---------- Field positions (inches, Pedro coordinates) ----------
    private final Pose startPose = new Pose(55, 120, Math.toRadians(90));   // same start as HiveSideDownAuto
    private final Pose shootPose = new Pose(55, 120, Math.toRadians(90));   // shoot where we start. TUNE
    private final Pose flowerMid = new Pose(55, 100, Math.toRadians(270));  // TUNE
    private final Pose flowerPose = new Pose(40, 110, Math.toRadians(270)); // TUNE: intake at the FLOWER
    private final Pose parkPose = new Pose(15, 110, Math.toRadians(90));    // TUNE: wall-side park

    // ---------- Red CELL AprilTag IDs (manual 9.9) ----------
    private static final int FAR_CELL = 1, AUDIENCE_CELL = 2, NO_CELL = 0;

    private Follower follower;
    private Timer stateTimer, opmodeTimer;
    private int state;

    private DcMotorEx shooter;
    private DcMotor kicker, intake;
    private Limelight3A limelight;

    private PathChain toFlower, toShoot, toPark;

    private int upCellAtStart = NO_CELL;
    private int tipSeenLoops = 0;
    private boolean tipSeen = false;

    // ---------- States ----------
    private static final int WAIT_FOR_TIP = 0;
    private static final int SHOOT_PRELOADS = 1;
    private static final int DRIVE_TO_FLOWER = 2;
    private static final int INTAKE_FLOWER = 3;
    private static final int DRIVE_TO_SHOOT = 4;
    private static final int SHOOT_FLOWER = 5;
    private static final int PARKING = 6;
    private static final int DONE = 7;

    private void buildPaths() {
        toFlower = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, flowerMid, flowerPose))
                .setTangentHeadingInterpolation()   // face where we drive, intake first
                .build();

        toShoot = follower.pathBuilder()
                .addPath(new BezierCurve(flowerPose, flowerMid, shootPose))
                .setLinearHeadingInterpolation(flowerPose.getHeading(), shootPose.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private double shooterRPM() {
        return Math.abs(shooter.getVelocity()) * 60.0 / TICKS_PER_REV;
    }

    /** Feed balls only while the shooter is fast enough. */
    private void shootIfReady() {
        if (shooterRPM() >= READY_RPM) {
            intake.setPower(INTAKE_IN);
            kicker.setPower(KICKER_FEED);
        } else {
            intake.setPower(0);
            kicker.setPower(0);
        }
    }

    private void setState(int newState) {
        state = newState;
        stateTimer.resetTimer();
    }

    private void autonomousUpdate() {
        double t = stateTimer.getElapsedTimeSeconds();

        // Safety: always leave time to park.
        if (opmodeTimer.getElapsedTimeSeconds() > PARK_BY && state < PARKING) {
            intake.setPower(0);
            kicker.setPower(0);
            follower.setMaxPower(1.0);
            follower.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), parkPose))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
                    .build(), true);
            setState(PARKING);
            return;
        }

        switch (state) {
            case WAIT_FOR_TIP:
                // Shooter is already spinning up (see start()).
                if (tipSeen || t > MAX_WAIT_FOR_TIP) {
                    setState(SHOOT_PRELOADS);
                }
                break;

            case SHOOT_PRELOADS:
                shootIfReady();
                if (t > SHOOT_TIME) {
                    intake.setPower(INTAKE_IN);
                    kicker.setPower(KICKER_HOLD);
                    follower.setMaxPower(1.0);   // "quickly" to the FLOWER
                    follower.followPath(toFlower, true);
                    setState(DRIVE_TO_FLOWER);
                }
                break;

            case DRIVE_TO_FLOWER:
                if (!follower.isBusy()) {
                    setState(INTAKE_FLOWER);
                }
                break;

            case INTAKE_FLOWER:
                if (t > INTAKE_TIME) {
                    follower.followPath(toShoot, true);
                    setState(DRIVE_TO_SHOOT);
                }
                break;

            case DRIVE_TO_SHOOT:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    kicker.setPower(0);
                    setState(SHOOT_FLOWER);
                }
                break;

            case SHOOT_FLOWER:
                shootIfReady();
                if (t > SHOOT_TIME) {
                    intake.setPower(0);
                    kicker.setPower(0);
                    follower.followPath(toPark, true);
                    setState(PARKING);
                }
                break;

            case PARKING:
                if (!follower.isBusy()) {
                    shooter.setPower(0);
                    setState(DONE);
                }
                break;

            case DONE:
            default:
                break;
        }
    }

    /** Which red CELL's tags the camera sees right now (FAR_CELL, AUDIENCE_CELL or NO_CELL). */
    private int visibleRedCell() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return NO_CELL;
        int far = 0, audience = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id >= 0 && id <= 3) far++;
            else if (id >= 4 && id <= 7) audience++;
        }
        if (far == 0 && audience == 0) return NO_CELL;
        return far >= audience ? FAR_CELL : AUDIENCE_CELL;
    }

    private void checkForTip() {
        if (tipSeen || upCellAtStart == NO_CELL) return;
        int now = visibleRedCell();
        if (now != NO_CELL && now != upCellAtStart) {
            tipSeenLoops++;
            if (tipSeenLoops >= TIP_CONFIRM_LOOPS) tipSeen = true;
        } else {
            tipSeenLoops = 0;
        }
    }

    @Override
    public void init() {
        stateTimer = new Timer();
        opmodeTimer = new Timer();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);   // same AprilTag pipeline as HiveSideDownAuto
        limelight.start();

        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(13.0, 0.5, 0.0, 12.0);
        kicker = hardwareMap.get(DcMotor.class, "Kicker");
        intake = hardwareMap.get(DcMotor.class, "Intake");

        shooter.setPower(0);
        kicker.setPower(0);
        intake.setPower(0);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        // Keep updating which CELL is face-up until the match starts.
        int seen = visibleRedCell();
        if (seen != NO_CELL) upCellAtStart = seen;
        telemetry.addData("Face-up red CELL", upCellAtStart == FAR_CELL ? "far (0-3)"
                : upCellAtStart == AUDIENCE_CELL ? "audience (4-7)" : "not seen, will use 7 s timer");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shooter.setVelocity(-COMMAND_RPM * TICKS_PER_REV / 60.0);  // negative = shooting direction
        setState(WAIT_FOR_TIP);
    }

    @Override
    public void loop() {
        follower.update();
        checkForTip();
        autonomousUpdate();

        telemetry.addData("State", state);
        telemetry.addData("Match time", "%.1f", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Tip seen", tipSeen);
        telemetry.addData("Shooter RPM", "%.0f (ready at %.0f)", shooterRPM(), READY_RPM);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        shooter.setPower(0);
        intake.setPower(0);
        kicker.setPower(0);
    }
}
