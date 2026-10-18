package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous from the start spot at (55, 8): shoot the preloads, then park.
 *
 * Uses a state machine like HiveUpAuto (pathState = the step we are on).
 *
 * NOTE: the name says "HiveDown" but it is in the "HiveUp" group and starts
 * at the HiveUp start spot (55, 8). Check which side this is for.
 */
@Autonomous(group = "HiveUp")
public class HiveDownParkAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Servo Headlight;   // NOTE: set up but not used in this auto
    private DcMotor Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;

    // ---------- Field positions (x inches, y inches, heading) ----------
    private final Pose startPose = new Pose(55, 8, Math.toRadians(270));   // where the robot starts
    private final Pose parkmid = new Pose(55, 20, Math.toRadians(270));    // control point toward park
    private final Pose park = new Pose(99, 36, Math.toRadians(270));       // park spot

    private PathChain park1;

    // Seconds since PLAY (reset again when the park drive starts).
    ElapsedTime waitimer = new ElapsedTime();

    /** Builds the path once in init(). */
    public void buildPaths() {
        park1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, parkmid, park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }

    /** The state machine: runs the current step, and moves on when it is done. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // SHOOT PRELOADS (timed)
                // Shooter on right away. 5 s: feed balls. 7 s: go park.
                Shooter.setPower(-0.62);
                if (waitimer.seconds() > 5) {
                    Intake.setPower(0.6);
                    Kicker.setPower(-0.6);
                }

                if (waitimer.seconds() > 7) {
                    setPathState(1);
                }
                break;
            case 1:
                // PARK: turn everything off and drive to the park spot.
                // After this, state 2 does nothing, so the robot waits there.
                Shooter.setPower(0);
                Kicker.setPower(0);
                Intake.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(park1, true);
                    waitimer.reset();
                    setPathState(2);
                }
                break;
        }
    }

    /** Moves the state machine to a new step. */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** Runs over and over after PLAY is pressed. */
    @Override
    public void loop() {

        // Pedro must update every loop to drive paths and track position.
        follower.update();
        autonomousPathUpdate();

        // Feedback on the Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer", waitimer.seconds());

        telemetry.update();
    }

    /** Runs once when INIT is pressed: find hardware and build the paths. */
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Headlight = hardwareMap.get(Servo.class, "Headlight");

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /** Runs over and over after INIT, while waiting for PLAY. */
    @Override
    public void init_loop() {}

    /** Runs once when PLAY is pressed: start the timers at step 0. */
    @Override
    public void start() {
        waitimer.reset();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** Nothing to do here: the SDK stops the motors when the OpMode ends. */
    @Override
    public void stop() {
    }
}
