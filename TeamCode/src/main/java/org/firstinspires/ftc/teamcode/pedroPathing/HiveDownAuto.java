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
 * Autonomous from the start spot at (86, 8): wait, then shoot the preloads.
 *
 * Uses a state machine like HiveUpAuto (pathState = the step we are on).
 * This one uses plain motor power for the shooter, not voltage-compensated
 * velocity.
 *
 * NOTE: a park path is built, but no step drives it. After shooting, the
 * state goes to 1, and there is no case 1, so the robot stays where it is.
 */
@Autonomous(group = "HiveDown")
public class HiveDownAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Servo Headlight;   // NOTE: set up but not used in this auto
    private DcMotor Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;

    // ---------- Field positions (x inches, y inches, heading) ----------
    private final Pose startPose = new Pose(86, 8, Math.toRadians(270));   // where the robot starts
    private final Pose ParkMid = new Pose(86, 20, Math.toRadians(270));    // control point toward park
    private final Pose Park = new Pose(100, 34, Math.toRadians(270));      // park spot

    // Start -> park (named midpickgarden, but it is the park path)
    private PathChain midpickgarden;

    // Seconds since PLAY was pressed.
    ElapsedTime waitimer = new ElapsedTime();

    /** Builds the path once in init(). */
    public void buildPaths() {
        midpickgarden = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, ParkMid, Park))
                .setLinearHeadingInterpolation(startPose.getHeading(), Park.getHeading())
                .build();
    }

    /** The state machine: runs the current step, and moves on when it is done. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // SHOOT PRELOADS (timed)
                // 4 s: spin up the shooter. 7 s: feed balls. 9 s: stop everything.
                if (waitimer.seconds() > 4) {
                    Shooter.setPower(-0.62);
                }
                if (waitimer.seconds() > 7) {
                    Intake.setPower(0.6);
                    Kicker.setPower(-0.6);
                }

                if (waitimer.seconds() > 9) {
                    Shooter.setPower(0);
                    Intake.setPower(0);
                    Kicker.setPower(0);
                    setPathState(1);
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
