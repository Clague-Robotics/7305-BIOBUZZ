package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shorter version of HiveUpAuto, from the start spot at (55, 8):
 *   1. Shoot the preloaded balls.
 *   2. Drive to the garden and intake slowly.
 *   3. Drive to parkb at (15, 69) and stop.
 *
 * Uses a state machine like HiveUpAuto (pathState = the step we are on).
 */
@Autonomous(group = "HiveUp")
public class SalineMatch1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Servo Headlight;   // NOTE: set up but not used in this auto
    private DcMotorEx Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;

    private VoltageSensor voltageSensor;

    // Battery voltage compensation (see setCompensatedShooterVelocity).
    private static final double PERFECT_VOLTAGE = 13.25;
    private static final double VOLTAGE_COMP_STRENGTH = 0.10;

    // ---------- Field positions (x inches, y inches, heading) ----------
    // Same as HiveUpAuto, except parkb. Only startPose through shootm and
    // parkb are used here; the rest are left from HiveUpAuto.
    private final Pose startPose = new Pose(55, 8, Math.toRadians(270));    // where the robot starts
    private final Pose gardenmid = new Pose(55, 30, Math.toRadians(180));   // control point toward the garden
    private final Pose gardenm = new Pose(45, 3, Math.toRadians(180));      // lined up to intake the garden
    private final Pose garden = new Pose(17, 1, Math.toRadians(185));       // end of the intake drive
    private final Pose shootm = new Pose(43, 20, Math.toRadians(90));       // control point leaving the garden
    private final Pose shootmid = new Pose(43, 80, Math.toRadians(90));
    private final Pose shoot1 = new Pose(63, 80, Math.toRadians(90));
    private final Pose shoot1b = new Pose(65, 103, Math.toRadians(90));
    private final Pose parkmid = new Pose(63, 80, Math.toRadians(90));
    private final Pose park = new Pose(30, 80, Math.toRadians(90));
    private final Pose parkb = new Pose(15, 69, Math.toRadians(90));        // final spot

    private PathChain midpickgarden, pickgarden, midshoot;

    // Seconds since PLAY was pressed.
    ElapsedTime waitimer = new ElapsedTime();

    /**
     * Sets the shooter speed, nudged up a little when the battery is low.
     * A weaker battery spins the wheel slower, so we ask for a bit more.
     */
    private void setCompensatedShooterVelocity(double baseVelocity) {
        double currentVoltage = voltageSensor.getVoltage();

        double fullCorrection = PERFECT_VOLTAGE / currentVoltage;

        double softenedCorrection = 1 + (fullCorrection - 1) * VOLTAGE_COMP_STRENGTH;

        Shooter.setVelocity(baseVelocity * softenedCorrection);
    }

    /** Builds every path once in init(). */
    public void buildPaths() {
        // Start -> lined up at the garden
        midpickgarden = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, gardenmid, gardenm))
                .setLinearHeadingInterpolation(startPose.getHeading(), gardenmid.getHeading())
                .build();

        // Drive along the garden while intaking
        pickgarden = follower.pathBuilder()
                .addPath(new BezierLine(gardenm, garden))
                .setLinearHeadingInterpolation(gardenm.getHeading(), garden.getHeading())
                .build();

        // Garden -> final spot (named midshoot, but it goes to parkb)
        midshoot = follower.pathBuilder()
                .addPath(new BezierCurve(garden, shootm, parkb))
                .setLinearHeadingInterpolation(garden.getHeading(), parkb.getHeading())
                .build();
    }

    /** The state machine: runs the current step, and moves on when it is done. */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // SHOOT PRELOADS (timed, 7 seconds)
                // 0 s: spin up the shooter. 2 s: feed balls. 3 s: slow the
                // shooter a little. 7 s: stop everything and go on.
                follower.setMaxPower(1);

                if (waitimer.seconds() > 0) {
                    setCompensatedShooterVelocity(-1485);
                }

                if (waitimer.seconds() > 2) {
                    Intake.setPower(0.6);
                    Kicker.setPower(-0.2);
                }

                if (waitimer.seconds() > 3) {
                    setCompensatedShooterVelocity(-1410);
                }

                if (waitimer.seconds() > 7) {
                    Shooter.setPower(0);
                    Intake.setPower(0);
                    Kicker.setPower(0);
                    setPathState(1);
                }

                break;

            case 1:
                // DRIVE TO THE GARDEN
                Intake.setPower(0);
                follower.setMaxPower(1);

                if (!follower.isBusy()) {
                    follower.followPath(midpickgarden, true);
                    setPathState(2);
                }

                break;

            case 2:
                // When we reach the garden, drive through it slowly (25%) while intaking.
                Intake.setPower(1);

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.25);
                    follower.followPath(pickgarden, true);
                    setPathState(3);
                }

                break;

            case 3:
                // Keep intaking with the shooter stopped. When the garden
                // drive ends, drive to the final spot. State 4 does nothing,
                // so the robot waits there.
                Intake.setPower(1);
                setCompensatedShooterVelocity(0);

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(midshoot, true);
                    setPathState(4);
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
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Shooter Velocity", Shooter.getVelocity());

        telemetry.update();
    }

    /** Runs once when INIT is pressed: find hardware and build the paths. */
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Headlight = hardwareMap.get(Servo.class, "Headlight");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
    }

    /** Runs once when PLAY is pressed: start the timers at step 0. */
    @Override
    public void start() {
        waitimer.reset();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
