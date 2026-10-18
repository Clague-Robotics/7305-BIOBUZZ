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
 * Autonomous from the start spot at (55, 8):
 *   1. Shoot the preloaded balls.
 *   2. Drive to the garden and intake slowly.
 *   3. Drive up the field and shoot again.
 *   4. Park.
 *
 * How it works: this is a "state machine". pathState is the step we are on.
 * loop() runs many times a second, and each time it checks the current step.
 * When a step is finished, it calls setPathState() to move to the next one.
 *
 * Poses are (x inches, y inches, heading). In a BezierCurve(a, b, c) the
 * middle pose b is a control point: the path bends toward it but does not
 * pass through it.
 */
@Autonomous(group = "HiveUp")
public class HiveUpAuto extends OpMode {

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

    // ---------- Field positions ----------
    private final Pose startPose = new Pose(55, 8, Math.toRadians(270));    // where the robot starts
    private final Pose gardenmid = new Pose(55, 30, Math.toRadians(180));   // control point toward the garden
    private final Pose gardenm = new Pose(45, 3, Math.toRadians(180));      // lined up to intake the garden
    private final Pose garden = new Pose(17, 1, Math.toRadians(185));       // end of the intake drive
    private final Pose shootm = new Pose(43, 20, Math.toRadians(90));       // control point leaving the garden
    private final Pose shootmid = new Pose(43, 80, Math.toRadians(90));     // halfway to the shooting spot
    private final Pose shoot1 = new Pose(63, 80, Math.toRadians(90));       // control point toward the shooting spot
    private final Pose shoot1b = new Pose(65, 103, Math.toRadians(90));     // shooting spot
    private final Pose parkmid = new Pose(63, 80, Math.toRadians(90));      // control point toward park
    private final Pose park = new Pose(30, 80, Math.toRadians(90));         // first park point
    private final Pose parkb = new Pose(15, 95, Math.toRadians(90));        // final park spot

    private PathChain midpickgarden, pickgarden, midshoot, firstshoot, park1, park2;

    // Seconds since the current shooting step started.
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

    /** Builds every path once in init(), so nothing has to be calculated during the match. */
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

        // Garden -> halfway up the field
        midshoot = follower.pathBuilder()
                .addPath(new BezierCurve(garden, shootm, shootmid))
                .setLinearHeadingInterpolation(garden.getHeading(), shootmid.getHeading())
                .build();

        // Halfway -> shooting spot
        firstshoot = follower.pathBuilder()
                .addPath(new BezierCurve(shootmid, shoot1, shoot1b))
                .setLinearHeadingInterpolation(shootmid.getHeading(), shoot1.getHeading())
                .build();

        // Shooting spot -> park, in two parts
        park1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1b, parkmid, park))
                .setLinearHeadingInterpolation(shoot1b.getHeading(), park.getHeading())
                .build();
        park2 = follower.pathBuilder()
                .addPath(new BezierLine(park, parkb))
                .setLinearHeadingInterpolation(park.getHeading(), parkb.getHeading())
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
                // Keep intaking and spin up the shooter. When the garden
                // drive ends, head up the field at full speed.
                Intake.setPower(1);
                setCompensatedShooterVelocity(-1485);

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(midshoot, true);
                    setPathState(4);
                }

                break;

            case 4:
                // Halfway there: start the drive to the shooting spot.
                Intake.setPower(0);
                follower.setMaxPower(1);

                if (!follower.isBusy()) {
                    follower.followPath(firstshoot, true);
                    waitimer.reset();
                    setPathState(6);   // there is no step 5
                }

                break;

            case 6:
                // SHOOT AGAIN (timed, 7 seconds)
                // NOTE: the timer starts when the drive to the shooting spot
                // STARTS, not when the robot arrives, so balls are fed at 2 s
                // even if the robot is still moving.
                follower.setMaxPower(1);

                if (waitimer.seconds() > 0) {
                    setCompensatedShooterVelocity(-1485);
                }

                if (waitimer.seconds() > 2) {
                    Intake.setPower(0.6);
                    Kicker.setPower(-0.25);
                }

                if (waitimer.seconds() > 4) {
                    setCompensatedShooterVelocity(-1420);
                }

                if (waitimer.seconds() > 7) {
                    Shooter.setPower(0);
                    Intake.setPower(0);
                    Kicker.setPower(0);
                    setPathState(7);
                }

                break;

            case 7:
                // PARK, part 1
                Shooter.setPower(0);
                Kicker.setPower(0);
                Intake.setPower(0);

                if (!follower.isBusy()) {
                    follower.followPath(park1, true);
                    waitimer.reset();
                    setPathState(8);
                }

                break;

            case 8:
                // PARK, part 2. After this, state 9 does nothing, so the robot waits.
                if (!follower.isBusy()) {
                    follower.followPath(park2, true);
                    waitimer.reset();
                    setPathState(9);
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
        // Pedro must update every loop to drive the path and track position.
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
