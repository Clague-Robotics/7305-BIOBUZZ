package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.currentThreadTimeMillis;
import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Thread;

@Autonomous(group = "HiveUp")
public class HiveUpAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Servo Headlight;
    private DcMotorEx Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;

    private VoltageSensor voltageSensor;

    private static final double PERFECT_VOLTAGE = 13.25;
    private static final double VOLTAGE_COMP_STRENGTH = 0.10;

    private final Pose startPose = new Pose(55, 8, Math.toRadians(270));
    private final Pose gardenmid = new Pose(55, 30, Math.toRadians(180));
    private final Pose gardenm = new Pose(45, 3, Math.toRadians(180));
    private final Pose garden = new Pose(17, 1, Math.toRadians(185));
    private final Pose shootm = new Pose(43, 20, Math.toRadians(90));
    private final Pose shootmid = new Pose(43, 80, Math.toRadians(90));
    private final Pose shoot1 = new Pose(63, 80, Math.toRadians(90));
    private final Pose shoot1b = new Pose(65, 103, Math.toRadians(90));
    private final Pose parkmid = new Pose(63, 80, Math.toRadians(90));
    private final Pose park = new Pose(30, 80, Math.toRadians(90));
    private final Pose parkb = new Pose(15, 95, Math.toRadians(90));

    private PathChain midpickgarden, pickgarden, midshoot, firstshoot, park1,park2;

    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime waitimer = new ElapsedTime();

    private void setCompensatedShooterVelocity(double baseVelocity) {
        double currentVoltage = voltageSensor.getVoltage();

        double fullCorrection = PERFECT_VOLTAGE / currentVoltage;

        double softenedCorrection =
                1 + (fullCorrection - 1) * VOLTAGE_COMP_STRENGTH;

        Shooter.setVelocity(baseVelocity * softenedCorrection);
    }

    public void buildPaths() {
        midpickgarden = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, gardenmid, gardenm))
                .setLinearHeadingInterpolation(startPose.getHeading(), gardenmid.getHeading())
                .build();

        pickgarden = follower.pathBuilder()
                .addPath(new BezierLine(gardenm, garden))
                .setLinearHeadingInterpolation(gardenm.getHeading(), garden.getHeading())
                .build();

        midshoot = follower.pathBuilder()
                .addPath(new BezierCurve(garden, shootm, shootmid))
                .setLinearHeadingInterpolation(garden.getHeading(), shootmid.getHeading())
                .build();

        firstshoot = follower.pathBuilder()
                .addPath(new BezierCurve(shootmid, shoot1, shoot1b))
                .setLinearHeadingInterpolation(shootmid.getHeading(), shoot1.getHeading())
                .build();

        park1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1b, parkmid, park))
                .setLinearHeadingInterpolation(shoot1b.getHeading(), park.getHeading())
                .build();
        park2 = follower.pathBuilder()
                .addPath(new BezierLine(park, parkb))
                .setLinearHeadingInterpolation(park.getHeading(), parkb.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
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
                Intake.setPower(0);
                follower.setMaxPower(1);

                if (!follower.isBusy()) {
                    follower.followPath(midpickgarden, true);
                    setPathState(2);
                }

                break;

            case 2:
                Intake.setPower(1);

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.25);
                    follower.followPath(pickgarden, true);
                    setPathState(3);
                }

                break;

            case 3:
                Intake.setPower(1);
                setCompensatedShooterVelocity(-1485);

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(midshoot, true);
                    setPathState(4);
                }

                break;

            case 4:
                Intake.setPower(0);
                follower.setMaxPower(1);

                if (!follower.isBusy()) {
                    follower.followPath(firstshoot, true);
                    waitimer.reset();
                    setPathState(6);
                }

                break;

            case 6:
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

                if (!follower.isBusy()) {
                    follower.followPath(park2, true);
                    waitimer.reset();
                    setPathState(9);
                }

                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer", waitimer.seconds());
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Shooter Velocity", Shooter.getVelocity());

        telemetry.update();
    }

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