
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "HiveUp")
public class HiveUpSideAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Servo Headlight;
    private DcMotorEx Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;

    private VoltageSensor voltageSensor;
    private Limelight3A limelight;

    private boolean on = false;

    private double targetRPM = 2350.0;
    private double targetRPMShoot2 = 2000.0;

    private double targetVelocity = -targetRPM * 28.0 / 60.0;
    private double targetVelocityShoot2 = -targetRPMShoot2 * 28.0 / 60.0;


    private static final double PERFECT_VOLTAGE = 13.25;
    private static final double VOLTAGE_COMP_STRENGTH = 0.10;

    private final Pose startPose = new Pose(55, 8, Math.toRadians(270));
    private final Pose gardenPickUp = new Pose(23, 1, Math.toRadians(180));
    private final Pose shootPoseMid = new Pose(35, 30, Math.toRadians(180));

    private final Pose shootPoseMid2 = new Pose(35, 80, Math.toRadians(90));
    private final Pose shootPose = new Pose(55, 95, Math.toRadians(120));

    private final Pose flowerPickUp = new Pose(55, 92, Math.toRadians(90));
    private final Pose flowerPickUpF = new Pose(55, 100, Math.toRadians(90));

    private final Pose flowerPickUpMid = new Pose(45, 70, Math.toRadians(90));

    private PathChain PickUpgarden, shoot2, shoot3, FlowerPickUp, FlowerPickUpF;

    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime waitimer = new ElapsedTime();


    public void buildPaths() {
        PickUpgarden = follower.pathBuilder()
                .addPath(new BezierLine(startPose, gardenPickUp))
                .setTangentHeadingInterpolation()
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(gardenPickUp, shootPoseMid))
                .setLinearHeadingInterpolation(
                        gardenPickUp.getHeading(),
                        shootPoseMid.getHeading()
                )
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseMid,shootPoseMid2, shootPose))
                .setLinearHeadingInterpolation(
                        shootPoseMid.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        FlowerPickUp = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, flowerPickUpMid, flowerPickUp))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        flowerPickUp.getHeading()
                )
                .build();
        FlowerPickUpF = follower.pathBuilder()
                .addPath(new BezierLine(flowerPickUp, flowerPickUpF))
                .setLinearHeadingInterpolation(
                        flowerPickUp.getHeading(),
                        flowerPickUpF.getHeading()
                )
                .build();
    }

    public void autonomousPathUpdate() {

        double actualRPM =
                Math.abs(Shooter.getVelocity()) * 60.0 / 28.0;

        switch (pathState) {

            case 0:

                if (actualRPM >= targetRPM) {
                    Intake.setPower(1.0);
                    Kicker.setPower(-0.40);
                } else {
                    Intake.setPower(0);
                    Kicker.setPower(0);
                }

                if (waitimer.seconds() > 5) {
                    setPathState(1);
                }

                break;

            case 1:

                Intake.setPower(1);
                Kicker.setPower(0.1);
                follower.setMaxPower(0.7);

                if (!follower.isBusy()) {
                    follower.followPath(PickUpgarden, true);
                    waitimer.reset();
                    setPathState(2);
                }

                break;

            case 2:
                Intake.setPower(1);
                Kicker.setPower(0.1);
                follower.setMaxPower(0.7);
                    if (!follower.isBusy()) {
                        follower.followPath(shoot2, true);
                        if(waitimer.seconds() >= 2){
                        waitimer.reset();
                        setPathState(3);}
                }
                    break;
            case 3:

                follower.setMaxPower(0.8);
                if (!follower.isBusy()) {
                    follower.followPath(shoot3, true);
                    if(waitimer.seconds() >= 3){
                        waitimer.reset();
                        setPathState(4);}
                }
                break;
            case 4:
                if (actualRPM >= targetRPMShoot2) {
                    Intake.setPower(1.0);
                    Kicker.setPower(-0.40);
                } else {
                    Intake.setPower(0);
                    Kicker.setPower(0);
                }

                if (waitimer.seconds() >= 5) {
                    setPathState(5);
                }

                break;
            case 5:
                Intake.setPower(1);
                Kicker.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(FlowerPickUp, true);
                    setPathState(6);
                }
                break;
            case 6:
                Intake.setPower(1);
                Kicker.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(FlowerPickUpF, true);
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

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Ty", llResult.getTy());
        }

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer", waitimer.seconds());
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Shooter Velocity", Shooter.getVelocity());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData(
                "Actual RPM",
                Math.abs(Shooter.getVelocity()) * 60.0 / 28.0
        );

        telemetry.update();
    }

    @Override
    public void init() {

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Headlight = hardwareMap.get(Servo.class, "Headlight");

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setVelocityPIDFCoefficients(13.0, 0.5, 0.0, 12.0);

        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Shooter.setPower(0);
        Kicker.setPower(0);
        Intake.setPower(0);

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

        Shooter.setVelocity(targetVelocity);

        setPathState(0);
    }

    @Override
    public void stop() {

        Shooter.setPower(0);
        Intake.setPower(0);
        Kicker.setPower(0);
    }
}
