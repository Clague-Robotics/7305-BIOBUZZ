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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Thread;

@Autonomous
public class DecodeAutoBlue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Servo Headlight;
    private DcMotor Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;
    private final Pose startPose = new Pose(55.504667444574096, 7.834889148191378, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose ShootPose = new Pose(60, 95, Math.toRadians(138)); // Start Pose of our robot.;
    private final Pose pickup = new Pose(30, 96, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose midpickup2 = new Pose(78, 70, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose pickuptwo = new Pose(26, 70, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose midpickup3 = new Pose(78, 50, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose pickupthree = new Pose(26, 50, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose Release = new Pose(20, 70, Math.toRadians(140)); // Start Pose of our robot.;
    private final Pose MidRelease = new Pose(70, 74, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose MidReleaseB = new Pose(37.67269544924156, 55, Math.toRadians(180)); // Start Pose of our robot.;
    private final Pose ExitPose = new Pose(50, 80, Math.toRadians(180)); // Start Pose of our robot.;
    private PathChain scorePreload,pickup1,Shoot1,pickup2,Shoot2,pickup3, ReleaseArtifacts, Reverse, Shoot3, Exit;
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime waitimer = new ElapsedTime();


    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose.getHeading())
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, pickup))
                .setTangentHeadingInterpolation()
                .build();
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup, ShootPose))
                .setLinearHeadingInterpolation(pickup.getHeading(), ShootPose.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, midpickup2, pickuptwo))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), midpickup2.getHeading(), pickuptwo.getHeading())
                .build();
        ReleaseArtifacts = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose,MidRelease, Release))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), Release.getHeading())
                .build();
        Reverse = follower.pathBuilder()
                .addPath(new BezierCurve(Release,MidRelease, ShootPose))
                .setLinearHeadingInterpolation(Release.getHeading(), ShootPose.getHeading())
                .build();
        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickuptwo, midpickup2, ShootPose))
                .setLinearHeadingInterpolation(pickuptwo.getHeading(), ShootPose.getHeading())
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, midpickup3, pickupthree))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickupthree.getHeading())
                .build();
        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupthree, midpickup3, ShootPose))
                .setLinearHeadingInterpolation(pickupthree.getHeading(), ShootPose.getHeading())
                .build();
        Exit = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, ExitPose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), ExitPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                }
                if(waitimer.seconds() > 0.7){
                    Headlight.setPosition(1);
                }
                if(waitimer.seconds() > 3.5){
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    Headlight.setPosition(0);

                    follower.followPath(pickup2, true);
                    waitimer.reset();
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(Shoot2, true);
                    setPathState(-1);
                }
                waitimer.reset();
                break;
            case -1:
                if(waitimer.seconds() > 1.5){
                    Headlight.setPosition(1);

                }
                if(waitimer.seconds() > 3.5){
                    Headlight.setPosition(0);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(ReleaseArtifacts, true);
                    setPathState(4);
                }
                waitimer.reset();
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(Reverse, true);
                    if(waitimer.seconds() > 1.5){
                        Headlight.setPosition(1);
                    }
                    if(waitimer.seconds() > 4){
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(ReleaseArtifacts, true);
                    setPathState(6);
                }
                waitimer.reset();
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(Reverse, true);
                }
                if(waitimer.seconds() > 4){
                setPathState(7);
                 }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(pickup1, true);
                    setPathState(8);
                }
                waitimer.reset();
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(Shoot1, true);
                }
                if(waitimer.seconds() > 3.5){
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(pickup3, true);
                    setPathState(10);
                }
                waitimer.reset();

                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(Shoot3, true);
                }
                if(waitimer.seconds() > 5.5){
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(Exit, true);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer", waitimer.seconds());

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
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

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        waitimer.reset();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
