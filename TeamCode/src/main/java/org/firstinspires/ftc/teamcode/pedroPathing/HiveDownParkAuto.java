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

@Autonomous(group = "HiveUp")
public class HiveDownParkAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Servo Headlight;
    private DcMotor Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;
    private final Pose startPose = new Pose(55, 8, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose parkmid = new Pose(55, 20, Math.toRadians(270)); // Start Pose of our robot.;

    private final Pose park = new Pose(99, 36, Math.toRadians(270)); // Start Pose of our robot.;



    private PathChain park1;
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime waitimer = new ElapsedTime();


    public void buildPaths() {
        park1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, parkmid ,park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
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

