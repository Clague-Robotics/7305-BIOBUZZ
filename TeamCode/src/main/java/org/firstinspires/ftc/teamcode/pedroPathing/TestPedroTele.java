
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class TestPedroTele extends OpMode {

    private DcMotorEx Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;
    private Follower follower;

    private Limelight3A limelight;

    public static Pose startingPose;

    private boolean automatedDrive = false;

    private Supplier<PathChain> pathShootClose, pathShootFar;

    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private boolean stayInPlace = false;
    private boolean StopAutoTele = false;

    private ElapsedTime waitimer;
    private ElapsedTime Endgametimer;

    private double slowModeMultiplier = 0.2;
    private double ShootPower = 0.70;

    private float GameRightStickX;
    private float GameLeftStickX;
    private float GameLeftStickY;

    private VoltageSensor voltageSensor;

    private static final double PERFECT_VOLTAGE = 13.2;
    private static final double VOLTAGE_COMP_STRENGTH = 0.05;




    private void setCompensatedShooterVelocity(double baseVelocity) {

        double currentVoltage = voltageSensor.getVoltage();

        double fullCorrection =
                PERFECT_VOLTAGE / Math.max(currentVoltage, 1.0);

        double softenedCorrection =
                1 + (fullCorrection - 1) * VOLTAGE_COMP_STRENGTH;

        Shooter.setVelocity(baseVelocity * softenedCorrection);
    }



    @Override
    public void init() {

        voltageSensor =
                hardwareMap.voltageSensor.iterator().next();

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        follower = Constants.createFollower(hardwareMap);

        startingPose = new Pose(
                0,
                0,
                Math.toRadians(90)
        );

        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitimer = new ElapsedTime();
        Endgametimer = new ElapsedTime();

        pathShootClose = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower::getPose,
                                        new Pose(55, 0)
                                )
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(270),
                                0.6
                        )
                )
                .build();

        pathShootFar = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower::getPose,
                                        new Pose(30, 30)
                                )
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                0.6
                        )
                )
                .build();
    }

    @Override
    public void start() {

        follower.startTeleopDrive();

        limelight.start();

        Endgametimer.reset();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Ty", llResult.getTy());
        }
        //double tx =  llResult.getTx();


        GameLeftStickX = -gamepad1.left_stick_x;
        GameLeftStickY = -gamepad1.left_stick_y;
        GameRightStickX = -gamepad1.right_stick_x;



        if (gamepad2.a) {
            Shooter.setPower(0);
        } else {
            setCompensatedShooterVelocity(-1485);
        }

        Kicker.setPower(
                (gamepad2.left_trigger - gamepad2.right_trigger)
                        * 0.3
        );

        if (gamepad2.right_bumper) {
            Intake.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            Intake.setPower(-1.0);
        } else {
            Intake.setPower(0);
        }

        if (Endgametimer.seconds() > 100 &&
                Endgametimer.seconds() < 101) {

            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        if (Endgametimer.seconds() > 115 &&
                Endgametimer.seconds() < 117) {

            gamepad1.rumble(1500);
            gamepad2.rumble(1500);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            stayInPlace = true;
        }

        if (gamepad1.dpadRightWasPressed()) {
            stayInPlace = false;
        }

        slowMode = gamepad1.b;



        if (gamepad1.aWasPressed()) {

            follower.followPath(pathShootClose.get());
            automatedDrive = true;
            waitimer.reset();

        } else if (gamepad1.bWasPressed()) {

            follower.followPath(pathShootFar.get());
            automatedDrive = true;
            waitimer.reset();
        }



        if (automatedDrive && ((waitimer.seconds() > 0.4 && (gamepad1.left_stick_x) > 0.05 || (gamepad1.left_stick_y) > 0.05 || (gamepad1.right_stick_x) > 0.05))) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (!automatedDrive) {

            double forward = GameLeftStickY;
            double strafe = GameLeftStickX;
            double turn = GameRightStickX;

            if (slowMode) {

                forward *= slowModeMultiplier;
                strafe *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }


            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    turn,
                    true
            );
        }

        follower.update();

        telemetry.addData("ShootPower", ShootPower);
        telemetry.addData("GameTime", Endgametimer.seconds());



        telemetry.addData("Robot Pose", follower.getPose());

        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);


        telemetryM.update();
    }

    @Override
    public void stop() {

        limelight.stop();

        Shooter.setPower(0);
        Kicker.setPower(0);
        Intake.setPower(0);
    }
}
