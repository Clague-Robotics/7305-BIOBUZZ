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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOp1Player extends OpMode {

    //change

    private DcMotorEx Shooter;
    private DcMotor Kicker;
    private DcMotor Intake;

    private Follower follower;

    public static Pose startingPose;

    private boolean automatedDrive;

    private Supplier<PathChain> pathShootClose, pathShootFar;

    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private boolean stayInPlace;
    private boolean StopAutoTele = false;

    ElapsedTime waitimer;
    ElapsedTime Endgametimer;

    private double slowModeMultiplier = 0.2;
    private double ShootPower = 0.70;
    private VoltageSensor voltageSensor;

    private static final double PERFECT_VOLTAGE = 13.2;
    private static final double VOLTAGE_COMP_STRENGTH = 0.30;

    private float GameRightStickX;
    private float GameLeftStickX;
    private float GameLeftStickY;
    private void setCompensatedShooterVelocity(double baseVelocity) {
        double currentVoltage = voltageSensor.getVoltage();

        double fullCorrection = PERFECT_VOLTAGE / currentVoltage;

        double softenedCorrection =
                1 + (fullCorrection - 1) * VOLTAGE_COMP_STRENGTH;

        Shooter.setVelocity(baseVelocity * softenedCorrection);
    }

    @Override
    public void init() {

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        follower = Constants.createFollower(hardwareMap);

        Pose startingPose = new Pose(
                55.504667444574096,
                7.834889148191378,
                90
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
                                        new Pose(25, 85)
                                )
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(160),
                                0.6
                        )
                )
                .build();

        pathShootFar = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower::getPose,
                                        new Pose(50, 12)
                                )
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(135),
                                0.6
                        )
                )
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        GameLeftStickX = -gamepad1.left_stick_x;
        GameLeftStickY = -gamepad1.left_stick_y;
        GameRightStickX = -gamepad1.right_stick_x;


        if (gamepad1.a) {
            Shooter.setPower(0);
        } else {
            setCompensatedShooterVelocity(-1485);
        }

        Kicker.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.right_bumper) {
            Intake.setPower(1.0);
        }
        else if (gamepad1.left_bumper) {
            Intake.setPower(-1.0);
        }
        else {
            Intake.setPower(0);
        }




        // ENDGAME RUMBLE
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


        // GAMEPAD 1 - STAY IN PLACE
        if (gamepad1.dpadLeftWasPressed()) {
            stayInPlace = true;
        }

        if (gamepad1.dpadRightWasPressed()) {
            stayInPlace = false;
        }


        follower.update();
        telemetryM.update();


        // GAMEPAD 1 - SLOW MODE
        slowMode = gamepad1.right_bumper;


        // DRIVE
        if (!automatedDrive) {

            if (!slowMode) {

                follower.setTeleOpDrive(
                        GameLeftStickY,
                        GameLeftStickX,
                        GameRightStickX,
                        true
                );

            } else {

                follower.setTeleOpDrive(
                        GameLeftStickY * slowModeMultiplier,
                        GameLeftStickX * slowModeMultiplier,
                        GameRightStickX * slowModeMultiplier,
                        true
                );
            }
        }


        // TELEMETRY
        telemetry.addData("ShootPower", ShootPower);
        telemetry.addData("GameTime", Endgametimer.seconds());

        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
