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

import java.util.function.Supplier;

/**
 * Two-driver test TeleOp with Pedro Pathing "drive to spot" buttons.
 *
 * Gamepad 1 (driver):
 *   Left stick / right stick X   drive and turn
 *   B (hold)                     slow mode
 *   A (press)                    auto-drive to the close shooting spot
 *   B (press)                    auto-drive to the far shooting spot
 *   Move a stick                 cancel auto-drive and take control back
 *
 * Gamepad 2 (operator):
 *   A (hold)                     shooter OFF (otherwise always spinning)
 *   Left/right trigger           kicker (at 30% power)
 *   Right bumper / left bumper   intake in / out
 */
@Configurable
@TeleOp
public class TestPedroTele extends OpMode {

    private DcMotorEx Shooter;   // DcMotorEx so we can set a velocity
    private DcMotor Kicker;
    private DcMotor Intake;
    private Follower follower;   // Pedro Pathing drive and position tracking

    private Limelight3A limelight;

    public static Pose startingPose;

    private boolean automatedDrive = false;   // true while Pedro is driving a path

    // Paths that start from the robot's current position (built when a button is pressed).
    private Supplier<PathChain> pathShootClose, pathShootFar;

    private TelemetryManager telemetryM;   // Panels dashboard telemetry

    private boolean slowMode = false;
    private boolean stayInPlace = false;

    private ElapsedTime waitimer;      // time since auto-drive started
    private ElapsedTime Endgametimer;  // time since START, for the rumble

    private double slowModeMultiplier = 0.2;   // slow mode = 20% speed
    private double ShootPower = 0.70;          // only shown on telemetry

    private float GameRightStickX;
    private float GameLeftStickX;
    private float GameLeftStickY;

    private VoltageSensor voltageSensor;

    // Battery voltage compensation. NOTE: TeleOp1Player uses 0.30 here.
    private static final double PERFECT_VOLTAGE = 13.2;
    private static final double VOLTAGE_COMP_STRENGTH = 0.05;

    /**
     * Sets the shooter speed, nudged up a little when the battery is low.
     * Math.max(..., 1.0) stops a divide-by-zero if the sensor reads 0 V.
     */
    private void setCompensatedShooterVelocity(double baseVelocity) {

        double currentVoltage = voltageSensor.getVoltage();

        double fullCorrection = PERFECT_VOLTAGE / Math.max(currentVoltage, 1.0);

        double softenedCorrection = 1 + (fullCorrection - 1) * VOLTAGE_COMP_STRENGTH;

        Shooter.setVelocity(baseVelocity * softenedCorrection);
    }

    @Override
    public void init() {

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        follower = Constants.createFollower(hardwareMap);

        // Test start position: field origin, facing 90 degrees.
        startingPose = new Pose(0, 0, Math.toRadians(90));

        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitimer = new ElapsedTime();
        Endgametimer = new ElapsedTime();

        // A button: drive from here to (55, 0), turning to face 270 degrees.
        pathShootClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(55, 0))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(270), 0.6)
                )
                .build();

        // B button: drive from here to (30, 30), turning to face 90 degrees.
        pathShootFar = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(30, 30))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.6)
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

        // Show what the Limelight sees (only when it has a valid target).
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Ty", llResult.getTy());
        }

        // Sticks are flipped so "push forward" and "push left" are positive.
        GameLeftStickX = -gamepad1.left_stick_x;
        GameLeftStickY = -gamepad1.left_stick_y;
        GameRightStickX = -gamepad1.right_stick_x;

        // SHOOTER (gamepad 2): always spinning unless A is held.
        if (gamepad2.a) {
            Shooter.setPower(0);
        } else {
            setCompensatedShooterVelocity(-1485);
        }

        // KICKER (gamepad 2): triggers, scaled down to 30% so it moves gently.
        Kicker.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * 0.3);

        // INTAKE (gamepad 2)
        if (gamepad2.right_bumper) {
            Intake.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            Intake.setPower(-1.0);
        } else {
            Intake.setPower(0);
        }

        // ENDGAME RUMBLE: short buzz at 100 s, long buzz at 115 s of TeleOp.
        if (Endgametimer.seconds() > 100 && Endgametimer.seconds() < 101) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        if (Endgametimer.seconds() > 115 && Endgametimer.seconds() < 117) {
            gamepad1.rumble(1500);
            gamepad2.rumble(1500);
        }

        // STAY IN PLACE toggle (saved, but nothing uses it yet).
        if (gamepad1.dpadLeftWasPressed()) {
            stayInPlace = true;
        }

        if (gamepad1.dpadRightWasPressed()) {
            stayInPlace = false;
        }

        // NOTE: B is both "slow mode" and "drive to far spot".
        slowMode = gamepad1.b;

        // AUTO-DRIVE buttons: Pedro takes over and drives the path.
        if (gamepad1.aWasPressed()) {

            follower.followPath(pathShootClose.get());
            automatedDrive = true;
            waitimer.reset();

        } else if (gamepad1.bWasPressed()) {

            follower.followPath(pathShootFar.get());
            automatedDrive = true;
            waitimer.reset();
        }

        // CANCEL auto-drive when the driver moves a stick.
        // NOTE: because && runs before ||, the 0.4 s wait only applies to
        // left_stick_x. Also only POSITIVE stick values count (no Math.abs),
        // so pushing the stick the other way does not cancel.
        if (automatedDrive && ((waitimer.seconds() > 0.4 && (gamepad1.left_stick_x) > 0.05 || (gamepad1.left_stick_y) > 0.05 || (gamepad1.right_stick_x) > 0.05))) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // MANUAL DRIVE
        if (!automatedDrive) {

            double forward = GameLeftStickY;
            double strafe = GameLeftStickX;
            double turn = GameRightStickX;

            if (slowMode) {
                forward *= slowModeMultiplier;
                strafe *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }

            // "true" = robot-centric driving
            follower.setTeleOpDrive(forward, strafe, turn, true);
        }

        // Pedro must update every loop to track where the robot is.
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
