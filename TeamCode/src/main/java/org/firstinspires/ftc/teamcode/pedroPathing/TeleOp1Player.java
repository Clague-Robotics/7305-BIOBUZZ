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

/**
 * One-driver TeleOp: gamepad 1 drives AND runs the shooter, kicker and intake.
 * Driving goes through Pedro Pathing's follower (robot-centric).
 *
 * Controls (gamepad 1):
 *   Left stick       drive / strafe
 *   Right stick X    turn
 *   A (hold)         shooter OFF (otherwise the shooter always spins)
 *   Left trigger     kicker forward    Right trigger  kicker backward
 *   Right bumper     intake in  (also turns on slow mode, see NOTE below)
 *   Left bumper      intake out
 *   D-pad left/right stayInPlace on/off (not used yet)
 */
@Configurable
@TeleOp
public class TeleOp1Player extends OpMode {

    private DcMotorEx Shooter;   // DcMotorEx so we can set a velocity, not just a power
    private DcMotor Kicker;
    private DcMotor Intake;

    private Follower follower;   // Pedro Pathing: drives the robot and tracks its position

    public static Pose startingPose;

    private boolean automatedDrive;   // true while Pedro is driving a path for us

    // Paths that start from wherever the robot is right now (built when needed).
    // NOTE: these are set up in init() but no button runs them yet.
    private Supplier<PathChain> pathShootClose, pathShootFar;

    private TelemetryManager telemetryM;   // Panels dashboard telemetry

    private boolean slowMode = false;
    private boolean stayInPlace;

    ElapsedTime Endgametimer;   // counts seconds for the endgame rumble

    private double slowModeMultiplier = 0.2;   // slow mode = 20% speed
    private double ShootPower = 0.70;          // only shown on telemetry; the shooter uses a velocity
    private VoltageSensor voltageSensor;

    // Battery voltage compensation (see setCompensatedShooterVelocity).
    private static final double PERFECT_VOLTAGE = 13.2;        // the voltage our shooter speeds were tuned at
    private static final double VOLTAGE_COMP_STRENGTH = 0.30;  // 0 = no correction, 1 = full correction

    private float GameRightStickX;
    private float GameLeftStickX;
    private float GameLeftStickY;

    /**
     * Sets the shooter speed, nudged up when the battery is low.
     * A weaker battery spins the wheel slower, so we ask for a bit more.
     * Example: at 12.0 V, full correction is 13.2 / 12.0 = 1.10 (10% more);
     * at strength 0.30 we only add 30% of that, so 3% more.
     */
    private void setCompensatedShooterVelocity(double baseVelocity) {
        double currentVoltage = voltageSensor.getVoltage();

        double fullCorrection = PERFECT_VOLTAGE / currentVoltage;

        double softenedCorrection = 1 + (fullCorrection - 1) * VOLTAGE_COMP_STRENGTH;

        Shooter.setVelocity(baseVelocity * softenedCorrection);
    }

    @Override
    public void init() {

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Kicker = hardwareMap.get(DcMotor.class, "Kicker");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        follower = Constants.createFollower(hardwareMap);

        // Where the robot starts on the field (x inches, y inches, heading).
        // NOTE: this makes a new local variable instead of setting the
        // startingPose field above, and the heading 90 is not wrapped in
        // Math.toRadians() like the other files.
        Pose startingPose = new Pose(
                55.504667444574096,
                7.834889148191378,
                90
        );

        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // NOTE: this timer starts at INIT, not at START, so the rumbles
        // happen early if the robot sits in init before the match.
        Endgametimer = new ElapsedTime();

        // Drive from the current position to the close shooting spot,
        // turning to face 160 degrees.
        pathShootClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(25, 85))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(160), 0.6)
                )
                .build();

        // Drive from the current position to the far shooting spot,
        // turning to face 135 degrees.
        pathShootFar = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(50, 12))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.6)
                )
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // Sticks are flipped so "push forward" and "push left" are positive.
        GameLeftStickX = -gamepad1.left_stick_x;
        GameLeftStickY = -gamepad1.left_stick_y;
        GameRightStickX = -gamepad1.right_stick_x;

        // SHOOTER: always spinning at -1485 ticks/second unless A is held.
        if (gamepad1.a) {
            Shooter.setPower(0);
        } else {
            setCompensatedShooterVelocity(-1485);
        }

        // KICKER: left trigger pushes one way, right trigger the other.
        Kicker.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        // INTAKE
        if (gamepad1.right_bumper) {
            Intake.setPower(1.0);
        } else if (gamepad1.left_bumper) {
            Intake.setPower(-1.0);
        } else {
            Intake.setPower(0);
        }

        // ENDGAME RUMBLE: TeleOp is 120 s. Short buzz at 100 s, long buzz at 115 s.
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

        // Pedro must update every loop to track where the robot is.
        follower.update();
        telemetryM.update();

        // SLOW MODE
        // NOTE: right bumper is ALSO intake-in, so the robot always drives
        // slowly while intaking. Check that is what the drivers want.
        slowMode = gamepad1.right_bumper;

        // DRIVE (the last "true" means robot-centric: forward = where the robot faces)
        if (!automatedDrive) {

            if (!slowMode) {
                follower.setTeleOpDrive(GameLeftStickY, GameLeftStickX, GameRightStickX, true);
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
