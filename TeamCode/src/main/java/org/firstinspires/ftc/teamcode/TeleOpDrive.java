package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Driver-controlled mecanum drive with Limelight AprilTag auto-aim.
 *
 * Controls (gamepad 1):
 *   Left stick      drive forward/back and strafe left/right
 *   Right stick X   turn
 *   Right bumper    slow mode (25% speed)
 *   Left bumper     HOLD to auto-turn toward an AprilTag
 *
 * This OpMode drives the motors itself (it does not use Pedro Pathing).
 */
@TeleOp(name = "TeleOpDrive")
public class TeleOpDrive extends LinearOpMode {

    // Drive motors. Names must match the Robot Configuration on the Driver Hub.
    private DcMotor FL, BL, FR, BR;
    private Limelight3A limelight;

    // Auto-aim tuning. "tx" is how many degrees left/right the tag is from
    // the center of the camera image.
    private static final double KP_TURN = 0.038;   // turn power per degree of error
    private static final double MAX_TURN = 0.35;   // never turn faster than this
    private static final double MIN_TURN = 0.07;   // smallest power that still moves the robot

    private static final double TARGET_TX = 3.0;        // the tx (degrees) that counts as "aimed"
    private static final double ALIGN_TOLERANCE = 1.0;  // within +/- this many degrees = aligned

    // The tag we are aiming at while the left bumper is held. -1 means none yet.
    private int lockedTagID = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- INIT ----------
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        // Right side is reversed so that positive power drives every wheel forward.
        // NOTE: Constants.java (used by Pedro Pathing) sets the LEFT side reversed
        // instead. Check on the robot which one is right for this drive base.
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrakeMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Pipeline 4 on the Limelight is set up for AprilTags.
        limelight.pipelineSwitch(4);
        limelight.start();

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Limelight Pipeline 4");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        // ---------- MAIN LOOP (runs many times per second) ----------
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean aprilTagDetected = false;
            boolean aligning = gamepad1.left_bumper;

            int tagID = -1;

            double tx = 0;   // degrees left/right of center
            double ty = 0;   // degrees up/down of center
            double ta = 0;   // how much of the image the tag fills (bigger = closer)

            double distanceInches = -1;

            // Tag position measured from the camera, in meters.
            double cameraX = 0;
            double cameraY = 0;
            double cameraZ = 0;

            boolean poseAvailable = false;

            LLResultTypes.FiducialResult selectedTag = null;

            // --- Pick which AprilTag to use ---
            if (result != null &&
                    result.isValid() &&
                    result.getFiducialResults() != null &&
                    !result.getFiducialResults().isEmpty()) {

                aprilTagDetected = true;

                if (aligning && lockedTagID != -1) {
                    // Already locked on: only follow that same tag, so the
                    // robot doesn't jump between tags while turning.
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        if (tag.getFiducialId() == lockedTagID) {
                            selectedTag = tag;
                            break;
                        }
                    }
                } else {
                    // Not locked yet: use the first tag the camera sees.
                    selectedTag = result.getFiducialResults().get(0);

                    // Driver just started aligning, so lock onto this tag.
                    if (aligning && lockedTagID == -1) {
                        lockedTagID = selectedTag.getFiducialId();
                    }
                }
            }

            // --- Read the chosen tag's position ---
            if (selectedTag != null) {

                tagID = selectedTag.getFiducialId();

                tx = selectedTag.getTargetXDegrees();
                ty = selectedTag.getTargetYDegrees();
                ta = selectedTag.getTargetArea();

                Pose3D tagPose = selectedTag.getTargetPoseCameraSpace();

                if (tagPose != null && tagPose.getPosition() != null) {

                    cameraX = tagPose.getPosition().x;
                    cameraY = tagPose.getPosition().y;
                    cameraZ = tagPose.getPosition().z;

                    // Straight-line distance to the tag (3D Pythagorean theorem).
                    double distanceMeters = Math.sqrt(
                            cameraX * cameraX +
                                    cameraY * cameraY +
                                    cameraZ * cameraZ
                    );

                    // Ignore broken readings (NaN, infinity, or zero).
                    if (Double.isFinite(distanceMeters) && distanceMeters > 0.001) {
                        distanceInches = distanceMeters * 39.3701;  // meters to inches
                        poseAvailable = true;
                    }
                }
            }

            // --- Telemetry: what the camera sees ---
            telemetry.addData("AprilTag Detected", aprilTagDetected ? "YES" : "NO");
            telemetry.addData("Tag ID", selectedTag != null ? tagID : "NONE");

            if (selectedTag != null) {

                telemetry.addData("tx", "%.2f", tx);
                telemetry.addData("ty", "%.2f", ty);
                telemetry.addData("ta", "%.3f", ta);

                if (poseAvailable) {
                    telemetry.addData("Distance", "%.2f inches", distanceInches);
                    telemetry.addData("Distance Status", "3D POSE AVAILABLE");
                } else {
                    telemetry.addData("Distance", "N/A");
                    telemetry.addData("Distance Status", "INVALID 3D POSE");
                }

                telemetry.addData("Camera X", "%.4f", cameraX);
                telemetry.addData("Camera Y", "%.4f", cameraY);
                telemetry.addData("Camera Z", "%.4f", cameraZ);

            } else {

                telemetry.addData("Distance", "N/A");
                telemetry.addData("tx", "N/A");
                telemetry.addData("ty", "N/A");
                telemetry.addData("ta", "N/A");
                telemetry.addData("Camera X", "N/A");
                telemetry.addData("Camera Y", "N/A");
                telemetry.addData("Camera Z", "N/A");
            }

            if (result != null) {
                telemetry.addData("Pipeline", result.getPipelineIndex());
                telemetry.addData("Pipeline Type", result.getPipelineType());
            }

            if (aligning) {

                // ---------- AUTO-AIM (left bumper held) ----------
                setBrakeMode();

                telemetry.addData("Locked Tag", lockedTagID == -1 ? "NONE" : lockedTagID);

                if (selectedTag == null) {

                    // Can't see the tag: stop instead of guessing.
                    setDrivePower(0, 0, 0, 0);
                    telemetry.addLine("ALIGNMENT: TAG NOT VISIBLE");

                } else {

                    // Proportional control: the farther off we are, the harder we turn.
                    double turnError = tx - TARGET_TX;

                    double turnOutput = -turnError * KP_TURN;

                    // Clamp between -MAX_TURN and +MAX_TURN.
                    turnOutput = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnOutput));

                    if (Math.abs(turnError) <= ALIGN_TOLERANCE) {

                        // Close enough: stop turning.
                        turnOutput = 0;
                        telemetry.addLine("ALIGNMENT: ALIGNED");

                    } else {

                        // Small powers can't overcome friction, so push at
                        // least MIN_TURN in the right direction.
                        if (Math.abs(turnOutput) < MIN_TURN) {
                            turnOutput = Math.copySign(MIN_TURN, turnOutput);
                        }

                        telemetry.addLine("ALIGNMENT: TURNING");
                    }

                    // Left wheels one way, right wheels the other = turn in place.
                    setDrivePower(turnOutput, turnOutput, -turnOutput, -turnOutput);

                    telemetry.addData("Target TX", "%.2f", TARGET_TX);
                    telemetry.addData("Turn Error", "%.2f", turnError);
                    telemetry.addData("Turn Power", "%.3f", turnOutput);
                }

            } else {

                // ---------- MANUAL DRIVING ----------
                lockedTagID = -1;  // forget the tag so the next align picks a fresh one

                double spd = gamepad1.right_bumper ? 0.25 : 1.0;  // slow mode

                double y = -gamepad1.left_stick_y * spd;        // forward (stick Y is upside down)
                double x = gamepad1.left_stick_x * 1.1 * spd;   // strafe; 1.1 makes up for mecanum strafing slower
                double rx = gamepad1.right_stick_x * spd;       // turn

                // Standard mecanum math. Dividing by the denominator keeps every
                // power between -1 and 1 while keeping the wheels in proportion.
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

                double FLP = (y + x + rx) / denominator;
                double BLP = (y - x + rx) / denominator;
                double FRP = (y - x - rx) / denominator;
                double BRP = (y + x - rx) / denominator;

                // Sticks centered: brake so the robot holds still.
                // Sticks moving: float so the robot glides smoothly.
                if (Math.abs(y) < 0.001 && Math.abs(x) < 0.001 && Math.abs(rx) < 0.001) {
                    setBrakeMode();
                } else {
                    setFloatMode();
                }

                setDrivePower(FLP, BLP, FRP, BRP);

                telemetry.addLine("MANUAL DRIVING");
            }

            telemetry.update();
        }

        // OpMode ended: stop everything.
        setDrivePower(0, 0, 0, 0);
        limelight.stop();
    }

    /** Sets all four drive motor powers at once (front-left, back-left, front-right, back-right). */
    private void setDrivePower(double fl, double bl, double fr, double br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }

    /** At zero power, the motors actively resist turning (robot stops fast). */
    private void setBrakeMode() {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** At zero power, the motors spin freely (robot coasts). */
    private void setFloatMode() {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
