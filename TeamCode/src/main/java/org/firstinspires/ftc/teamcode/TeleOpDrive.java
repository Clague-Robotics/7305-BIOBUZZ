
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TeleOpDrive")
public class TeleOpDrive extends LinearOpMode {

    private DcMotor FL, BL, FR, BR;
    private Limelight3A limelight;

    private static final double KP_TURN = 0.038;
    private static final double MAX_TURN = 0.35;
    private static final double MIN_TURN = 0.07;

    private static final double TARGET_TX = 3.0;
    private static final double ALIGN_TOLERANCE = 1.0;

    private int lockedTagID = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrakeMode();

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

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

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean aprilTagDetected = false;
            boolean aligning = gamepad1.left_bumper;

            int tagID = -1;

            double tx = 0;
            double ty = 0;
            double ta = 0;

            double distanceInches = -1;

            double cameraX = 0;
            double cameraY = 0;
            double cameraZ = 0;

            boolean poseAvailable = false;

            LLResultTypes.FiducialResult selectedTag = null;

            if (result != null &&
                    result.isValid() &&
                    result.getFiducialResults() != null &&
                    !result.getFiducialResults().isEmpty()) {

                aprilTagDetected = true;

                if (aligning && lockedTagID != -1) {

                    for (LLResultTypes.FiducialResult tag :
                            result.getFiducialResults()) {

                        if (tag.getFiducialId() == lockedTagID) {
                            selectedTag = tag;
                            break;
                        }
                    }

                } else {

                    selectedTag =
                            result.getFiducialResults().get(0);

                    if (aligning && lockedTagID == -1) {
                        lockedTagID =
                                selectedTag.getFiducialId();
                    }
                }
            }

            if (selectedTag != null) {

                tagID = selectedTag.getFiducialId();

                tx = selectedTag.getTargetXDegrees();
                ty = selectedTag.getTargetYDegrees();
                ta = selectedTag.getTargetArea();

                Pose3D tagPose =
                        selectedTag.getTargetPoseCameraSpace();

                if (tagPose != null &&
                        tagPose.getPosition() != null) {

                    cameraX = tagPose.getPosition().x;
                    cameraY = tagPose.getPosition().y;
                    cameraZ = tagPose.getPosition().z;

                    double distanceMeters = Math.sqrt(
                            cameraX * cameraX +
                                    cameraY * cameraY +
                                    cameraZ * cameraZ
                    );

                    if (Double.isFinite(distanceMeters) &&
                            distanceMeters > 0.001) {

                        distanceInches =
                                distanceMeters * 39.3701;

                        poseAvailable = true;
                    }
                }
            }

            telemetry.addData(
                    "AprilTag Detected",
                    aprilTagDetected ? "YES" : "NO"
            );

            telemetry.addData(
                    "Tag ID",
                    selectedTag != null ? tagID : "NONE"
            );

            if (selectedTag != null) {

                telemetry.addData("tx", "%.2f", tx);
                telemetry.addData("ty", "%.2f", ty);
                telemetry.addData("ta", "%.3f", ta);

                if (poseAvailable) {

                    telemetry.addData(
                            "Distance",
                            "%.2f inches",
                            distanceInches
                    );

                    telemetry.addData(
                            "Distance Status",
                            "3D POSE AVAILABLE"
                    );

                } else {

                    telemetry.addData("Distance", "N/A");

                    telemetry.addData(
                            "Distance Status",
                            "INVALID 3D POSE"
                    );
                }

                telemetry.addData(
                        "Camera X",
                        "%.4f",
                        cameraX
                );

                telemetry.addData(
                        "Camera Y",
                        "%.4f",
                        cameraY
                );

                telemetry.addData(
                        "Camera Z",
                        "%.4f",
                        cameraZ
                );

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

                telemetry.addData(
                        "Pipeline",
                        result.getPipelineIndex()
                );

                telemetry.addData(
                        "Pipeline Type",
                        result.getPipelineType()
                );
            }

            if (aligning) {

                setBrakeMode();

                telemetry.addData(
                        "Locked Tag",
                        lockedTagID == -1 ?
                                "NONE" : lockedTagID
                );

                if (selectedTag == null) {

                    setDrivePower(0, 0, 0, 0);

                    telemetry.addLine(
                            "ALIGNMENT: TAG NOT VISIBLE"
                    );

                } else {

                    double turnError = tx - TARGET_TX;

                    double turnOutput =
                           - turnError * KP_TURN;

                    turnOutput = Math.max(
                            -MAX_TURN,
                            Math.min(MAX_TURN, turnOutput)
                    );

                    if (Math.abs(turnError) <=
                            ALIGN_TOLERANCE) {

                        turnOutput = 0;

                        telemetry.addLine(
                                "ALIGNMENT: ALIGNED"
                        );

                    } else {

                        if (Math.abs(turnOutput) <
                                MIN_TURN) {

                            turnOutput = Math.copySign(
                                    MIN_TURN,
                                    turnOutput
                            );
                        }

                        telemetry.addLine(
                                "ALIGNMENT: TURNING"
                        );
                    }

                    setDrivePower(
                            turnOutput,
                            turnOutput,
                            -turnOutput,
                            -turnOutput
                    );

                    telemetry.addData(
                            "Target TX",
                            "%.2f",
                            TARGET_TX
                    );

                    telemetry.addData(
                            "Turn Error",
                            "%.2f",
                            turnError
                    );

                    telemetry.addData(
                            "Turn Power",
                            "%.3f",
                            turnOutput
                    );
                }

            } else {

                lockedTagID = -1;

                double spd =
                        gamepad1.right_bumper ?
                                0.25 : 1.0;

                double y =
                        -gamepad1.left_stick_y * spd;

                double x =
                        gamepad1.left_stick_x * 1.1 * spd;

                double rx =
                        gamepad1.right_stick_x * spd;

                double denominator = Math.max(
                        Math.abs(y) +
                                Math.abs(x) +
                                Math.abs(rx),
                        1.0
                );

                double FLP =
                        (y + x + rx) / denominator;

                double BLP =
                        (y - x + rx) / denominator;

                double FRP =
                        (y - x - rx) / denominator;

                double BRP =
                        (y + x - rx) / denominator;

                if (Math.abs(y) < 0.001 &&
                        Math.abs(x) < 0.001 &&
                        Math.abs(rx) < 0.001) {

                    setBrakeMode();

                } else {

                    setFloatMode();
                }

                setDrivePower(FLP, BLP, FRP, BRP);

                telemetry.addLine("MANUAL DRIVING");
            }

            telemetry.update();
        }

        setDrivePower(0, 0, 0, 0);
        limelight.stop();
    }

    private void setDrivePower(
            double fl,
            double bl,
            double fr,
            double br) {

        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }

    private void setBrakeMode() {

        FL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        BL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        FR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        BR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
    }

    private void setFloatMode() {

        FL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        BL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        FR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        BR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );
    }
}
