package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Limelight Auto Align (71in)", group = "Autonomous")
public class AimbotV2 extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private Limelight3A limelight;
    private Follower follower;

    private static final int APRILTAG_PIPELINE = 4;
    private static final double TARGET_DISTANCE_IN = 71.0;
    private static final double DISTANCE_TOLERANCE_IN = 1.0;

    private static final double TURN_POWER = 0.2;
    private static final long TURN_RIGHT_MS = 800;
    private static final long TURN_LEFT_MS = 1600;
    private static final int SCAN_INTERVAL_MS = 50;

    private static final double DISTANCE_SCALE_SQRT_IN = 67.0; // calibrate with ta%
    private static final double FOLLOWER_MAX_POWER = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Limelight Auto Align...");
        telemetry.update();

        // --- Hardware mapping ---
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Limelight setup ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // --- PedroPathing follower ---
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(FOLLOWER_MAX_POWER);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Waiting for Limelight warm-up...");
        telemetry.update();
        sleep(2000);

        telemetry.addLine("Ready to start alignment...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        boolean found = searchForAnyTag();
        if (found && opModeIsActive()) {
            adjustToTargetDistance();
        } else {
            telemetry.addLine("No AprilTag detected during scan.");
        }

        stopAllDriveMotors();
        follower.breakFollowing();
        limelight.stop();

        telemetry.addLine("=== Auto Align Complete ===");
        telemetry.update();
    }

    private boolean searchForAnyTag() throws InterruptedException {
        telemetry.addLine("Sweeping RIGHT...");
        if (scanWhileTurning(TURN_POWER, -TURN_POWER, TURN_RIGHT_MS)) return true;

        telemetry.addLine("Sweeping LEFT...");
        if (scanWhileTurning(-TURN_POWER, TURN_POWER, TURN_LEFT_MS)) return true;

        telemetry.addLine("Tag not found after sweeps.");
        telemetry.update();
        return false;
    }

    private boolean scanWhileTurning(double leftPower, double rightPower, long durationMs) throws InterruptedException {
        long start = System.currentTimeMillis();
        setDrivePowers(leftPower, leftPower, rightPower, rightPower);

        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    telemetry.addData("Tag Detected", fiducials.get(0).getFiducialId());
                    telemetry.update();
                    stopAllDriveMotors();
                    sleep(300);
                    return true;
                }
            }
            sleep(SCAN_INTERVAL_MS);
        }
        stopAllDriveMotors();
        return false;
    }

    private void adjustToTargetDistance() throws InterruptedException {
        telemetry.addLine("=== Measuring Distance to Tag ===");
        telemetry.update();
        sleep(400);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("Invalid Limelight result!");
            telemetry.update();
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addLine("No fiducials in result!");
            telemetry.update();
            return;
        }

        LLResultTypes.FiducialResult tag = fiducials.get(0);
        double ta = result.getTa();
        double distanceIn = getDistanceFromTagInches(ta);
        double delta = distanceIn - TARGET_DISTANCE_IN;

        telemetry.addData("Target Area %", ta);
        telemetry.addData("Calculated Dist (in)", distanceIn);
        telemetry.addData("Delta (in)", delta);
        telemetry.update();

        if (Math.abs(delta) < DISTANCE_TOLERANCE_IN) {
            telemetry.addLine("Within tolerance, no movement required.");
            telemetry.update();
            return;
        }

        resetPedroPose();

        if (delta > 0) {
            telemetry.addData("Too far", "Moving FORWARD %.1f in", delta);
            moveForward(delta);
        } else {
            telemetry.addData("Too close", "Moving BACKWARD %.1f in", Math.abs(delta));
            moveBackward(Math.abs(delta));
        }

        telemetry.addLine("Distance adjustment complete.");
        telemetry.update();
    }

    private double getDistanceFromTagInches(double targetArea) {
        if (targetArea < 0.01) return 999.0;
        return DISTANCE_SCALE_SQRT_IN / Math.sqrt(targetArea);
    }

    private void moveForward(double inches) throws InterruptedException {
        Pose start = follower.getPose();
        Pose target = new Pose(start.getX(), start.getY() + inches, start.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Moving forward...");
            telemetry.update();
        }
        follower.breakFollowing();
    }

    private void moveBackward(double inches) throws InterruptedException {
        Pose start = follower.getPose();
        Pose target = new Pose(start.getX(), start.getY() - inches, start.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Moving backward...");
            telemetry.update();
        }
        follower.breakFollowing();
    }

    private void resetPedroPose() {
        follower.breakFollowing();
        follower.setStartingPose(new Pose(0, 0, 0));
        telemetry.addLine("Pedro pose reset to (0,0,0)");
        telemetry.update();
    }

    private void setDrivePowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private void stopAllDriveMotors() {
        setDrivePowers(0, 0, 0, 0);
    }
}
