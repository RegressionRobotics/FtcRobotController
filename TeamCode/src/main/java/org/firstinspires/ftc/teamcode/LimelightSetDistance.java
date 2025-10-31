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

@Autonomous(name = "Limelight Distance Control Y (71in Base)", group = "Autonomous")
public class LimelightSetDistance extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private Limelight3A limelight;
    private Follower follower;

    // Config constants
    private static final int APRILTAG_PIPELINE = 4;
    private static final double TARGET_DISTANCE_IN = 71.0;
    private static final double DISTANCE_TOLERANCE_IN = 1.0;
    private static final double DISTANCE_SCALE_SQRT_IN = 67.0;  // adjust this based on real readings

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Limelight Distance Control (Y-axis)...");
        telemetry.update();

        // --- Motor setup ---
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

        // --- Pedro follower setup ---
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.4);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Waiting for Limelight to warm up...");
        telemetry.update();
        sleep(1500);

        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // ===== Get AprilTag result =====
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid Limelight result!");
            telemetry.update();
            sleep(1000);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addLine("No AprilTag detected!");
            telemetry.update();
            sleep(1000);
            return;
        }

        double ta = result.getTa(); // target area
        double currentDist = getDistanceFromTagInches(ta);
        double delta = currentDist - TARGET_DISTANCE_IN;

        telemetry.addData("Current Distance (in)", "%.1f", currentDist);
        telemetry.addData("Target Distance (in)", TARGET_DISTANCE_IN);
        telemetry.addData("Delta (in)", "%.1f", delta);
        telemetry.update();

        if (Math.abs(delta) < DISTANCE_TOLERANCE_IN) {
            telemetry.addLine("Already within tolerance. Done!");
            telemetry.update();
            return;
        }

        resetPedroPose();

        if (delta > 0) {
            telemetry.addData("Action", "Moving FORWARD by %.1f in", delta);
            telemetry.update();
            moveForwardY(delta);
        } else {
            telemetry.addData("Action", "Moving BACKWARD by %.1f in", Math.abs(delta));
            telemetry.update();
            moveBackwardY(Math.abs(delta));
        }

        telemetry.addLine("Final position reached.");
        telemetry.update();

        limelight.stop();
    }

    // Estimate distance in inches using area (simple sqrt inverse model)
    private double getDistanceFromTagInches(double targetArea) {
        if (targetArea <= 0.0) return 999.0;
        return DISTANCE_SCALE_SQRT_IN / Math.sqrt(targetArea);
    }

    private void moveForwardY(double inches) throws InterruptedException {
        Pose start = follower.getPose();
        Pose target = new Pose(start.getX(), start.getY() + inches, start.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Moving forward on Y-axis...");
            telemetry.update();
        }
        follower.breakFollowing();
    }

    private void moveBackwardY(double inches) throws InterruptedException {
        Pose start = follower.getPose();
        Pose target = new Pose(start.getX(), start.getY() - inches, start.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Moving backward on Y-axis...");
            telemetry.update();
        }
        follower.breakFollowing();
    }

    private void resetPedroPose() {
        follower.breakFollowing();
        follower.setStartingPose(new Pose(0, 0, 0));
        telemetry.addLine("Pedro pose reset (0,0,0)");
        telemetry.update();
    }
}
