package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;

@Autonomous(name = "Spike Marks - Shooter + Servo Timed", group = "Autonomous")
public class SpikeMark extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intake, shooter;
    private CRServo leftTransfer, rightTransfer;

    private static final int APRILTAG_PIPELINE = 5;
    private static final int DEBUG_DELAY_MS = 250;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Mapping ---
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.6);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Initialized — Waiting for AprilTag...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        int detectedTagId = detectAprilTag();
        telemetry.addData("Detected Tag", detectedTagId);
        telemetry.update();

        sleep(3000); // debug pause

        // --- Main autonomous logic ---
        switch (detectedTagId) {
            case 21:
                executeTag21();
                break;
            case 22:
                executeTag22();
                break;
            case 23:
                executeTag23();
                break;
            default:
                executeDefault();
                break;
        }

        telemetry.addLine("Autonomous Complete ✅");
        telemetry.update();
    }

    // === Tag-specific methods ===
    private void executeTag21() throws InterruptedException {
        moveBackward(39);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(3);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        // Tag 21 extra move
        moveBackwardNoIntake(38.25);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startTimedShooterAndTransfer();
    }

    private void executeTag22() throws InterruptedException {
        moveBackward(63);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(3);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        // Tag 22 extra move
        moveBackwardNoIntake(11.5);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startTimedShooterAndTransfer();
    }

    private void executeTag23() throws InterruptedException {
        moveBackward(89);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(3);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(11);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startTimedShooterAndTransfer();
    }

    private void executeDefault() throws InterruptedException {
        moveBackward(33.5);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(3);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);
    }

    // === Helper methods ===
    private int detectAprilTag() {
        int timeout = 0;
        while (opModeIsActive() && timeout < 100) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    return (int) fiducials.get(0).getFiducialId();
                }
            }
            sleep(50);
            timeout++;
        }
        return -1;
    }

    private void moveBackward(double inches) {
        moveBackward(inches, false);
    }

    private void moveBackward(double inches, boolean stopIntakeAfter) {
        Pose startPose = follower.getPose();
        Pose targetPose = new Pose(startPose.getX() - inches, startPose.getY(), startPose.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();

        if (stopIntakeAfter) intake.setPower(0);
    }

    private void moveBackwardNoIntake(double inches) {
        intake.setPower(0);
        Pose startPose = follower.getPose();
        Pose targetPose = new Pose(startPose.getX() - inches, startPose.getY(), startPose.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
    }

    private void moveForwardHeadingRelative(double inches) {
        intake.setPower(0.85);
        Pose startPose = follower.getPose();
        double headingRad = Math.toRadians(startPose.getHeading());
        Pose targetPose = new Pose(
                startPose.getX() + inches * Math.cos(headingRad),
                startPose.getY() + inches * Math.sin(headingRad),
                startPose.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
    }

    private void moveForwardNoIntake(double inches) {
        intake.setPower(0);
        Pose startPose = follower.getPose();
        double headingRad = Math.toRadians(startPose.getHeading());
        Pose targetPose = new Pose(
                startPose.getX() + inches * Math.cos(headingRad),
                startPose.getY() + inches * Math.sin(headingRad),
                startPose.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.breakFollowing();
    }

    private void pivotTurnClockwise() throws InterruptedException {
        double power = 0.4;
        long duration = 490;
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        sleep(duration);
        stopAllDriveMotors();
    }

    private void pivotTurnAnticlockwise90() throws InterruptedException {
        double power = 0.4;
        long duration = 490;
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        sleep(duration);
        stopAllDriveMotors();
    }

    private void pivotTurnAnticlockwise45() throws InterruptedException {
        double power = 0.4;
        long duration = 245;
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        sleep(duration);
        stopAllDriveMotors();
    }

    private void stopAllDriveMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(50);
    }

    /** Timed Shooter + Transfer sequence (mimics TeleOp pattern) */
    private void startTimedShooterAndTransfer() throws InterruptedException {
        // Shooter ON continuously
        shooter.setPower(1.0);
        telemetry.addLine("Shooter: ON continuously");
        telemetry.update();

        // Warm-up delay
        sleep(1000);

        // CRServos ON continuously
        leftTransfer.setPower(1.0);
        rightTransfer.setPower(1.0);
        telemetry.addLine("CRServos: ON continuously");
        telemetry.update();

        // Run for 5 seconds then stop everything
        sleep(5000);
        shooter.setPower(0);
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
        telemetry.addLine("Shooter + CRServos: STOPPED after 5s");
        telemetry.update();
    }

    private void resetPedroPose() {
        follower.breakFollowing();
        follower.setStartingPose(new Pose(0, 0, 0));
        telemetry.addLine("Pedro pose fully reset to (0,0,0)");
        telemetry.update();
    }
}
