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
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

@Autonomous(name = "Spike Marks - Blue Alliance", group = "Autonomous")
public class SpikeMarkBlue extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intake, shooter;
    private CRServo leftTransfer, rightTransfer;
    private Servo arjav;

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
        arjav = hardwareMap.get(Servo.class, "arjav");

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
                executeBlueTag21();
                break;
            case 22:
                executeBlueTag22();
                break;
            case 23:
                executeBlueTag23();
                break;
            default:
                executeBlueDefault();
                break;
        }

        telemetry.addLine("Autonomous Complete ✅");
        telemetry.update();
    }

    // --- Blue Alliance specific sequences ---

    private void executeBlueTag21() throws InterruptedException {
        moveBackwardNoIntake(39);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
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

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(38.25);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startAutoShooterSequence();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise45();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(25);
    }

    private void executeBlueTag22() throws InterruptedException {
        moveBackwardNoIntake(63);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
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

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(11.5);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startAutoShooterSequence();


        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise45();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(25);
    }

    private void executeBlueTag23() throws InterruptedException {
        moveBackwardNoIntake(89);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
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

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(11);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startAutoShooterSequence();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise45();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(25);
    }

    private void executeBlueDefault() throws InterruptedException {
        moveBackwardNoIntake(33.5);
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
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

    // === Helper Methods ===

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

    private void pivotTurnClockwise90() throws InterruptedException {
        double power = 0.4;
        long duration = 490;
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        sleep(duration);
        stopAllDriveMotors();
    }

    private void pivotTurnClockwise45() throws InterruptedException {
        double power = 0.4;
        long duration = 245;
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

    private void resetPedroPose() {
        follower.breakFollowing();
        follower.setStartingPose(new Pose(0, 0, 0));
        telemetry.addLine("Pedro pose fully reset to (0,0,0)");
        telemetry.update();
    }

    // === New Autonomous Shooter Sequence ===
    private void startAutoShooterSequence() throws InterruptedException {


        // --- INITIALIZATION ---
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arjav.setPosition(1);


        // --- AUTONOMOUS SEQUENCE ---
        if (opModeIsActive()) {

            // Step 1: Power up the shooter motor

            telemetry.addData("Status", "Step 1: Spinning up shooter...");
            telemetry.update();
            shooter.setPower(0.8);

            // Step 2: Wait 2 seconds for the motor to get to speed
            sleep(2000);

            // Step 3: Turn on the transfer servos to feed
            telemetry.addData("Status", "Step 2: Feeding into shooter...");
            telemetry.update();
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);

            // Step 4: Wait 5 seconds while everything is running
            sleep(5000);

            // Step 5: Stop all motors and servos
            telemetry.addData("Status", "Step 3: Stopping all hardware.");
            telemetry.update();
            shooter.setPower(0);
            leftTransfer.setPower(0);
            rightTransfer.setPower(0);

        }
    }
}
