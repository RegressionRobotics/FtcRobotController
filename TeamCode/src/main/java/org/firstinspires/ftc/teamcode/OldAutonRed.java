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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;

@Autonomous(name = "Spike Marks - Red Alliance", group = "Autonomous")
public class OldAutonRed extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intake, shooter;
    private CRServo leftTransfer, rightTransfer;
    private Servo arjav;

    private static final int APRILTAG_PIPELINE = 5;
    private static final int DEBUG_DELAY_MS = 100;

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
        follower.setMaxPower(0.75);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Initialized — Waiting for AprilTag...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        int detectedTagId = detectAprilTag();
        telemetry.addData("Detected Tag", detectedTagId);
        telemetry.update();

        sleep(2000); // debug pause

        // --- Main autonomous logic ---
        switch (detectedTagId) {
            case 21:
                executeRedTag21();
                break;
            case 22:
                executeRedTag22();
                break;
            case 23:
                executeRedTag23();
                break;
            default:
                executeRedDefault();
                break;
        }

        telemetry.addLine("Autonomous Complete ✅");
        telemetry.update();
    }

    // --- Red Alliance specific sequences ---

    private void executeRedTag21() throws InterruptedException {
        moveBackwardNoIntake(37);
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(4);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(38.25);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise55();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startAutoShooterSequence();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(25);
    }

    private void executeRedTag22() throws InterruptedException {
        moveBackwardNoIntake(61);
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(4);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(11.5);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise55();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startAutoShooterSequence();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(25);
    }

    private void executeRedTag23() throws InterruptedException {
        moveBackwardNoIntake(87);
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(26);
        sleep(DEBUG_DELAY_MS);

        moveForwardHeadingRelative(4);
        sleep(DEBUG_DELAY_MS);

        moveBackwardNoIntake(29);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise90();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(11);
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnClockwise55();
        sleep(DEBUG_DELAY_MS);

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        startAutoShooterSequence();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise45();

        resetPedroPose();
        sleep(DEBUG_DELAY_MS);

        moveForwardNoIntake(25);
    }

    private void executeRedDefault() throws InterruptedException {
        moveBackwardNoIntake(33.5);
        sleep(DEBUG_DELAY_MS);

        pivotTurnAnticlockwise90();
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

        pivotTurnClockwise90();
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

    private void pivotTurnClockwise55() throws InterruptedException {
        double power = 0.4;
        long duration = 265;
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
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


        // --- AUTONOMOUS SEQUENCE ---
        if (opModeIsActive()) {

            // === Step 0: Activate Arjav servo ===
            telemetry.addData("Status", "Step 0: Setting Arjav servo to position 1");
            telemetry.update();
            arjav.setPosition(1.0);
            sleep(250); // small delay for servo to move

            // === Step 1: Power up the shooter motor ===
            telemetry.addData("Status", "Step 1: Spinning up shooter...");
            telemetry.update();
            shooter.setPower(0.85);

            // Wait 4 seconds for shooter to reach speed
            sleep(1000);

            // === Step 2: Feed first ball with transfer servos ===
            telemetry.addData("Status", "Step 2: Feeding first ball...");
            telemetry.update();
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);
            sleep(100);

            // === Step 3: Run intake briefly to load second ball ===
            telemetry.addData("Status", "Step 3: Loading second ball...");
            telemetry.update();
            intake.setPower(0.85);
            sleep(250);


            // Keep shooter + transfers running for a while
            telemetry.addData("Status", "Step 4: Running shooter...");
            telemetry.update();

            sleep(5000);

            // === Step 5: Stop everything ===
            telemetry.addData("Status", "Step 5: Stopping all hardware.");
            telemetry.update();
            shooter.setPower(0);
            leftTransfer.setPower(0);
            rightTransfer.setPower(0);



        }
    }
}
