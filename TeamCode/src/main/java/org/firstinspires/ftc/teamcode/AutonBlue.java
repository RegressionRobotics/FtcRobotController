package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Auton Blue", group = "Competition")
public class AutonBlue extends LinearOpMode {

    // Drive Motors
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    // Game Mechanism Motors
    private DcMotorEx shooter;
    private DcMotorEx intake;

    // Servos (CRServo = Continuous Rotation Servo)
    private CRServo leftTransfer;
    private CRServo rightTransfer;

    // Limelight wrapper
    private Limelight3AWrapper limelight;

    private ElapsedTime runtime = new ElapsedTime();

    // Pedro follower with Pinpoint odometry
    private Follower follower;

    // Field Constants for FTC 2024-2025 INTO THE DEEP
    private static final double BLUE_START_X = 12.0;
    private static final double BLUE_START_Y = 12.0;
    private static final double START_HEADING = 0.0;
    private static final double BLUE_GOAL_X = 24.0;
    private static final double BLUE_GOAL_Y = 132.0;

    // Motor Constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Spike Mark positions for blue alliance
    private static final double[][] BLUE_SPIKE_POSITIONS = {
            {36.0, 48.0},  // Left spike
            {36.0, 72.0},  // Center spike
            {36.0, 96.0}   // Right spike
    };

    @Override
    public void runOpMode() {
        initializeHardware();

        // Initialize Limelight
        limelight = new Limelight3AWrapper(hardwareMap, telemetry);
        limelight.setPipeline(0);

        // Initialize Pedro Pathing follower using your Constants file
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "BLUE");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        // Set starting pose
        Pose startPose = new Pose(BLUE_START_X, BLUE_START_Y, Math.toRadians(START_HEADING));
        follower.setStartingPose(startPose);

        if (opModeIsActive()) {
            try {
                // Autonomous sequence
                moveToObeliskView();
                int targetSpike = readObeliskAprilTag();
                telemetry.addData("Target Spike", targetSpike);
                telemetry.update();
                sleep(500);

                navigateToSpikeMark(targetSpike);
                collectArtifacts();
                moveToShootingPosition();
                aimAndShoot();
                parkRobot();

                telemetry.addData("Status", "Autonomous Complete");
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
            }
        }
    }

    private void initializeHardware() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Set motor directions (adjust based on your robot configuration)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize game mechanism motors
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize continuous rotation servos
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");

        // Stop servos initially
        leftTransfer.setPower(0.0);
        rightTransfer.setPower(0.0);
    }

    private void moveToObeliskView() {
        telemetry.addData("Status", "Moving to Obelisk View");
        telemetry.update();

        Pose targetPose = new Pose(36.0, 48.0, Math.toRadians(START_HEADING));

        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        sleep(500);
    }

    private int readObeliskAprilTag() {
        telemetry.addData("Status", "Reading Obelisk AprilTag");
        telemetry.update();

        limelight.updateTracking();
        int id = limelight.getTargetAprilTagId();

        telemetry.addData("AprilTag ID", id);
        telemetry.update();

        // AprilTag mapping for 2024-2025 season
        if (id == 21) return 0;  // Left spike
        if (id == 22) return 1;  // Center spike
        if (id == 23) return 2;  // Right spike

        return 1; // Default to center spike
    }

    private void navigateToSpikeMark(int spikeIndex) {
        telemetry.addData("Status", "Navigating to Spike " + spikeIndex);
        telemetry.update();

        double targetX = BLUE_SPIKE_POSITIONS[spikeIndex][0];
        double targetY = BLUE_SPIKE_POSITIONS[spikeIndex][1];

        Pose targetPose = new Pose(targetX, targetY, Math.toRadians(START_HEADING));

        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    private void collectArtifacts() {
        telemetry.addData("Status", "Collecting Artifacts");
        telemetry.update();

        // Lower transfer mechanism
        leftTransfer.setPower(0.5);
        rightTransfer.setPower(-0.5);  // Opposite direction for mirrored servo
        sleep(300);

        // Run intake
        intake.setPower(1.0);
        sleep(1000);

        // Raise transfer mechanism
        leftTransfer.setPower(0.0);
        rightTransfer.setPower(0.0);
        intake.setPower(0.0);
    }

    private void moveToShootingPosition() {
        telemetry.addData("Status", "Moving to Shooting Position");
        telemetry.update();

        Pose targetPose = new Pose(BLUE_GOAL_X, BLUE_GOAL_Y, Math.toRadians(START_HEADING));

        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    private void aimAndShoot() {
        telemetry.addData("Status", "Aiming and Shooting");
        telemetry.update();

        // Switch to shooting pipeline
        limelight.setPipeline(1);
        sleep(500);
        limelight.updateTracking();

        // Calculate shooter power (simplified)
        double shooterPower = 0.7;

        // Spin up shooter
        shooter.setPower(shooterPower);
        sleep(1500);

        // Feed samples into shooter
        leftTransfer.setPower(0.7);
        rightTransfer.setPower(-0.7);  // Opposite direction
        intake.setPower(0.5);
        sleep(2000);

        // Stop all mechanisms
        shooter.setPower(0.0);
        intake.setPower(0.0);
        leftTransfer.setPower(0.0);
        rightTransfer.setPower(0.0);
    }

    private void parkRobot() {
        telemetry.addData("Status", "Parking");
        telemetry.update();

        Pose parkPose = new Pose(12.0, 12.0, Math.toRadians(START_HEADING));

        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), parkPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        telemetry.addData("Status", "Parked Successfully");
        telemetry.update();
    }
}