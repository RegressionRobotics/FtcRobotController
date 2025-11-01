package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "BlueMecanumTeleOp")
public class Me3 extends OpMode {

    // === Hardware ===
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor shooter;
    DcMotor intake;
    CRServo leftTransfer;
    CRServo rightTransfer;
    Servo arjav;

    // === Controllers ===
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // === Pathing ===
    Follower follower;
    private final Pose targetScoringPose = new Pose(57, 88, Math.toRadians(305)); // Target: (57, 88, 305°)
    private boolean isFollowingPath = false;
    private PathChain scoringPath;

    // === Logging ===
    private void log(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    @Override
    public void init() {
        // Motor declarations
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        leftTransfer = hardwareMap.crservo.get("leftTransfer");
        rightTransfer = hardwareMap.crservo.get("rightTransfer");
        arjav = hardwareMap.servo.get("arjav");

        // Pathing setup – assumes odometry starts from auto end (~48,49,305°)
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);  // Max power (-1 to 1 range) for full-speed pathing to target

        log("Status", "INIT: Ready – Press Y to path to scoring pose (57,88,305°), B to cancel");
        log("Target Pose", targetScoringPose);
        telemetry.update();
    }

    @Override
    public void start() {
        // Set initial pose and start teleop drive mode
        follower.setStartingPose(new Pose(48, 49, Math.toRadians(305)));
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update gamepad states
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Update follower (always, for odometry) – keeps encoders tracking even after path ends
        follower.update();

        // Check for Y button press to start pathing (edge detection)
        if (currentGamepad1.y && !previousGamepad1.y && !isFollowingPath) {
            buildPathToScoring();
            follower.followPath(scoringPath, true);  // Async follow
            isFollowingPath = true;
            log("Pathing", "Started to scoring pose!");
        }

        // Stop automated following if B pressed or path done (matches Pedro example)
        if (isFollowingPath && ( (currentGamepad1.b && !previousGamepad1.b) || !follower.isBusy() )) {
            follower.startTeleopDrive();
            isFollowingPath = false;
            log("Pathing", "Stopped/Cancelled – back to manual");
        }

        if (!isFollowingPath) {
            // Manual drive mode using Pedro's setTeleOpDrive (robot-centric, with strafe correction)
            double ly = -gamepad1.left_stick_y;
            double lx = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(ly, lx, rx, true);  // true for robot-centric
        }

        // Non-drive controls (always active, but safe during pathing as they don't move chassis)
        double transfer = gamepad1.right_trigger;
        rightTransfer.setPower(transfer);
        leftTransfer.setPower(transfer);

        // Intake (A button toggle)
        if (currentGamepad1.a && !previousGamepad1.a) {
            gamepad1.rumble(100, 100, 100);  // Short rumble: effect1, effect2, duration_ms
            intake.setPower(1.0);
        }
        if (!currentGamepad1.a && previousGamepad1.a) {
            intake.setPower(0);
        }

        // Shooter (RB toggle)
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            gamepad1.rumble(100, 100, 100);
            shooter.setPower(0.7);
        }
        if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
            shooter.setPower(0);
        }

        // Transfer/Arjav (LB hold)
        if (currentGamepad1.left_bumper) {
            rightTransfer.setPower(1.0);
            leftTransfer.setPower(1.0);
            arjav.setPosition(1);
            gamepad1.rumble(100, 100, 100);
        } else {
            rightTransfer.setPower(0);
            leftTransfer.setPower(0);
            arjav.setPosition(0.6);
        }

        // Telemetry
        Pose currentPose = follower.getPose();
        double normH = Math.toDegrees((currentPose.getHeading() + 2 * Math.PI) % (2 * Math.PI));
        log("Pose (X,Y,H)", String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), normH));
        log("Path Status", isFollowingPath ? "Following (B to cancel)" : "Manual");
        log("Controls", "Y: Start path | B: Cancel");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Zero out motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);
        rightTransfer.setPower(0);
        leftTransfer.setPower(0);
    }

    // === Path Building ===
    private void buildPathToScoring() {
        Pose currentPose = follower.getPose();  // Current from odometry

        scoringPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetScoringPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetScoringPose.getHeading())
                .build();

        log("Path Built", "From current to target");
    }
}