package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;

@TeleOp(name = "MecanumTeleOpWithAimbot")
public class MecanumTeleopWithAimbot extends LinearOpMode {

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor shooter, intake;
    private CRServo leftTransfer, rightTransfer;
    private Servo arjav;
    private Limelight3A limelight;

    // AprilTag settings
    private static final int APRILTAG_PIPELINE = 4;
    private static final int TAG_LEFT = 20;
    private static final int TAG_RIGHT = 24;
    private static final double TURN_POWER = 0.24;
    private static final long TURN_RIGHT_MS = 700;
    private static final long TURN_LEFT_MS = 1400;
    private static final int SCAN_INTERVAL_MS = 40;
    private static final long RUN_AFTER_TAG_MS = 5000; // Run shooter/servos/intake for 7s after tag detection
    private static final double SHOOTER_POWER = 1;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");
        leftTransfer = hardwareMap.crservo.get("leftTransfer");
        rightTransfer = hardwareMap.crservo.get("rightTransfer");
        arjav = hardwareMap.servo.get("arjav");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Reverse left-side motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter setup
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Limelight
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update gamepad states
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Mecanum drive control
            double y = -gamepad1.left_stick_y; // Reversed Y stick
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Calculate motor powers
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply drive powers unless auto-aiming is active
            if (!currentGamepad1.y) {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

            // Transfer servo control with right trigger
            double transfer = gamepad1.right_trigger;
            rightTransfer.setPower(transfer);
            leftTransfer.setPower(transfer);

            // Intake control (A button toggle)
            if (currentGamepad1.a && !previousGamepad1.a) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                intake.setPower(1.0);
            }
            if (!currentGamepad1.a && previousGamepad1.a) {
                intake.setPower(0);
                gamepad1.stopRumble();
            }

            // Shooter control (Right bumper toggle)
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                shooter.setPower(SHOOTER_POWER);
            }
            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                shooter.setPower(0);
                gamepad1.stopRumble();
            }

            // Transfer and arjav servo control (Left bumper)
            if (currentGamepad1.left_bumper) {
                rightTransfer.setPower(1.0);
                leftTransfer.setPower(1.0);
                arjav.setPosition(1.0);
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                rightTransfer.setPower(0);
                leftTransfer.setPower(0);
                arjav.setPosition(0.5);
                gamepad1.stopRumble();
            }

            // Auto-aimer (Y button toggle)
            if (currentGamepad1.y && !previousGamepad1.y) {
                telemetry.addLine("Auto-aim activated, scanning for AprilTags...");
                telemetry.update();
                shooter.setPower(SHOOTER_POWER); // Start shooter
                boolean found = false;
                try {
                    found = searchForTag(new int[]{TAG_LEFT, TAG_RIGHT});
                } catch (InterruptedException e) {
                    telemetry.addLine("Auto-aim interrupted");
                    telemetry.update();
                }
                stopAllDriveMotors();
                if (found) {
                    telemetry.addLine("Tag found! Running servos + intake...");
                    telemetry.update();
                    leftTransfer.setPower(1.0);
                    rightTransfer.setPower(1.0);
                    intake.setPower(0.9);
                    arjav.setPosition(1);
                    long startTime = System.currentTimeMillis();
                    while (opModeIsActive() && System.currentTimeMillis() - startTime < RUN_AFTER_TAG_MS) {
                        sleep(50);
                    }
                    // Stop servos and intake after 7s
                    leftTransfer.setPower(0);
                    arjav.setPosition(0.5);
                    rightTransfer.setPower(0);
                    intake.setPower(0);
                }
                shooter.setPower(0); // Stop shooter after aiming sequence
                telemetry.addLine("Auto-aim sequence complete");
                telemetry.update();
            }
        }

        // Cleanup
        stopAllHardware();
        limelight.stop();
        telemetry.addLine("TeleOp complete");
        telemetry.update();
    }

    // Sweep right, then left to search for AprilTags
    private boolean searchForTag(int[] allowedIds) throws InterruptedException {
        if (scanWhileTurning(allowedIds, TURN_POWER, -TURN_POWER, TURN_RIGHT_MS)) return true;
        if (scanWhileTurning(allowedIds, -TURN_POWER, TURN_POWER, TURN_LEFT_MS)) return true;
        return false;
    }

    // Scan for tags while turning
    private boolean scanWhileTurning(int[] allowedIds, double leftPower, double rightPower, long durationMs) throws InterruptedException {
        long start = System.currentTimeMillis();
        setDrivePowers(leftPower, leftPower, rightPower, rightPower);
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        int id = (int) f.getFiducialId();
                        for (int allowed : allowedIds) {
                            if (id == allowed) {
                                stopAllDriveMotors();
                                telemetry.addData("Detected Tag", id);
                                telemetry.update();
                                return true;
                            }
                        }
                    }
                }
            }
            sleep(SCAN_INTERVAL_MS);
        }
        stopAllDriveMotors();
        return false;
    }

    // Set drive motor powers
    private void setDrivePowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    // Stop all drive motors
    private void stopAllDriveMotors() {
        setDrivePowers(0, 0, 0, 0);
    }

    // Stop all hardware
    private void stopAllHardware() {
        stopAllDriveMotors();
        shooter.setPower(0);
        intake.setPower(0);
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
        arjav.setPosition(0.5);
    }
}