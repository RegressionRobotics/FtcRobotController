package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@Autonomous(name = "Aimbot Shooter Sweep w/ Continuous Run", group = "Autonomous")
public class Aimbot extends LinearOpMode {

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor shooter, intake;
    private CRServo leftTransfer, rightTransfer;
    private Limelight3A limelight;

    // Tag info
    private static final int APRILTAG_PIPELINE = 4;
    private static final int TAG_LEFT = 20;
    private static final int TAG_RIGHT = 24;

    // Sweep settings
    private static final double TURN_POWER = 0.18;
    private static final long TURN_RIGHT_MS = 800;
    private static final long TURN_LEFT_MS = 1600;
    private static final int SCAN_INTERVAL_MS = 40;

    // Timing
    private static final long RUN_AFTER_TAG_MS = 7000; // keep everything running 7s after tag detection

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // --- Hardware map ---
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");
        leftTransfer = hardwareMap.crservo.get("leftTransfer");
        rightTransfer = hardwareMap.crservo.get("rightTransfer");

        // Reverse left side
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter setup
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // --- Start shooter immediately ---
        shooter.setPower(0.8);
        telemetry.addLine("Shooter running, scanning for AprilTags...");
        telemetry.update();

        boolean found = searchForTag(new int[]{TAG_LEFT, TAG_RIGHT});

        // stop turning once tag found (but shooter keeps going)
        stopAllDriveMotors();

        if (found && opModeIsActive()) {
            telemetry.addLine("Tag found! Running servos + intake...");
            telemetry.update();

            // Servos full power
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);

            // Intake on
            intake.setPower(0.9);

            // Keep everything running 7 seconds after tag found
            long startTime = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - startTime < RUN_AFTER_TAG_MS) {
                sleep(50);
            }
        }

        // Stop all hardware (end of auto)
        stopAllHardware();
        limelight.stop();

        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }

    // Sweep right, then left
    private boolean searchForTag(int[] allowedIds) throws InterruptedException {
        if (scanWhileTurning(allowedIds, TURN_POWER, -TURN_POWER, TURN_RIGHT_MS)) return true;
        if (scanWhileTurning(allowedIds, -TURN_POWER, TURN_POWER, TURN_LEFT_MS)) return true;
        return false;
    }

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

    private void setDrivePowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private void stopAllDriveMotors() {
        setDrivePowers(0, 0, 0, 0);
    }

    private void stopAllHardware() {
        stopAllDriveMotors();
        shooter.setPower(0);
        intake.setPower(0);
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
    }
}
