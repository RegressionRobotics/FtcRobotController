package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AimBot")
public class MecanumTeleOpWithAimbot extends LinearOpMode {

    // Configurable constants
    private static final double SHOOTER_POWER = 0.8; // Fallback power
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_MAX_POWER = 1.0;
    private static final double DRIVE_SPEED_MULTIPLIER = 0.7;
    private static final double ARJAV_OPEN = 1.0;
    private static final double ARJAV_CLOSED = 0.5; // Corrected declaration
    private static final int RUMBLE_DURATION_MS = 1000;

    // Limelight constants (commented out until integrated)
    // private static final String LIMELIGHT_NAME = "limelight";
    // private static final int TARGET_TAG_ID = 1;
    // private static final double TX_TOLERANCE = 2.0;
    // private static final double STRAFE_GAIN = 0.3;
    private static final double CAMERA_PITCH_DEG = 15.0; // Camera angle up
    private static final double CAMERA_HEIGHT_IN = 6.0; // Camera height
    private static final double TAG_HEIGHT_IN = 12.0; // Goal tag height
    private static final double A_RPM = 2000.0; // RPM = a / dist + b (calibrate)
    private static final double B_RPM = 500.0; // Base RPM
    private static final double MAX_RPM = 6000.0; // Safety clamp
    private static final long AIMBOT_TIMEOUT_MS = 5000; // 5s timeout

    // Aimbot state
    private boolean aimbotActive = false;
    private long aimbotStartTime = 0;

    @Override
    public void runOpMode() {
        // Motor declarations
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left-side motors for mecanum
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo declarations
        CRServo leftTransfer = hardwareMap.crservo.get("leftTransfer");
        CRServo rightTransfer = hardwareMap.crservo.get("rightTransfer");
        Servo arjav = hardwareMap.servo.get("arjav");
        // rightTransfer.setDirection(Servo.Direction.REVERSE); // Uncomment if needed

        // Gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        // Initialize telemetry
        telemetry.addData("Status", "Initialized - Integrate Limelight library");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update gamepad states
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // Aimbot trigger: Toggle with A button (simulated)
            if (currentGamepad1.a && !previousGamepad1.a) {
                aimbotActive = !aimbotActive;
                if (aimbotActive) {
                    aimbotStartTime = System.currentTimeMillis();
                    telemetry.addData("Aimbot", "Starting alignment... (Simulated)");
                } else {
                    telemetry.addData("Aimbot", "Cancelled");
                    shooter.setPower(0);
                }
                gamepad1.rumble(RUMBLE_DURATION_MS);
            }

            double y = 0, x = 0, rx = 0; // Default for aimbot
            double speedMultiplier = DRIVE_SPEED_MULTIPLIER * (1.0 - 0.5 * currentGamepad1.left_trigger);

            if (aimbotActive) {
                // Simulated aimbot (replace with Limelight logic)
                // LimelightHelpers.LimelightResults result = LimelightHelpers.getLatestResults(LIMELIGHT_NAME);
                // if (result != null && result.hasValidTargets()) {
                //     boolean foundTarget = false;
                //     for (LimelightHelpers.LimelightTarget_Fiducial target : result.targets_Fiducials) {
                //         if ((int) target.fiducialID == TARGET_TAG_ID) {
                //             double tx = target.getTx();
                //             double ty = target.getTy();
                //             telemetry.addData("TX", "%.1f°", tx);
                //             telemetry.addData("TY", "%.1f°", ty);

                //             if (Math.abs(tx) < TX_TOLERANCE) {
                //                 double distIn = calculateDistance(ty);
                //                 double rpm = Math.max(0, Math.min(MAX_RPM, A_RPM / distIn + B_RPM));
                //                 shooter.setPower(rpm / MAX_RPM); // Scaled power
                //                 telemetry.addData("Distance", "%.1f in", distIn);
                //                 telemetry.addData("Target RPM", "%.0f", rpm);
                //                 aimbotActive = false;
                //                 gamepad1.rumble(500);
                //             } else {
                //                 x = -tx * STRAFE_GAIN;
                //                 telemetry.addData("Strafing", "X: %.2f", x);
                //             }
                //             foundTarget = true;
                //             break;
                //         }
                //     }
                //     if (!foundTarget) {
                //         telemetry.addData("Target", "No matching tag ID");
                //         aimbotActive = false;
                //         shooter.setPower(0);
                //     }
                // } else {
                //     telemetry.addData("Limelight", "No valid targets");
                //     aimbotActive = false;
                //     shooter.setPower(0);
                // }

                // Simulated distance for testing
                double simulatedDistance = 60.0; // Hardcode a distance (e.g., 60 inches)
                double simulatedRpm = Math.max(0, Math.min(MAX_RPM, A_RPM / simulatedDistance + B_RPM));
                shooter.setPower(simulatedRpm / MAX_RPM);
                telemetry.addData("Simulated Distance", "%.1f in", simulatedDistance);
                telemetry.addData("Simulated RPM", "%.0f", simulatedRpm);

                // Timeout
                if (System.currentTimeMillis() - aimbotStartTime > AIMBOT_TIMEOUT_MS) {
                    aimbotActive = false;
                    shooter.setPower(0);
                    telemetry.addData("Aimbot", "Timeout");
                }
            } else {
                // Manual mecanum drive
                y = -currentGamepad1.left_stick_y;
                x = currentGamepad1.left_stick_x * 1.1; // Strafing correction
                rx = currentGamepad1.right_stick_x;
            }

            // Mecanum power calculations
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * speedMultiplier;
            double backLeftPower = (y - x + rx) / denominator * speedMultiplier;
            double frontRightPower = (y - x - rx) / denominator * speedMultiplier;
            double backRightPower = (y + x - rx) / denominator * speedMultiplier;

            // Set drive powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Transfer control (right trigger)
            double transferPower = currentGamepad1.right_trigger * TRANSFER_MAX_POWER;
            leftTransfer.setPower(transferPower);
            rightTransfer.setPower(transferPower);

            // Intake toggle (B button)
            if (currentGamepad1.b && !previousGamepad1.b) {
                intake.setPower(intake.getPower() == 0 ? INTAKE_POWER : 0);
                gamepad1.rumble(RUMBLE_DURATION_MS);
            }

            // Shooter manual toggle (right bumper)
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                shooter.setPower(shooter.getPower() == 0 ? SHOOTER_POWER : 0);
                gamepad1.rumble(RUMBLE_DURATION_MS);
            }

            // Transfer and arjav (left bumper)
            if (currentGamepad1.left_bumper) {
                leftTransfer.setPower(TRANSFER_MAX_POWER);
                rightTransfer.setPower(TRANSFER_MAX_POWER);
                arjav.setPosition(ARJAV_OPEN);
                gamepad1.rumble(RUMBLE_DURATION_MS);
            } else if (!currentGamepad1.left_bumper) {
                leftTransfer.setPower(0);
                rightTransfer.setPower(0);
                arjav.setPosition(ARJAV_CLOSED);
            }

            // Telemetry
            telemetry.addData("Aimbot Active", aimbotActive);
            telemetry.addData("Drive", "FL: %.2f, BL: %.2f, FR: %.2f, BR: %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Intake", "Power: %.2f", intake.getPower());
            telemetry.addData("Shooter", "Power: %.2f", shooter.getPower());
            telemetry.update();
        }
    }

    /**
     * Calculate distance to tag using vertical offset (ty).
     * Note: Currently unused due to commented-out Limelight code.
     */
    private double calculateDistance(double ty) {
        double tyRad = Math.toRadians(ty);
        double pitchRad = Math.toRadians(CAMERA_PITCH_DEG);
        double angleToTag = pitchRad + tyRad;
        double heightDiff = TAG_HEIGHT_IN - CAMERA_HEIGHT_IN;
        return Math.abs(heightDiff / Math.tan(angleToTag));
    }
}