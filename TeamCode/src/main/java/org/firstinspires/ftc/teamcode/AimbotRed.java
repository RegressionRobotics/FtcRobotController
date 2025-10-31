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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Aimbot Red - Tag 24 (Fixed Centering + Pedro Resets)", group = "Autonomous")
public class AimbotRed extends LinearOpMode {

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private Limelight3A limelight;
    private Follower follower;

    // Tag info - RED ALLIANCE
    private static final int APRILTAG_PIPELINE = 4;
    private static final int TARGET_TAG = 24;

    // Sweep settings
    private static final double TURN_POWER = 0.18;
    private static final long TURN_RIGHT_MS = 800;
    private static final long TURN_LEFT_MS = 1600;
    private static final int SCAN_INTERVAL_MS = 40;

    // Centering & Distance settings
    private static final double TARGET_DISTANCE_IN = 70.0;  // inches
    private static final double DISTANCE_TOLERANCE_IN = 1.0;
    private static final double CENTER_TOLERANCE_DEG = 1.5;  // Tighter for better centering
    private static final double TURN_POWER_CENTER = 0.15;  // Slightly higher for quicker response

    // Distance calibration - TUNE BASED ON TELEMETRY AT KNOWN DISTANCE
    private static final double DISTANCE_SCALE_SQRT_IN = 67.0;  // Update: 70 * sqrt(ta %)

    // Pathing settings
    private static final double FOLLOWER_MAX_POWER = 0.3;  // Gentle for precise adjust
    private static final int DEBUG_DELAY_MS = 100;
    private static final int CENTER_TIMEOUT_MS = 8000;  // Longer for reliable centering
    private static final double PROPORTIONAL_GAIN_TURN = 50.0;  // ms per degree (tuned higher)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // --- Hardware map ---
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Directions (from auton examples)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Pathing follower (Pinpoint/Pedro)
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(FOLLOWER_MAX_POWER);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Wait for Limelight stabilize
        sleep(2000);

        telemetry.addLine("Ready - Red Alliance, Tag 24");
        telemetry.addLine("Sweep -> Reset Pedro -> Center (w/ resets) -> Measure & Move to 70in");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        telemetry.addLine("Scanning for Tag 24 (right to left sweep)...");
        telemetry.update();

        boolean found = searchForTag();

        if (found && opModeIsActive()) {
            telemetry.addLine("Tag 24 found! Resetting Pedro...");
            resetPedroPose();  // Reset after sweep

            telemetry.addLine("Centering on tag...");
            centerOnTag();

            if (opModeIsActive()) {
                telemetry.addLine("Centered! Measuring distance & adjusting to 70in...");
                adjustToTargetDistance();
            }
        } else {
            telemetry.addLine("Tag 24 not found!");
        }

        // Stop drive
        stopAllDriveMotors();
        follower.breakFollowing();

        limelight.stop();
        telemetry.addLine("Complete - Positioned at 70 inches from Tag 24");
        telemetry.update();
    }

    // Sweep right, then left to find tag
    private boolean searchForTag() throws InterruptedException {
        if (scanWhileTurning(TURN_POWER, -TURN_POWER, TURN_RIGHT_MS)) return true;  // Turn right
        if (scanWhileTurning(-TURN_POWER, TURN_POWER, TURN_LEFT_MS)) return true;   // Turn left
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
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        int id = (int) f.getFiducialId();
                        if (id == TARGET_TAG) {
                            stopAllDriveMotors();
                            telemetry.addData("Detected Tag", id);
                            telemetry.update();
                            return true;
                        }
                    }
                }
            }
            sleep(SCAN_INTERVAL_MS);
        }
        stopAllDriveMotors();
        return false;
    }

    // Improved centering: Proportional turns with Pedro resets after each
    private void centerOnTag() throws InterruptedException {
        long centerStart = System.currentTimeMillis();
        int centerIterations = 0;
        while (opModeIsActive() && (System.currentTimeMillis() - centerStart < CENTER_TIMEOUT_MS)) {
            centerIterations++;
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                sleep(50);
                continue;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                sleep(50);
                continue;
            }

            LLResultTypes.FiducialResult tag = null;
            for (LLResultTypes.FiducialResult f : fiducials) {
                if ((int) f.getFiducialId() == TARGET_TAG) {
                    tag = f;
                    break;
                }
            }
            if (tag == null) {
                sleep(50);
                continue;
            }

            double tx = tag.getTargetXDegrees();
            telemetry.addData("Center Loop %d: TX", "%.2f° (tol: %.1f°)", centerIterations, tx, CENTER_TOLERANCE_DEG);
            telemetry.update();

            if (Math.abs(tx) < CENTER_TOLERANCE_DEG) {
                telemetry.addLine("Centered on tag!");
                resetPedroPose();  // Final reset after centering
                telemetry.update();
                break;
            }

            // Proportional turn toward tag
            double turnPower = TURN_POWER_CENTER;
            long turnTimeMs = (long) (Math.abs(tx) * PROPORTIONAL_GAIN_TURN);  // Tuned for better response
            turnTimeMs = Math.max(turnTimeMs, 100);  // Min time
            turnTimeMs = Math.min(turnTimeMs, 600);  // Max time

            if (tx > 0) {
                // Tag right: turn RIGHT (like pivotTurnClockwise in auton)
                setDrivePowers(turnPower, turnPower, -turnPower, -turnPower);
                telemetry.addData("Turning", "RIGHT %.1f° (time: %d ms)", tx, turnTimeMs);
            } else {
                // Tag left: turn LEFT (like pivotTurnAnticlockwise in auton)
                setDrivePowers(-turnPower, -turnPower, turnPower, turnPower);
                telemetry.addData("Turning", "LEFT %.1f° (time: %d ms)", Math.abs(tx), turnTimeMs);
            }
            telemetry.update();

            sleep(turnTimeMs);
            stopAllDriveMotors();

            // Reset Pedro after EVERY turn (as per request)
            resetPedroPose();
            sleep(50);  // Brief pause
        }

        if (System.currentTimeMillis() - centerStart >= CENTER_TIMEOUT_MS) {
            telemetry.addLine("Centering timeout - proceeding anyway");
        }
    }

    // Measure distance once, compute delta, move exactly that amount using pathing
    private void adjustToTargetDistance() throws InterruptedException {
        // Get final measurement (after centering)
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid measurement - skipping adjust");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addLine("No fiducials - skipping adjust");
            return;
        }

        LLResultTypes.FiducialResult tag = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if ((int) f.getFiducialId() == TARGET_TAG) {
                tag = f;
                break;
            }
        }
        if (tag == null) {
            telemetry.addLine("Tag 24 missing - skipping adjust");
            return;
        }

        double targetArea = result.getTa();
        double distanceIn = getDistanceFromTagInches(targetArea);
        double delta = distanceIn - TARGET_DISTANCE_IN;

        // Calibration helper
        double suggestedScale = 70.0 * Math.sqrt(targetArea);
        telemetry.addData("Measured ta %%", "%.3f", targetArea);
        telemetry.addData("Current Dist", "%.1f in", distanceIn);
        telemetry.addData("Delta to 70in", "%.1f in (tol: %.1f)", delta, DISTANCE_TOLERANCE_IN);
        telemetry.addData("CALIB Suggest SCALE", "%.1f (current: %.1f)", suggestedScale, DISTANCE_SCALE_SQRT_IN);
        telemetry.update();

        if (Math.abs(delta) < DISTANCE_TOLERANCE_IN) {
            telemetry.addLine("Already within target distance!");
            return;
        }

        // Reset Pedro before distance move
        resetPedroPose();

        // Move the exact delta (forward if too far, backward if too close)
        if (delta > 0) {
            // Too far: move FORWARD |delta| inches (toward tag, like moveForwardHeadingRelative)
            telemetry.addData("Adjusting", "FORWARD %.1f in to close gap", delta);
            moveForwardHeadingRelative(delta);
        } else {
            // Too close: move BACKWARD |delta| inches (away from tag, like moveBackwardNoIntake but no intake)
            telemetry.addData("Adjusting", "BACKWARD %.1f in to extend", Math.abs(delta));
            moveBackwardHeadingRelative(Math.abs(delta));
        }
        telemetry.update();

        // Reset Pedro after distance move
        resetPedroPose();
        sleep(DEBUG_DELAY_MS);
    }

    /**
     * Distance in inches: SCALE / sqrt(targetArea %)
     */
    private double getDistanceFromTagInches(double targetArea) {
        if (targetArea < 0.01) return 999.0;
        return DISTANCE_SCALE_SQRT_IN / Math.sqrt(targetArea);
    }

    // === Pathing Movement Methods (Adapted from SpikeMarkBlue/TestAuton - No Intake) ===

    private void moveForwardHeadingRelative(double inches) throws InterruptedException {
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

    private void moveBackwardHeadingRelative(double inches) throws InterruptedException {
        Pose startPose = follower.getPose();
        double headingRad = Math.toRadians(startPose.getHeading());
        Pose targetPose = new Pose(
                startPose.getX() - inches * Math.cos(headingRad),
                startPose.getY() - inches * Math.sin(headingRad),
                startPose.getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
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