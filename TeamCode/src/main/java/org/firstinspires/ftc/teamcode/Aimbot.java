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

@Autonomous(name = "Aimbot Red - Tag 24 (Ultra Debug Centering)", group = "Autonomous")
public class Aimbot extends LinearOpMode {

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
    private static final double CENTER_TOLERANCE_DEG = 2.0;  // Wider for debug
    private static final double TURN_POWER_CENTER = 0.20;  // Higher for visible movement

    // Distance calibration - TUNE BASED ON TELEMETRY AT KNOWN DISTANCE
    private static final double DISTANCE_SCALE_SQRT_IN = 67.0;  // Update: 70 * sqrt(ta %)

    // Pathing settings
    private static final double FOLLOWER_MAX_POWER = 0.3;  // Gentle for precise adjust
    private static final int DEBUG_DELAY_MS = 200;  // Longer pauses for watching
    private static final int CENTER_TIMEOUT_MS = 10000;  // 10s
    private static final double PROPORTIONAL_GAIN_TURN = 60.0;  // ms per degree (higher for more turn)

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
        telemetry.addLine("Waiting 2s for Limelight...");
        telemetry.update();
        sleep(2000);

        telemetry.addLine("Ready - Red Alliance, Tag 24 (ULTRA DEBUG)");
        telemetry.addLine("Watch telemetry closely after detection!");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        telemetry.addLine("=== SCAN PHASE START ===");
        telemetry.update();

        boolean found = searchForTag();

        if (found && opModeIsActive()) {
            telemetry.addLine("=== TAG FOUND! Starting post-detection ===");
            telemetry.update();

            telemetry.addLine("Resetting Pedro after sweep...");
            resetPedroPose();
            telemetry.addLine("Pedro reset complete.");

            telemetry.addLine("=== CENTER PHASE START ===");
            telemetry.addLine("If no 'Loop X: TX' logs, Limelight data issue!");
            centerOnTag();

            if (opModeIsActive()) {
                telemetry.addLine("=== DISTANCE ADJUST PHASE START ===");
                adjustToTargetDistance();
            }
        } else {
            telemetry.addLine("Tag 24 not found! Check pipeline/lighting/tag visibility.");
        }

        // Stop drive
        stopAllDriveMotors();
        follower.breakFollowing();

        limelight.stop();
        telemetry.addLine("=== COMPLETE ===");
        telemetry.update();
    }

    // Sweep right, then left to find tag - with extra logs
    private boolean searchForTag() throws InterruptedException {
        telemetry.addLine("Sweep 1: RIGHT TURN");
        if (scanWhileTurning(TURN_POWER, -TURN_POWER, TURN_RIGHT_MS)) return true;
        telemetry.addLine("Sweep 2: LEFT TURN");
        if (scanWhileTurning(-TURN_POWER, TURN_POWER, TURN_LEFT_MS)) return true;
        return false;
    }

    private boolean scanWhileTurning(double leftPower, double rightPower, long durationMs) throws InterruptedException {
        long start = System.currentTimeMillis();
        setDrivePowers(leftPower, leftPower, rightPower, rightPower);
        telemetry.addData("Turning | Power L:%.2f R:%.2f", leftPower, rightPower);
        telemetry.update();

        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            LLResult result = limelight.getLatestResult();
            boolean valid = (result != null && result.isValid());
            if (valid) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        int id = (int) f.getFiducialId();
                        telemetry.addData("Scan: Saw Tag", id);
                        if (id == TARGET_TAG) {
                            stopAllDriveMotors();
                            telemetry.addLine("*** TAG 24 DETECTED - STOPPING SCAN ***");
                            telemetry.update();
                            sleep(500);  // Pause to settle
                            return true;
                        }
                    }
                } else {
                    telemetry.addData("Scan: Valid but", "No fiducials");
                }
            } else {
                telemetry.addData("Scan: Result", valid ? "Valid" : "INVALID/NULL");
            }
            telemetry.update();
            sleep(SCAN_INTERVAL_MS);
        }
        stopAllDriveMotors();
        return false;
    }

    // ULTRA DEBUG Centering: More logs, initial read, Pedro after each
    private void centerOnTag() throws InterruptedException {
        long centerStart = System.currentTimeMillis();
        int centerIterations = 0;

        // Initial read before loop
        LLResult initialResult = limelight.getLatestResult();
        double initialTx = 999;
        boolean initialValid = (initialResult != null && initialResult.isValid());
        if (initialValid) {
            List<LLResultTypes.FiducialResult> initialFid = initialResult.getFiducialResults();
            if (initialFid != null && !initialFid.isEmpty()) {
                for (LLResultTypes.FiducialResult f : initialFid) {
                    if ((int) f.getFiducialId() == TARGET_TAG) {
                        initialTx = f.getTargetXDegrees();
                        break;
                    }
                }
            }
        }
        telemetry.addData("INITIAL READ: Valid?", initialValid);
        telemetry.addData("INITIAL TX", "%.2f°", initialTx);
        telemetry.update();

        if (Math.abs(initialTx) < CENTER_TOLERANCE_DEG) {
            telemetry.addLine("Already centered! Skipping turns.");
            resetPedroPose();
            return;
        }

        while (opModeIsActive() && (System.currentTimeMillis() - centerStart < CENTER_TIMEOUT_MS)) {
            centerIterations++;
            telemetry.addData("=== CENTER LOOP %d START ===", centerIterations);
            telemetry.update();

            LLResult result = limelight.getLatestResult();
            boolean valid = (result != null && result.isValid());
            telemetry.addData("Loop %d: Result Valid?", valid);
            if (!valid) {
                telemetry.addLine("Loop %d: INVALID RESULT - RETRY", centerIterations);
                sleep(100);
                continue;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            int numFid = (fiducials != null) ? fiducials.size() : 0;
            telemetry.addData("Loop %d: Num Fiducials", numFid);
            if (numFid == 0) {
                telemetry.addLine("Loop %d: NO FIDUCIALS - RETRY", centerIterations);
                sleep(100);
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
                telemetry.addLine("Loop %d: TAG 24 NOT FOUND - RETRY", centerIterations);
                sleep(100);
                continue;
            }

            double tx = tag.getTargetXDegrees();
            double ta = result.getTa();
            telemetry.addData("Loop %d: TX", "%.2f° (tol: %.1f°)", centerIterations, tx, CENTER_TOLERANCE_DEG);
            telemetry.addData("Loop %d: ta %%", "%.3f", centerIterations, ta);
            telemetry.update();

            if (Math.abs(tx) < CENTER_TOLERANCE_DEG) {
                telemetry.addLine("*** CENTERED! TX within tol ***");
                resetPedroPose();  // Final reset
                telemetry.update();
                break;
            }

            // Proportional turn
            double turnPower = TURN_POWER_CENTER;
            long turnTimeMs = (long) (Math.abs(tx) * PROPORTIONAL_GAIN_TURN);
            turnTimeMs = Math.max(turnTimeMs, 150);  // Min 150ms for visible turn
            turnTimeMs = Math.min(turnTimeMs, 800);  // Max 800ms

            telemetry.addData("Loop %d: Turning", "%s %.1f° for %d ms",
                    (tx > 0 ? "RIGHT" : "LEFT"), Math.abs(tx), turnTimeMs);
            telemetry.update();

            if (tx > 0) {
                // RIGHT turn (clockwise)
                setDrivePowers(turnPower, turnPower, -turnPower, -turnPower);
            } else {
                // LEFT turn (anticlockwise)
                setDrivePowers(-turnPower, -turnPower, turnPower, turnPower);
            }

            sleep(turnTimeMs);
            stopAllDriveMotors();
            telemetry.addLine("Loop %d: Turn complete - stopping motors");
            telemetry.update();

            // Reset Pedro after turn
            telemetry.addLine("Loop %d: Resetting Pedro after turn...", centerIterations);
            resetPedroPose();
            telemetry.addLine("Loop %d: Pedro reset done", centerIterations);

            sleep(100);  // Settle time
        }

        telemetry.addData("Center End: Iterations", centerIterations);
        telemetry.addData("Center End: Time", "%.1fs", (System.currentTimeMillis() - centerStart) / 1000.0);
        if (System.currentTimeMillis() - centerStart >= CENTER_TIMEOUT_MS) {
            telemetry.addLine("*** CENTER TIMEOUT - Check Limelight data above ***");
        }
        telemetry.update();
    }

    // Measure distance once, compute delta, move exactly that amount using pathing
    private void adjustToTargetDistance() throws InterruptedException {
        telemetry.addLine("=== MEASURING DISTANCE ===");
        telemetry.update();

        // Get final measurement (after centering)
        sleep(500);  // Extra settle time
        LLResult result = limelight.getLatestResult();
        boolean valid = (result != null && result.isValid());
        telemetry.addData("Measure: Valid Result?", valid);
        if (!valid) {
            telemetry.addLine("Measure: INVALID - Skipping adjust");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int numFid = (fiducials != null) ? fiducials.size() : 0;
        telemetry.addData("Measure: Num Fiducials", numFid);
        if (numFid == 0) {
            telemetry.addLine("Measure: NO FIDUCIALS - Skipping");
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
            telemetry.addLine("Measure: TAG 24 MISSING - Skipping");
            return;
        }

        double tx = tag.getTargetXDegrees();
        double targetArea = result.getTa();
        double distanceIn = getDistanceFromTagInches(targetArea);
        double delta = distanceIn - TARGET_DISTANCE_IN;

        // Calibration helper
        double suggestedScale = 70.0 * Math.sqrt(targetArea);
        telemetry.addData("Measure: Final TX", "%.2f°", tx);
        telemetry.addData("Measure: ta %%", "%.3f", targetArea);
        telemetry.addData("Measure: Dist", "%.1f in", distanceIn);
        telemetry.addData("Measure: Delta", "%.1f in (tol: %.1f)", delta, DISTANCE_TOLERANCE_IN);
        telemetry.addData("Measure: CALIB Suggest SCALE", "%.1f (curr: %.1f)", suggestedScale, DISTANCE_SCALE_SQRT_IN);
        telemetry.update();

        if (Math.abs(delta) < DISTANCE_TOLERANCE_IN) {
            telemetry.addLine("Already at target - No move needed!");
            return;
        }

        // Reset Pedro before distance move
        telemetry.addLine("Resetting Pedro before distance move...");
        resetPedroPose();

        // Move the exact delta
        if (delta > 0) {
            // Too far: move FORWARD |delta| inches (toward tag)
            telemetry.addData("Moving FORWARD %.1f in", delta);
            moveForwardHeadingRelative(delta);
        } else {
            // Too close: move BACKWARD |delta| inches (away from tag)
            telemetry.addData("Moving BACKWARD %.1f in", Math.abs(delta));
            moveBackwardHeadingRelative(Math.abs(delta));
        }
        telemetry.update();

        // Final reset
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

    // === Pathing Movement Methods (No Intake) ===

    private void moveForwardHeadingRelative(double inches) throws InterruptedException {
        telemetry.addLine("Pathing: Building FORWARD path...");
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
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Path Progress", "%.1f%%", follower.getProgress() * 100);
            telemetry.update();
        }
        follower.breakFollowing();
        telemetry.addLine("FORWARD path complete");
    }

    private void moveBackwardHeadingRelative(double inches) throws InterruptedException {
        telemetry.addLine("Pathing: Building BACKWARD path...");
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
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Path Progress", "%.1f%%", follower.getProgress() * 100);
            telemetry.update();
        }
        follower.breakFollowing();
        telemetry.addLine("BACKWARD path complete");
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