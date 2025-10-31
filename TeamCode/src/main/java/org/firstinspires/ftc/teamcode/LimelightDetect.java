package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * ========== LIMELIGHT SETUP INSTRUCTIONS ==========
 *
 * STEP 1: INITIAL PIPELINE CONFIGURATION (DO THIS FIRST!)
 * ------------------------------------------------------
 * 1. Connect Limelight to YOUR COMPUTER via USB-C cable
 * 2. Wait 30 seconds for Limelight to boot up (light will turn solid)
 * 3. Open web browser on your computer
 * 4. Go to: limelight.local:5801
 *    - If that doesn't work, try: 10.0.0.3:5801
 * 5. In the web interface:
 *    a) Click on "Pipelines" tab at the top
 *    b) Select pipeline slot 5 (we're using pipeline 5)
 *    c) Click "Type" dropdown and select "AprilTag"
 *    d) Under "AprilTag Family", select "36h11" (FTC standard)
 *    e) Adjust any other settings if needed (exposure, gain, etc.)
 * 6. IMPORTANT: Pipeline auto-saves to Limelight memory!
 *    - Settings are stored ON the Limelight device itself
 *    - They will persist even after unplugging
 * 7. Optional: Click "Settings" tab and give your Limelight a name
 * 8. Optional: Download config backup:
 *    - Go to "Settings" tab
 *    - Click "Download Settings" button
 *    - Save .zip file to your computer as backup
 * 9. UNPLUG Limelight from your computer
 *
 * STEP 2: CONTROL HUB CONFIGURATION FILE
 * ---------------------------------------
 * 1. PLUG Limelight into Control Hub's BLUE USB 3.0 port (NOT black port!)
 * 2. On Driver Station app:
 *    a) Click three dots menu (⋮)
 *    b) Select "Configure Robot"
 *    c) Choose your active configuration (or create new one)
 *    d) Click "SCAN" button at bottom
 *    e) You'll see "Limelight3A" appear as Ethernet device
 *    f) Click on it and rename to "limelight" (must match code!)
 *    g) Click "Done" then "Save"
 *    h) Click "Activate" (this will restart the Control Hub)
 * 3. Wait for Control Hub to restart (about 10-15 seconds)
 *
 * STEP 3: TESTING
 * ----------------
 * 1. Make sure Limelight is still plugged into Control Hub (blue port)
 * 2. Run this OpMode (Limelight Detection)
 * 3. Point Limelight at an AprilTag
 * 4. Check Driver Station for detected tag ID and data
 *
 * TROUBLESHOOTING:
 * ----------------
 * - If "limelight.local:5801" doesn't work, try: 10.0.0.3:5801
 * - Make sure USB-C cable is fully plugged in (try different cables)
 * - Limelight light should be solid green when connected properly
 * - Pipeline 5 must be configured BEFORE plugging into Control Hub
 * - If you change pipelines later, plug back into computer, make changes,
 *   then plug back into Control Hub (settings auto-save)
 *
 * IMPORTANT NOTES:
 * ----------------
 * - Pipelines are saved IN the Limelight's memory (not on Control Hub)
 * - You CAN'T create pipelines in code, only switch between saved ones
 * - Always use BLUE USB 3.0 port on Control Hub for best performance
 * - Pipeline 5 is specified in this code (see APRILTAG_PIPELINE constant)
 */

@TeleOp(name = "Limelight Detection - FIXED FOR 70 INCHES", group = "Autonomous")
public class LimelightDetect extends OpMode {

    // Hardware objects
    private Limelight3A limelight;

    // We are using Pipeline 5 for AprilTag detection
    private static final int APRILTAG_PIPELINE = 4;

    // === CORRECTED DISTANCE CALIBRATION ===
    // ta is in PERCENT (0-100), NO *100 NEEDED!
    // Formula: distance_inches = DISTANCE_SCALE_SQRT_IN / sqrt(ta)
    // Calibrated assuming ta ≈ 0.92% at 70 inches (from your original readings)
    // 70 * sqrt(0.92) ≈ 67
    private static final double DISTANCE_SCALE_SQRT_IN = 67.0;  // <-- TUNE IF NEEDED

    @Override
    public void init() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to AprilTag pipeline (must be pre-configured in web UI!)
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        // Tell driver we're ready
        telemetry.addData("Status", "Initialized!");
        telemetry.addData("AprilTag Pipeline", APRILTAG_PIPELINE);
        telemetry.addData("Distance Formula", "SCALE_IN / sqrt(ta %)");
        telemetry.addData("Scale (inches)", DISTANCE_SCALE_SQRT_IN);
        telemetry.addData("CALIBRATE: At known dist, set SCALE = dist_in * sqrt(ta)", "");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start Limelight streaming when OpMode starts
        limelight.start();
    }

    @Override
    public void loop() {
        // Get latest result from Limelight
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {

                // Use the first detected tag
                LLResultTypes.FiducialResult tag = fiducials.get(0);

                // Extract data
                double tagId       = tag.getFiducialId();
                double tx          = tag.getTargetXDegrees();   // Horizontal offset
                double ty          = tag.getTargetYDegrees();   // Vertical offset
                double targetArea  = result.getTa();            // % of image occupied (0-100)

                // === CORRECT DISTANCE CALCULATION (IN INCHES) ===
                double distanceIn = getDistanceFromTagInches(targetArea);

                // Also show cm for reference
                double distanceCm = distanceIn * 2.54;

                // Calibration helper: Suggested scale based on THIS reading
                double suggestedScale = 70.0 * Math.sqrt(targetArea);  // Assuming 70in physical

                // === TELEMETRY OUTPUT ===
                telemetry.addData("===== APRILTAG DETECTED =====", "");
                telemetry.addData("", "");

                telemetry.addData("AprilTag ID", "%.0f", tagId);
                telemetry.addData("Total Tags Visible", fiducials.size());
                telemetry.addData("", "");

                telemetry.addData("TX (horizontal)", "%.2f degrees", tx);
                telemetry.addData("TY (vertical)",   "%.2f degrees", ty);
                telemetry.addData("Target Area",     "%.3f%%", targetArea);  // Show 3 decimals
                telemetry.addData("", "");

                telemetry.addData("Distance to Tag", "%.1f inches", distanceIn);
                telemetry.addData("Distance to Tag", "%.1f cm", distanceCm);
                telemetry.addData("", "");

                telemetry.addData("CALIB HELP (for 70in):", "SCALE = 70 * sqrt(%.3f) = %.1f", targetArea, suggestedScale);
                telemetry.addData("Current Scale Used", "%.1f", DISTANCE_SCALE_SQRT_IN);

            } else {
                telemetry.addData("Status", "No AprilTags in view");
            }

        } else {
            telemetry.addData("Status", "Waiting for Limelight data...");
        }

        telemetry.update();
    }

    /**
     * Calculate distance using inverse square root of target area.
     *
     * ta is already in PERCENT (0–100) from Limelight.
     *
     * Physical principle:
     *   - Tag has fixed size (6 inches / 15.24 cm)
     *   - As distance doubles, area becomes 1/4
     *   - So linear size scales with 1/sqrt(area)
     *
     * @param targetArea percentage of image filled by tag (0–100)
     * @return distance in inches
     */
    public double getDistanceFromTagInches(double targetArea) {
        if (targetArea < 0.01) return 0;  // Avoid division by zero / tiny values
        return DISTANCE_SCALE_SQRT_IN / Math.sqrt(targetArea);
    }

    @Override
    public void stop() {
        // Stop Limelight to save power
        if (limelight != null) {
            limelight.stop();
        }
    }
}