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

@TeleOp(name = "Limelight Detection", group = "Autonomous")
public class LimelightTest extends OpMode {

    // Hardware objects
    private Limelight3A limelight;

    // We are using Pipeline 5 for AprilTag detection
    private static final int APRILTAG_PIPELINE = 5;

    // Distance scale calibrated for your setup
    private static final double DISTANCE_SCALE = 341;

    @Override
    public void init() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to AprilTag pipeline (set this in web interface first!)
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        // Tell driver we're ready
        telemetry.addData("Status", "Initialized!");
        telemetry.addData("AprilTag Pipeline", APRILTAG_PIPELINE);
        telemetry.update();
    }

    @Override
    public void start() {
        // Start Limelight when we press START (saves battery)
        limelight.start();
    }

    @Override
    public void loop() {
        // Get latest Limelight result
        LLResult result = limelight.getLatestResult();

        // Check if we got valid AprilTag data
        if (result != null && result.isValid()) {

            // Get list of detected AprilTags
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Check if any AprilTags are detected
            if (fiducials != null && !fiducials.isEmpty()) {

                // Get the first AprilTag
                LLResultTypes.FiducialResult tag = fiducials.get(0);

                // Get AprilTag ID
                double tagId = tag.getFiducialId();

                // Get target offsets
                double tx = tag.getTargetXDegrees();  // Horizontal offset
                double ty = tag.getTargetYDegrees();  // Vertical offset

                // Get general target data from result
                double targetArea = result.getTa();   // How much screen tag fills

                // Calculate distance using target area (inverse square law)
                double distance = getDistanceFromTag(targetArea);

                // ===== DISPLAY EVERYTHING =====
                telemetry.addData("===== APRILTAG DETECTED =====", "");
                telemetry.addData("", "");

                // AprilTag info
                telemetry.addData("AprilTag ID", "%.0f", tagId);
                telemetry.addData("Total Tags Visible", fiducials.size());
                telemetry.addData("", "");

                // Target offsets
                telemetry.addData("TX (horizontal)", "%.2f degrees", tx);
                telemetry.addData("TY (vertical)", "%.2f degrees", ty);
                telemetry.addData("Target Area", "%.2f%%", targetArea);
                telemetry.addData("", "");

                // Calculated distance
                telemetry.addData("Distance to Tag", "%.2f cm", distance);

            } else {
                // No AprilTags found
                telemetry.addData("Status", "No AprilTags in view");
            }

        } else {
            // No valid result from Limelight
            telemetry.addData("Status", "Waiting for Limelight...");
        }

        // Update telemetry
        telemetry.update();
    }

    /**
     * Calculate distance from AprilTag using target area
     * Uses inverse square law: as you move 2x farther, tag appears 4x smaller
     * Formula: distance = scale / targetArea
     */
    public double getDistanceFromTag(double targetArea) {
        return DISTANCE_SCALE / targetArea;
    }

    @Override
    public void stop() {
        // Stop Limelight when OpMode ends
        limelight.stop();
    }
}