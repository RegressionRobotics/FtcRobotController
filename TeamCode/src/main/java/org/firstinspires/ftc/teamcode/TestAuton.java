package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "RedAutonWPedro", group = "Opmode")
@Configurable
@SuppressWarnings("FieldCanBeLocal")
public class TestAuton extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(81, 8, Math.toRadians(270)); // Updated starting pose

    // Initialize variables for paths
    private int pathState = 0;

    // AprilTag IDs
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;

    // Limelight configuration
    private static final int APRILTAG_PIPELINE = 5; // From SpikeMarkRed example
    private static final int DETECTION_TIMEOUT = 100; // Number of loops to wait for detection
    private Limelight3A limelight;

    // Hardware
    private DcMotor intake, shooter;
    private CRServo leftTransfer, rightTransfer;
    private Servo arjav;

    // Path building flags
    private boolean pathsBuiltPPG = false;
    private boolean pathsBuiltPGP = false;
    private boolean pathsBuiltGPP = false;

    // Other variables
    private Pose currentPose;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int foundID;

    // Custom logging function
    private void log(String caption, Object value) {
        telemetry.addData(caption, value);
        panelsTelemetry.debug(caption + ": " + value);
    }

    // Intake function (adapted from SpikeMarkRed)
    public void intakeArtifacts() {
        intake.setPower(0.85);
        sleep(250); // Run intake briefly to load artifact
        intake.setPower(0);
    }

    // Shooting function (adapted from SpikeMarkRed's startAutoShooterSequence)
    public void shootArtifacts() {
        // --- INITIALIZATION ---
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- AUTONOMOUS SEQUENCE ---
        if (opModeIsActive()) {
            // === Step 0: Activate Arjav servo ===
            log("Status", "Step 0: Setting Arjav servo to position 1");
            arjav.setPosition(1.0);

            // === Step 1: Power up the shooter motor ===
            log("Status", "Step 1: Spinning up shooter...");
            shooter.setPower(0.75); // Wait 4 seconds for shooter to reach speed
            sleep(1000);

            // === Step 2: Feed first ball with transfer servos ===
            log("Status", "Step 2: Feeding first ball...");
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);
            sleep(500);

            // === Step 3: Run intake briefly to load second ball ===
            log("Status", "Step 3: Loading second ball...");
            intake.setPower(0.85);
            sleep(250);

            // === Step 4: Keep shooter + transfers running for a while ===
            log("Status", "Step 4: Running shooter...");
            sleep(3000);

            // === Step 5: Stop everything ===
            log("Status", "Step 5: Stopping all hardware.");
            shooter.setPower(0);
            leftTransfer.setPower(0);
            rightTransfer.setPower(0);
            intake.setPower(0);
            arjav.setPosition(0.5);
        }
    }

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");
        arjav = hardwareMap.get(Servo.class, "arjav");

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75); // From SpikeMarkRed, reduced for safety
        follower.setStartingPose(startPose);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Fallback path building for debugging (remove after testing)
        buildPathsPPG(); // Default to PPG for testing
        log("Status", "Paths built for debugging (PPG)");

        log("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start and allow initial detection
        waitForStart();
        runtime.reset();
        sleep(1000); // Give Limelight time to detect
        pathState = 0;
        foundID = 0;

        // Force initial path detection
        detectAprilTag();
        if (foundID == 0) {
            log("Warning", "No tag detected initially, using PPG as fallback");
            foundID = PPG_TAG_ID; // Fallback to PPG if no tag is detected
        }

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();

            // Check for AprilTag detection
            detectAprilTag();
            log("Found ID", foundID); // Debug: Check if foundID is set

            // Update state machine if a tag is found
            if (foundID != 0) {
                updateStateMachine();
            }

            // Log position data with normalized heading
            double rawHeading = currentPose.getHeading();
            double normalizedHeading = Math.toDegrees((rawHeading + 2 * Math.PI) % (2 * Math.PI)); // 0 to 360
            log("Position - X", String.format("%.2f", currentPose.getX()));
            log("Position - Y", String.format("%.2f", currentPose.getY()));
            log("Position - Heading (Raw)", String.format("%.2f°", Math.toDegrees(rawHeading))); // Raw value
            log("Position - Heading (Normalized)", String.format("%.2f°", normalizedHeading)); // 0 to 360

            // Debug heading during path execution
            if (pathState > 0 && !follower.isBusy()) {
                log("Debug", "Heading after path " + pathState + ": " + String.format("%.2f°", Math.toDegrees(rawHeading)));
            }

            telemetry.update();
        }

        // Stop Limelight
        limelight.stop();
    }

    private void detectAprilTag() {
        int timeout = 0;
        while (opModeIsActive() && timeout < DETECTION_TIMEOUT && foundID == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int tagID = fiducials.get(0).getFiducialId();
                    if (tagID == PPG_TAG_ID) {
                        setCurrentPath(PPG_TAG_ID);
                        foundID = PPG_TAG_ID;
                    } else if (tagID == PGP_TAG_ID) {
                        setCurrentPath(PGP_TAG_ID);
                        foundID = PGP_TAG_ID;
                    } else if (tagID == GPP_TAG_ID) {
                        setCurrentPath(GPP_TAG_ID);
                        foundID = GPP_TAG_ID;
                    } else {
                        log("Limelight", "Unknown tag ID: " + tagID);
                    }
                    log("Detected Tag", tagID); // Debug: Confirm detection
                    return; // Exit loop once a tag is detected
                }
            }
            sleep(50);
            timeout++;
        }
        log("Limelight", "No target detected after timeout");
    }

    public void setCurrentPath(int tagID) {
        log("Setting path for ID", tagID); // Debug: Confirm path setting
        switch (tagID) {
            case PPG_TAG_ID:
                buildPathsPPG();
                break;
            case PGP_TAG_ID:
                buildPathsPGP();
                break;
            case GPP_TAG_ID:
                buildPathsGPP();
                break;
        }
        pathState = 0; // Reset state machine
    }

    private PathChain alignPPG, shoot2PPG, scoopPPG, shoot1PPG, leavePPG;
    private PathChain alignPGP, shoot2PGP, scoopPGP, shoot1PGP, leavePGP;
    private PathChain alignGPP, shoot2GPP, scoopGPP, shoot1GPP, leaveGPP;

    public void buildPathsPPG() {
        if (!pathsBuiltPPG) {
            log("Building paths for PPG", "Starting...");
            // Path 1: Align (start to scoring position)
            Pose scoring1 = new Pose(84, 84, Math.toRadians(225));
            alignPPG = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoring1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                    .build();

            // Path 1.5: Move to pickup align (scoring to pickup)
            Pose pickup1 = new Pose(108, 84, Math.toRadians(0));
            shoot2PPG = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, pickup1))
                    .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1.getHeading())
                    .build();

            // Path 2: Scoop (pickup to second pickup)
            Pose pickup2 = new Pose(120, 84, Math.toRadians(0));
            scoopPPG = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1, pickup2))
                    .setConstantHeadingInterpolation(pickup1.getHeading())
                    .build();

            // Path 3: Shoot (second pickup back to scoring)
            shoot1PPG = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2, scoring1))
                    .setLinearHeadingInterpolation(pickup2.getHeading(), scoring1.getHeading())
                    .build();

            // Path 4: Leave (scoring to final pose)
            Pose scoring2 = new Pose(96, 48, Math.toRadians(225));
            leavePPG = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, scoring2))
                    .setConstantHeadingInterpolation(scoring1.getHeading())
                    .build();

            pathsBuiltPPG = true;
            log("Building paths for PPG", "Completed");
        }
    }

    public void buildPathsPGP() {
        if (!pathsBuiltPGP) {
            log("Building paths for PGP", "Starting...");
            // Path 1: Align (start to scoring position)
            Pose scoring1 = new Pose(84, 84, Math.toRadians(225));
            alignPGP = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoring1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                    .build();

            // Path 1.5: Move to pickup align (scoring to pickup)
            Pose pickup1 = new Pose(108, 84, Math.toRadians(0));
            shoot2PGP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, pickup1))
                    .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1.getHeading())
                    .build();

            // Path 2: Scoop (pickup to second pickup, Y lowered by 24)
            Pose pickup2 = new Pose(120, 84 - 24, Math.toRadians(0));
            scoopPGP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1, pickup2))
                    .setConstantHeadingInterpolation(pickup1.getHeading())
                    .build();

            // Path 3: Shoot (second pickup back to scoring, Y lowered by 24)
            shoot1PGP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2, scoring1))
                    .setLinearHeadingInterpolation(pickup2.getHeading(), scoring1.getHeading())
                    .build();

            // Path 4: Leave (scoring to final pose, no y adjustment)
            Pose scoring2 = new Pose(96, 48, Math.toRadians(225));
            leavePGP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, scoring2))
                    .setConstantHeadingInterpolation(scoring1.getHeading())
                    .build();

            pathsBuiltPGP = true;
            log("Building paths for PGP", "Completed");
        }
    }

    public void buildPathsGPP() {
        if (!pathsBuiltGPP) {
            log("Building paths for GPP", "Starting...");
            // Path 1: Align (start to scoring position)
            Pose scoring1 = new Pose(84, 84, Math.toRadians(225));
            alignGPP = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoring1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                    .build();

            // Path 1.5: Move to pickup align (scoring to pickup)
            Pose pickup1 = new Pose(108, 84, Math.toRadians(0));
            shoot2GPP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, pickup1))
                    .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1.getHeading())
                    .build();

            // Path 2: Scoop (pickup to second pickup, Y lowered by 48)
            Pose pickup2 = new Pose(120, 84 - 48, Math.toRadians(0));
            scoopGPP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1, pickup2))
                    .setConstantHeadingInterpolation(pickup1.getHeading())
                    .build();

            // Path 3: Shoot (second pickup back to scoring, Y lowered by 48)
            shoot1GPP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2, scoring1))
                    .setLinearHeadingInterpolation(pickup2.getHeading(), scoring1.getHeading())
                    .build();

            // Path 4: Leave (scoring to final pose, no y adjustment)
            Pose scoring2 = new Pose(96, 48, Math.toRadians(225));
            leaveGPP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, scoring2))
                    .setConstantHeadingInterpolation(scoring1.getHeading())
                    .build();

            pathsBuiltGPP = true;
            log("Building paths for GPP", "Completed");
        }
    }

    public void updateStateMachine() {
        switch (pathState) {
            case 0:
                switch (foundID) {
                    case PPG_TAG_ID:
                        if (alignPPG != null) follower.followPath(alignPPG);
                        else log("Error", "alignPPG is null");
                        break;
                    case PGP_TAG_ID:
                        if (alignPGP != null) follower.followPath(alignPGP);
                        else log("Error", "alignPGP is null");
                        break;
                    case GPP_TAG_ID:
                        if (alignGPP != null) follower.followPath(alignGPP);
                        else log("Error", "alignGPP is null");
                        break;
                }
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    switch (foundID) {
                        case PPG_TAG_ID:
                            if (shoot2PPG != null) follower.followPath(shoot2PPG);
                            else log("Error", "shoot2PPG is null");
                            if (scoopPPG != null) follower.followPath(scoopPPG);
                            else log("Error", "scoopPPG is null");
                            break;
                        case PGP_TAG_ID:
                            if (shoot2PGP != null) follower.followPath(shoot2PGP);
                            else log("Error", "shoot2PGP is null");
                            if (scoopPGP != null) follower.followPath(scoopPGP);
                            else log("Error", "scoopPGP is null");
                            break;
                        case GPP_TAG_ID:
                            if (shoot2GPP != null) follower.followPath(shoot2GPP);
                            else log("Error", "shoot2GPP is null");
                            if (scoopGPP != null) follower.followPath(scoopGPP);
                            else log("Error", "scoopGPP is null");
                            break;
                    }
                    intakeArtifacts();
                    switch (foundID) {
                        case PPG_TAG_ID:
                            if (shoot1PPG != null) follower.followPath(shoot1PPG);
                            else log("Error", "shoot1PPG is null");
                            break;
                        case PGP_TAG_ID:
                            if (shoot1PGP != null) follower.followPath(shoot1PGP);
                            else log("Error", "shoot1PGP is null");
                            break;
                        case GPP_TAG_ID:
                            if (shoot1GPP != null) follower.followPath(shoot1GPP);
                            else log("Error", "shoot1GPP is null");
                            break;
                    }
                    shootArtifacts();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    switch (foundID) {
                        case PPG_TAG_ID:
                            if (leavePPG != null) follower.followPath(leavePPG);
                            else log("Error", "leavePPG is null");
                            break;
                        case PGP_TAG_ID:
                            if (leavePGP != null) follower.followPath(leavePGP);
                            else log("Error", "leavePGP is null");
                            break;
                        case GPP_TAG_ID:
                            if (leaveGPP != null) follower.followPath(leaveGPP);
                            else log("Error", "leaveGPP is null");
                            break;
                    }
                    pathState = -1; // Stop or reset for next tag
                }
                break;
        }
    }
}