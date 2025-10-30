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
    private final Pose startPose = new Pose(81, 8, Math.toRadians(90)); // Updated starting pose

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
            shooter.setPower(0.85); // Wait 4 seconds for shooter to reach speed
            sleep(1000);

            // === Step 2: Feed first ball with transfer servos ===
            log("Status", "Step 2: Feeding first ball...");
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);
            sleep(500);

            // === Step 3: Run intake briefly to load second ball ===
            log("Status", "Step 3: Loading second ball...");
            intake.setPower(-0.25);
            sleep(50);
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
        follower.setMaxPower(0.75); // From SpikeMarkRed
        follower.setStartingPose(startPose);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        log("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        runtime.reset();
        pathState = 0;
        foundID = 0;

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();

            // Check for AprilTag detection
            detectAprilTag();

            // Update state machine if a tag is found
            if (foundID != 0) {
                updateStateMachine();
            }

            // Log telemetry
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update();
        }

        // Stop Limelight
        limelight.stop();
    }

    private void detectAprilTag() {
        int timeout = 0;
        while (opModeIsActive() && timeout < DETECTION_TIMEOUT) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int tagID = fiducials.get(0).getFiducialId(); // Removed redundant cast
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
                    return; // Exit loop once a tag is detected
                }
            }
            sleep(50);
            timeout++;
        }
        log("Limelight", "No target detected after timeout");
    }

    public void setCurrentPath(int tagID) {
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
            // Path 1: Align (start to shoot)
            Pose scoring1 = new Pose(84, 84, Math.toRadians(45));
            Pose pickup1 = new Pose(108, 84, Math.toRadians(0));
            alignPPG = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoring1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                    .build();
            //Path 1.5: shoot to pickup align
            shoot2PPG = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, pickup1))
                    .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1.getHeading())
                    .build();

            // Path 2: Scoop (first pickup to second pickup)
            Pose pickup2 = new Pose(120, 84, Math.toRadians(0));
            scoopPPG = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1, pickup2))
                    .setConstantHeadingInterpolation(pickup1.getHeading())
                    .build();

            // Path 3: Shoot (second pickup to first scoring)
            shoot1PPG = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2, scoring1))
                    .setLinearHeadingInterpolation(pickup2.getHeading(), scoring1.getHeading())
                    .build();

            // Path 4: Leave (first scoring to final pose)
            Pose scoring2 = new Pose(96, 48, Math.toRadians(45));
            leavePPG = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, scoring2))
                    .setConstantHeadingInterpolation(scoring1.getHeading())
                    .build();

            pathsBuiltPPG = true;
        }
    }

    public void buildPathsPGP() {
        if (!pathsBuiltPGP) {
            // Path 1: Align (start to first shoot, no y adjustment)
            Pose pickup1 = new Pose(108, 60, Math.toRadians(0));
            Pose scoring1 = new Pose(84, 84, Math.toRadians(45));
            alignPGP = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoring1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                    .build();
            //Path 1.5: shoot to pickup align
            shoot2PGP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, pickup1))
                    .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1.getHeading())
                    .build();
            // Path 2: Scoop (first pickup to second pickup, Y lowered by 24)
            Pose pickup2 = new Pose(120, 60, Math.toRadians(0));
            scoopPGP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1, pickup2))
                    .setConstantHeadingInterpolation(pickup1.getHeading())
                    .build();

            // Path 3: Shoot (second pickup to first scoring, Y lowered by 24)

            shoot1PGP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2, scoring1))
                    .setLinearHeadingInterpolation(pickup2.getHeading(), scoring1.getHeading())
                    .build();

            // Path 4: Leave (first scoring to final pose, no y adjustment)
            Pose scoring2 = new Pose(96, 48, Math.toRadians(45));
            leavePGP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, scoring2))
                    .setConstantHeadingInterpolation(scoring1.getHeading())
                    .build();

            pathsBuiltPGP = true;
        }
    }

    public void buildPathsGPP() {
        if (!pathsBuiltGPP) {
            // Path 1: Align (start to first pickup, no y adjustment)
            Pose pickup1 = new Pose(108, 36, Math.toRadians(0));
            Pose scoring1 = new Pose(84, 84, Math.toRadians(45));
            alignGPP = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scoring1))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                    .build();
            //Path 1.5: shoot to pickup align
            shoot2GPP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, pickup1))
                    .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1.getHeading())
                    .build();
            // Path 2: Scoop (first pickup to second pickup, Y lowered by 48)
            Pose pickup2 = new Pose(120, 36, Math.toRadians(0));
            scoopGPP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1, pickup2))
                    .setConstantHeadingInterpolation(pickup1.getHeading())
                    .build();

            // Path 3: Shoot (second pickup to first scoring, Y lowered by 48)

            shoot1GPP = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2, scoring1))
                    .setLinearHeadingInterpolation(pickup2.getHeading(), scoring1.getHeading())
                    .build();

            // Path 4: Leave (first scoring to final pose, no y adjustment)
            Pose scoring2 = new Pose(96, 48, Math.toRadians(45));
            leaveGPP = follower.pathBuilder()
                    .addPath(new BezierLine(scoring1, scoring2))
                    .setConstantHeadingInterpolation(scoring1.getHeading())
                    .build();

            pathsBuiltGPP = true;
        }
    }

    public void updateStateMachine() {
        switch (pathState) {
            case 0:
                switch (foundID) {
                    case PPG_TAG_ID: follower.followPath(alignPPG); break;
                    case PGP_TAG_ID: follower.followPath(alignPGP); break;
                    case GPP_TAG_ID: follower.followPath(alignGPP); break;
                }
                shootArtifacts();
                switch (foundID) {
                    case PPG_TAG_ID: follower.followPath(shoot2PPG); break;
                    case PGP_TAG_ID: follower.followPath(shoot2PGP); break;
                    case GPP_TAG_ID: follower.followPath(shoot2GPP); break;
                }
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(scoopPPG); break;
                        case PGP_TAG_ID: follower.followPath(scoopPGP); break;
                        case GPP_TAG_ID: follower.followPath(scoopGPP); break;
                    }
                    intakeArtifacts();
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(shoot1PPG); break;
                        case PGP_TAG_ID: follower.followPath(shoot1PGP); break;
                        case GPP_TAG_ID: follower.followPath(shoot1GPP); break;
                    }
                    shootArtifacts();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(leavePPG); break;
                        case PGP_TAG_ID: follower.followPath(leavePGP); break;
                        case GPP_TAG_ID: follower.followPath(leaveGPP); break;
                    }
                    pathState = -1; // Stop or reset for next tag
                }
                break;
        }
    }
}