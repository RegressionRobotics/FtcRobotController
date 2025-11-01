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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "RedAutonWPedro", group = "Opmode")
@Configurable
public class AutonRed extends OpMode {

    // === Timers ===
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final Timer pathTimer = new Timer();
    private final Timer opmodeTimer = new Timer();
    private int intakePulseCount = 0;   // counts completed ON-pulses

    // === Poses ===
    private final Pose startPose = new Pose(85, 9, Math.toRadians(270));

    // === AprilTag IDs ===
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int APRILTAG_PIPELINE = 5;
    private static final int DETECTION_TIMEOUT = 100;

    // === Hardware ===
    private DcMotor  intake;
    private DcMotor  shooter;
    private CRServo  leftTransfer;
    private CRServo  rightTransfer;
    private Servo    arjav;

    // === Pathing ===
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathState = 0;
    private int foundID   = 0;

    // === Limelight ===
    private Limelight3A limelight;

    // === Path Chains ===
    private PathChain alignPPG, toPickup1PPG, scoopPPG, backToScorePPG, leavePPG;
    private PathChain alignPGP, toPickup1PGP, scoopPGP, backToScorePGP, leavePGP;
    private PathChain alignGPP, toPickup1GPP, scoopGPP, backToScoreGPP, leaveGPP;

    // === Shooting ===
    private boolean isShooting = false;
    private int     shootStep  = 0;
    private int     shootCount = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // === Intake ===
    private boolean isIntaking = false;
    private final ElapsedTime intakeTimer = new ElapsedTime();

    // === Timer Class ===
    public static class Timer {
        private final ElapsedTime t = new ElapsedTime();
        public void resetTimer() { t.reset(); }
        public double getElapsedTimeSeconds() { return t.seconds(); }
    }

    // === Logging ===
    private void log(String caption, Object value) {
        telemetry.addData(caption, value);
        if (panelsTelemetry != null) panelsTelemetry.debug(caption + ": " + value);
    }

    // === NON-BLOCKING WAIT (no idle, no isStopRequested) ===
    private void waitFor(double milliseconds) {
        waitTimer.reset();
        while (waitTimer.milliseconds() < milliseconds) {
            // Keep robot alive
            follower.update();
            updateShooting();
            updateIntake();
            if (panelsTelemetry != null) panelsTelemetry.update();
            // No idle(), no Thread.yield() — just loop
        }
    }

    // === NON-BLOCKING INTAKE ===

    private void updateIntake() {
        if (!isIntaking) return;
        if (intakeTimer.milliseconds() >= 1000) {
            intake.setPower(0);
            isIntaking = false;
            log("Intake", "Finished");
        }
    }

    // === Shooting: Non-blocking ===
    private void startShooting() {
        if (isShooting) return;
        isShooting = true;
        shootStep = 0;
        shootTimer.reset();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arjav.setPosition(1.0);

        // Set shooter power: 0.9 only on the second shoot, otherwise 0.75
        double shooterPower = (shootCount == 1) ? 0.75 : 0.751;
        shooter.setPower(shooterPower);

        log("Shooting", "Started (Count: " + shootCount + ", Power: " + shooterPower + ")");
    }

    private void updateShooting() {
        if (!isShooting) return;

        switch (shootStep) {
            case 0: // Spin up shooter — power already set in startShooting()
                if (shootTimer.milliseconds() > 1900) {
                    shootStep = 1;
                    shootTimer.reset();
                    leftTransfer.setPower(1.0);
                    rightTransfer.setPower(1.0);
                    log("Shooting", "Feeding first ball");
                }
                break;

            case 1: // Feed first ball
                if (shootTimer.milliseconds() > 1000) {
                    shootStep = 2;
                    shootTimer.reset();
                    intake.setPower(1.0);               // start first intake pulse
                    intakePulseCount = 0;               // reset counter for the 5 cycles
                    log("Shooting", "Starting intake pulses");
                }
                break;

            case 2: // Intake pulsing (0.2 s on / 0.2 s off, 5 times)
                if (intakePulseCount >= 5) {            // all 5 cycles done?
                    shootStep = 3;
                    shootTimer.reset();
                    log("Shooting", "Intake pulses finished");
                    break;
                }

                // 0.2 s = 200 ms
                if (shootTimer.milliseconds() >= 200) {
                    if (intake.getPower() > 0) {        // was ON → turn OFF
                        intake.setPower(0.0);
                    } else {                            // was OFF → turn ON & count
                        intake.setPower(1.0);
                        intakePulseCount++;
                    }
                    shootTimer.reset();                 // restart timer for next 0.2 s slot
                }
                break;

            case 3: // Run shooter (original 3-second hold)
                if (shootTimer.milliseconds() > 1000) {
                    stopShooting();
                }
                break;
        }
    }
    private void stopShooting() {
        shooter.setPower(0);
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
        intake.setPower(0);
        arjav.setPosition(0.5);
        isShooting = false;
        shootCount++;  // ADD THIS LINE
        log("Shooting", "Finished (Total Shoots: " + shootCount + ")");
    }

    // === Path State Machine ===
    private void autonomousPathUpdate() {
        if (foundID == 0) return;

        switch (pathState) {
            case 0: /* This case starts the alignment to the scoring position */
                if (true) {
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(alignPPG, true); break;
                        case PGP_TAG_ID: follower.followPath(alignPGP, true); break;
                        case GPP_TAG_ID: follower.followPath(alignGPP, true); break;
                    }
                    setPathState(1);
                }
                break;

            case 1: /* This case waits for return to scoring and starts shooting */
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(2);
                }
                break;

            case 2: /* This case waits for alignment to finish and moves to pickup 1 */
                if (!isShooting) {
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(toPickup1PPG, true); break;
                        case PGP_TAG_ID: follower.followPath(toPickup1PGP, true); break;
                        case GPP_TAG_ID: follower.followPath(toPickup1GPP, true); break;
                    }
                    setPathState(3);
                }
                break;

            case 3: /* This case waits for robot to reach pickup 1 and scoops the sample */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.setMaxPower(0.2);
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(scoopPPG, true); break;
                        case PGP_TAG_ID: follower.followPath(scoopPGP, true); break;
                        case GPP_TAG_ID: follower.followPath(scoopGPP, true); break;
                    }
                    setPathState(4);
                }
                break;

            case 4: /* This case waits for scoop to finish and returns to scoring position */

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    intake.setPower(0);
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(backToScorePPG, true); break;
                        case PGP_TAG_ID: follower.followPath(backToScorePGP, true); break;
                        case GPP_TAG_ID: follower.followPath(backToScoreGPP, true); break;
                    }
                    setPathState(5);
                }
                break;

            case 5: /* This case waits for return to scoring and starts shooting */
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(6);
                }
                break;

            case 6: /* This case waits for shooting to complete */
                if (!isShooting) {
                    follower.setMaxPower(1);
                    switch (foundID) {
                        case PPG_TAG_ID: follower.followPath(leavePPG, true); break;
                        case PGP_TAG_ID: follower.followPath(leavePGP, true); break;
                        case GPP_TAG_ID: follower.followPath(leaveGPP, true); break;
                    }
                    setPathState(7);
                }
                break;

            case 7: /* This case waits for leave path to finish */
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        log("Path State", pathState);
    }

    // === Path Building ===
    private void buildPaths() {
        Pose scoring1 = new Pose(90, 90, Math.toRadians(220));
        Pose scoring2 = new Pose(96, 49, Math.toRadians(220));

        // PPG
        Pose pickup1GPP = new Pose(100, 39, Math.toRadians(0));
        Pose pickup2GPP = new Pose(124, 39, Math.toRadians(0));

        alignPPG = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1PPG = follower.pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1GPP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1GPP.getHeading())
                .build();

        scoopPPG = follower.pathBuilder()
                .addPath(new BezierLine(pickup1GPP, pickup2GPP))
                .setConstantHeadingInterpolation(pickup1GPP.getHeading())
                .build();

        backToScorePPG = follower.pathBuilder()
                .addPath(new BezierLine(pickup2GPP, scoring1))
                .setLinearHeadingInterpolation(pickup2GPP.getHeading(), scoring1.getHeading())
                .build();

        leavePPG = follower.pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();

        // PGP
        Pose pickup1PPG = new Pose(100, 83, Math.toRadians(0));
        Pose pickup2PPG = new Pose(124, 83, Math.toRadians(0));

        alignPGP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1PGP = follower.pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PPG))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PPG.getHeading())
                .build();

        scoopPGP = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PPG, pickup2PPG))
                .setConstantHeadingInterpolation(pickup1PPG.getHeading())
                .build();

        backToScorePGP = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PPG, scoring1))
                .setLinearHeadingInterpolation(pickup2PPG.getHeading(), scoring1.getHeading())
                .build();

        leavePGP = follower.pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();

        // GPP
        Pose pickup1PGP = new Pose(100, 59, Math.toRadians(0));
        Pose pickup2PGP = new Pose(124, 59, Math.toRadians(0));

        alignGPP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scoring1))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoring1.getHeading())
                .build();

        toPickup1GPP = follower.pathBuilder()
                .addPath(new BezierLine(scoring1, pickup1PGP))
                .setLinearHeadingInterpolation(scoring1.getHeading(), pickup1PGP.getHeading())
                .build();

        scoopGPP = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PGP, pickup2PGP))
                .setConstantHeadingInterpolation(pickup1PGP.getHeading())
                .build();

        backToScoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PGP, scoring1))
                .setLinearHeadingInterpolation(pickup2PGP.getHeading(), scoring1.getHeading())
                .build();

        leaveGPP = follower.pathBuilder()
                .addPath(new BezierLine(scoring1, scoring2))
                .setConstantHeadingInterpolation(scoring1.getHeading())
                .build();
    }

    // === AprilTag Detection (runs in start()) ===
    private void detectAprilTag() {
        int timeout = 0;
        while (timeout < DETECTION_TIMEOUT && foundID == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int tagID = fiducials.get(0).getFiducialId();
                    if (tagID == PPG_TAG_ID || tagID == PGP_TAG_ID || tagID == GPP_TAG_ID) {
                        foundID = tagID;
                        log("Detected Tag", tagID);
                        return;
                    }
                }
            }
            waitFor(50);
            timeout++;
        }
        foundID = PPG_TAG_ID;
        log("Warning", "No tag detected – using PPG");
    }

    // ==============================================================
    //  OpMode Lifecycle – ONLY THESE 5 METHODS
    // ==============================================================

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake        = hardwareMap.get(DcMotor.class,  "intake");
        shooter       = hardwareMap.get(DcMotor.class,  "shooter");
        leftTransfer  = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");
        arjav         = hardwareMap.get(Servo.class,    "arjav");

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        opmodeTimer.resetTimer();

        buildPaths();

        log("Status", "INIT: Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        log("Status", "INIT_LOOP: Press START");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        waitFor(1000);
        detectAprilTag();
        setPathState(0);
        log("Status", "START: Running");
    }

    @Override
    public void loop() {
        follower.update();
        updateShooting();
        updateIntake();
        autonomousPathUpdate();

        if (panelsTelemetry != null) panelsTelemetry.update();

        Pose pose = follower.getPose();
        double normH = Math.toDegrees((pose.getHeading() + 2 * Math.PI) % (2 * Math.PI));

        log("X",            String.format("%.2f", pose.getX()));
        log("Y",            String.format("%.2f", pose.getY()));
        log("Heading",      String.format("%.2f°", normH));
        log("Path State",   pathState);
        log("Found ID",     foundID);
        log("Time (s)",     String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        stopShooting();
        log("Status", "STOP: Done");
    }
}