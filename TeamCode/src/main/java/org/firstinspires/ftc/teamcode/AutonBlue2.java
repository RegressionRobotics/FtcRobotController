package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
//hi
@Autonomous(name = "BlueAuton - 3 SHOTS + INTAKE DELAY (FINAL)", group = "Opmode")
@Configurable
public class AutonBlue2 extends OpMode {

    private DcMotor intake, shooter;
    private Servo rightSerial;
    private ServoImplEx spinner;

    private Follower follower;
    private Limelight3A limelight;
    private int foundID = 0;
    private boolean detectionComplete = false;

    // Shooting
    private boolean isShooting = false;
    private int shotsFired = 0;                   // 0,1,2,3
    private final ElapsedTime shootTimer = new ElapsedTime();

    // Intake
    private boolean isIntaking = false;
    private int intakeSlot = 0;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private boolean intakeStarted = false;        // NEW: ensures 0.5s delay before first spin

    private final Pose startPose = new Pose(59, 9, Math.toRadians(90));
    private final Pose scoringPose = new Pose(48, 96, Math.toRadians(130));

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int APRILTAG_PIPELINE = 5;

    // Servo Positions
    private static final double SHOOT_1 = 0.175;
    private static final double SHOOT_2 = 0.520;
    private static final double SHOOT_3 = 0.853;

    private static final double PUSHER_HOME = 0.55;
    private static final double PUSHER_FIRE = 0.95;

    private static final double SHOOTER_IDLE = 0.20;
    private static final double SHOOTER_FIRE = 0.70;

    // Paths
    private PathChain alignPPG, toPickup1PPG, scoopPPG, backToScorePPG, leavePPG;
    private PathChain alignPGP, toPickup1PGP, scoopPGP, backToScorePGP, leavePGP;
    private PathChain alignGPP, toPickup1GPP, scoopGPP, backToScoreGPP, leaveGPP;

    private int pathState = 0;

    @Override public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSerial = hardwareMap.servo.get("rightSerial");
        spinner = (ServoImplEx)  hardwareMap.servo.get("spinner");
        spinner.setPwmRange(new PwmControl.PwmRange(500, 2500));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        buildPaths();

        telemetry.addData("Status", "Ready – Waiting for AprilTag");
        telemetry.update();
    }

    @Override public void init_loop() {
        if (!detectionComplete) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    int id = tags.get(0).getFiducialId();
                    if (id == PPG_TAG_ID || id == PGP_TAG_ID || id == GPP_TAG_ID) {
                        foundID = id;
                        detectionComplete = true;
                        telemetry.addData("TAG LOCKED", id);
                    }
                }
            }
            telemetry.update();
        }
    }

    @Override public void start() {
        if (foundID == 0) foundID = PPG_TAG_ID;
        pathState = 0;
        follower.followPath(getAlignPath(), true);

        shooter.setPower(SHOOTER_IDLE);
        spinner.setPosition(SHOOT_1);
        rightSerial.setPosition(PUSHER_HOME);
    }

    @Override public void loop() {
        follower.update();
        updateShooting();
        updateIntake();

        switch (pathState) {
            case 0: if (!follower.isBusy()) { startShooting(); pathState = 1; } break;
            case 1: if (!isShooting) { follower.setMaxPower(0.75); follower.followPath(getToPickupPath(), true); pathState = 2; } break;
            case 2: if (!follower.isBusy()) { startIntake(); follower.setMaxPower(0.30); follower.followPath(getScoopPath(), true); pathState = 3; } break;
            case 3: if (!follower.isBusy()) { follower.setMaxPower(0.85); follower.followPath(getBackToScorePath(), true); pathState = 4; } break;
            case 4: if (!follower.isBusy()) { startShooting(); pathState = 5; } break;
            case 5: if (!isShooting) { follower.setMaxPower(0.9); follower.followPath(getLeavePath(), true); pathState = 99; } break;
        }

        Pose p = follower.getPose();
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("H", "%.0f°", Math.toDegrees(p.getHeading()));
        telemetry.addData("State", pathState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Intake Slot", intakeSlot);
        telemetry.addData("Shoot Timer", "%.2f", shootTimer.seconds());
        telemetry.update();
    }

    // ———————————————— 100% WORKING 3-SHOT SEQUENCE ————————————————

    private void startShooting() {
        isShooting = true;
        shotsFired = 0;
        shooter.setPower(SHOOTER_FIRE);
        spinner.setPosition(SHOOT_1);
        rightSerial.setPosition(PUSHER_HOME);
        shootTimer.reset();
    }

    private void updateShooting() {
        if (!isShooting) return;

        double t = shootTimer.seconds();

        // First shot: wait 3.0s spool-up → fire
        if (shotsFired == 0 && t >= 3.0) {
            rightSerial.setPosition(PUSHER_FIRE);
            shotsFired = 1;
            shootTimer.reset();
            return;
        }

        // Cycle for shots 1→2 and 2→3
        if (shotsFired >= 1 && shotsFired <= 2) {
            if (t >= 0.75) {
                rightSerial.setPosition(PUSHER_HOME);                    // retract
            }
            if (t >= 0.75 + 0.90) {
                if (shotsFired == 1) spinner.setPosition(SHOOT_2);       // advance to next ring
                if (shotsFired == 2) spinner.setPosition(SHOOT_3);
            }
            if (t >= 0.75 + 0.90 + 1.00) {
                rightSerial.setPosition(PUSHER_FIRE);                    // FIRE!
                shotsFired++;
                shootTimer.reset();
            }
        }

        // After 3rd shot: final retract and finish
        if (shotsFired == 3 && t >= 0.75) {
            rightSerial.setPosition(PUSHER_HOME);
            shooter.setPower(SHOOTER_IDLE);
            isShooting = false;
        }
    }

    // ———————————————— INTAKE WITH 0.5s DELAY BEFORE FIRST ROTATION ————————————————

    private void startIntake() {
        isIntaking = true;
        intakeSlot = 0;
        intakeStarted = false;
        intake.setPower(1.0);
        spinner.setPosition(0.000);  // start at intake position 1
        intakeTimer.reset();
    }

    private void updateIntake() {
        if (!isIntaking) return;

        double t = intakeTimer.seconds();

        // First: wait 0.5s after intake starts before any spinner movement
        if (!intakeStarted && t >= 0.5) {
            intakeStarted = true;
            intakeTimer.reset();
            return;
        }

        // Now run normal timing (1.0s between each slot)
        if (intakeStarted && t >= 1.0) {
            intakeSlot++;
            if (intakeSlot == 1) spinner.setPosition(0.340);
            else if (intakeSlot == 2) spinner.setPosition(0.700);
            else {
                intake.setPower(0.3);
                isIntaking = false;
            }
            intakeTimer.reset();
        }
    }

    // ———————————————— PATH GETTERS ————————————————

    private PathChain getAlignPath()       { return foundID == PPG_TAG_ID ? alignPPG : foundID == PGP_TAG_ID ? alignPGP : alignGPP; }
    private PathChain getToPickupPath()    { return foundID == PPG_TAG_ID ? toPickup1PPG : foundID == PGP_TAG_ID ? toPickup1PGP : toPickup1GPP; }
    private PathChain getScoopPath()       { return foundID == PPG_TAG_ID ? scoopPPG : foundID == PGP_TAG_ID ? scoopPGP : scoopGPP; }
    private PathChain getBackToScorePath() { return foundID == PPG_TAG_ID ? backToScorePPG : foundID == PGP_TAG_ID ? backToScorePGP : backToScoreGPP; }
    private PathChain getLeavePath()       { return foundID == PPG_TAG_ID ? leavePPG : foundID == PGP_TAG_ID ? leavePGP : leaveGPP; }

    private void buildPaths() {
        // PPG
        Pose ppg1 = new Pose(48, 83, Math.toRadians(180));
        Pose ppg2 = new Pose(24, 83, Math.toRadians(180));
        alignPPG = follower.pathBuilder().addPath(new BezierLine(startPose, scoringPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading()).build();
        toPickup1PPG = follower.pathBuilder().addPath(new BezierLine(scoringPose, ppg1))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), ppg1.getHeading()).build();
        scoopPPG = follower.pathBuilder().addPath(new BezierLine(ppg1, ppg2))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();
        backToScorePPG = follower.pathBuilder().addPath(new BezierLine(ppg2, scoringPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scoringPose.getHeading()).build();
        leavePPG = follower.pathBuilder().addPath(new BezierLine(scoringPose, new Pose(48, 49, Math.toRadians(135))))
                .setConstantHeadingInterpolation(scoringPose.getHeading()).build();

        // PGP
        Pose pgp1 = new Pose(48, 59, Math.toRadians(180));
        Pose pgp2 = new Pose(24, 59, Math.toRadians(180));
        alignPGP = follower.pathBuilder().addPath(new BezierLine(startPose, scoringPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading()).build();
        toPickup1PGP = follower.pathBuilder().addPath(new BezierLine(scoringPose, pgp1))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), pgp1.getHeading()).build();
        scoopPGP = follower.pathBuilder().addPath(new BezierLine(pgp1, pgp2))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();
        backToScorePGP = follower.pathBuilder().addPath(new BezierLine(pgp2, scoringPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scoringPose.getHeading()).build();
        leavePGP = follower.pathBuilder().addPath(new BezierLine(scoringPose, new Pose(48, 49, Math.toRadians(135))))
                .setConstantHeadingInterpolation(scoringPose.getHeading()).build();

        // GPP
        Pose gpp1 = new Pose(48, 35, Math.toRadians(180));
        Pose gpp2 = new Pose(24, 35, Math.toRadians(180));
        alignGPP = follower.pathBuilder().addPath(new BezierLine(startPose, scoringPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading()).build();
        toPickup1GPP = follower.pathBuilder().addPath(new BezierLine(scoringPose, gpp1))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), gpp1.getHeading()).build();
        scoopGPP = follower.pathBuilder().addPath(new BezierLine(gpp1, gpp2))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();
        backToScoreGPP = follower.pathBuilder().addPath(new BezierLine(gpp2, scoringPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scoringPose.getHeading()).build();
        leaveGPP = follower.pathBuilder().addPath(new BezierLine(scoringPose, new Pose(48, 49, Math.toRadians(135))))
                .setConstantHeadingInterpolation(scoringPose.getHeading()).build();
    }

    @Override public void stop() {
        if (limelight != null) limelight.stop();
        shooter.setPower(0);
        intake.setPower(0);
        rightSerial.setPosition(PUSHER_HOME);
    }
}