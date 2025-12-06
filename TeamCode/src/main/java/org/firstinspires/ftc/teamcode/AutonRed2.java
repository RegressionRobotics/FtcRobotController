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
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "RedAuton - FINAL FIXED SPINNER & PUSHER", group = "Opmode")
@Configurable
public class AutonRed2 extends OpMode {

    private DcMotor intake, shooter;
    private Servo rightSerial;
    private ServoImplEx spinner;

    private Follower follower;
    private int pathState = 0;
    private int foundID = 0;

    private Limelight3A limelight;
    private boolean detectionComplete = false;

    private final Pose startPose = new Pose(85, 9, Math.toRadians(90));
    private final Pose scoringPose = new Pose(96, 96, Math.toRadians(135));

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final int APRILTAG_PIPELINE = 5;

    private final double SHOOT_1 = 0.175;    // 833

    private final double PUSHER_HOME = 0.55;   // retracted = safe
    private final double SHOOTER_IDLE = 0.20;

    private boolean isShooting = false;
    private int shotsFired = 0;                    // counts 0,1,2,3
    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean isIntaking = false;
    private int intakeSlot = 0;                    // 0,1,2
    private final ElapsedTime intakeTimer = new ElapsedTime();

    private PathChain alignPPG, toPickup1PPG, scoopPPG, backToScorePPG, leavePPG;
    private PathChain alignPGP, toPickup1PGP, scoopPGP, backToScorePGP, leavePGP;
    private PathChain alignGPP, toPickup1GPP, scoopGPP, backToScoreGPP, leaveGPP;

    @Override public void init() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSerial = hardwareMap.servo.get("rightSerial");
        spinner = (ServoImplEx) hardwareMap.servo.get("spinner");
        spinner.setPwmRange(new PwmControl.PwmRange(500, 2500));  // ← MAKES SPINNER MOVE

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        buildPaths();

        telemetry.addData("Status", "INIT — WAITING FOR APRILTAG");
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
            case 1: if (!isShooting) { follower.followPath(getToPickupPath(), true); pathState = 2; } break;
            case 2: if (!follower.isBusy()) { startIntake(); follower.followPath(getScoopPath(), true); pathState = 3; } break;
            case 3: if (!follower.isBusy()) { follower.followPath(getBackToScorePath(), true); pathState = 4; } break;
            case 4: if (!follower.isBusy()) { startShooting(); pathState = 5; } break;
            case 5: if (!isShooting) { follower.followPath(getLeavePath(), true); pathState = 99; } break;
        }

        Pose p = follower.getPose();
        telemetry.addData("X", "%.2f", p.getX());
        telemetry.addData("Y", "%.2f", p.getY());
        telemetry.addData("H", "%.1f°", Math.toDegrees(p.getHeading()));
        telemetry.addData("Path", pathState);
        telemetry.addData("Tag", foundID);
        telemetry.addData("Spinner", "%.3f", spinner.getPosition());
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.update();
    }

    private void startIntake() {
        isIntaking = true;
        intakeSlot = 0;
        // 0.0 = 500μs | 1.0 = 2500μs — THESE ARE YOUR WORKING VALUES
        // 500
        double INTAKE_1 = 0.000;
        spinner.setPosition(INTAKE_1);
        intake.setPower(1.0);
        intakeTimer.reset();
    }

    private void updateIntake() {
        if (!isIntaking) return;
        if (intakeTimer.seconds() >= 1.0) {
            intakeSlot++;
            // 1166
            double INTAKE_2 = 0.340;
            // 1833
            double INTAKE_3 = 0.700;
            if (intakeSlot == 1) spinner.setPosition(INTAKE_2);
            else if (intakeSlot == 2) spinner.setPosition(INTAKE_3);
            else {
                intake.setPower(0.35);
                isIntaking = false;
            }
            intakeTimer.reset();
        }
    }

    private void startShooting() {
        isShooting = true;
        shotsFired = 0;
        double SHOOTER_FIRE = 0.70;
        shooter.setPower(SHOOTER_FIRE);
        spinner.setPosition(SHOOT_1);
        shootTimer.reset();
    }

    private void updateShooting() {
        if (!isShooting) return;

        if (shotsFired == 0 && shootTimer.seconds() >= 3.0) {
            // extended = fire
            double PUSHER_FIRE = 0.95;
            rightSerial.setPosition(PUSHER_FIRE);
            shootTimer.reset();
            shotsFired = 1;
        }
        else if (shotsFired >= 1 && shootTimer.seconds() >= 0.5) {
            rightSerial.setPosition(PUSHER_HOME);
            shootTimer.reset();

            // 1500
            double SHOOT_2 = 0.520;
            // 1833
            double SHOOT_3 = 0.853;
            if (shotsFired == 1) spinner.setPosition(SHOOT_2);
            else if (shotsFired == 2) spinner.setPosition(SHOOT_3);

            if (shotsFired < 3) {
                shootTimer.reset();  // wait 1 sec before next fire
            } else {
                shooter.setPower(SHOOTER_IDLE);
                rightSerial.setPosition(PUSHER_HOME);
                isShooting = false;
            }
            if (shotsFired < 3) shotsFired++;
        }
    }

    // PATH GETTERS
    private PathChain getAlignPath()       { return foundID == PPG_TAG_ID ? alignPPG : foundID == PGP_TAG_ID ? alignPGP : alignGPP; }
    private PathChain getToPickupPath()    { return foundID == PPG_TAG_ID ? toPickup1PPG : foundID == PGP_TAG_ID ? toPickup1PGP : toPickup1GPP; }
    private PathChain getScoopPath()       { return foundID == PPG_TAG_ID ? scoopPPG : foundID == PGP_TAG_ID ? scoopPGP : scoopGPP; }
    private PathChain getBackToScorePath() { return foundID == PPG_TAG_ID ? backToScorePPG : foundID == PGP_TAG_ID ? backToScorePGP : backToScoreGPP; }
    private PathChain getLeavePath()       { return foundID == PPG_TAG_ID ? leavePPG : foundID == PGP_TAG_ID ? leavePGP : leaveGPP; }

    private void buildPaths() {
        // PPG
        Pose ppg1 = new Pose(48, 83, Math.toRadians(180));
        Pose ppg2 = new Pose(24, 83, Math.toRadians(180));
        alignPPG = follower.pathBuilder().addPath(new BezierLine(startPose, scoringPose)).setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading()).build();
        toPickup1PPG = follower.pathBuilder().addPath(new BezierLine(scoringPose, ppg1)).setLinearHeadingInterpolation(scoringPose.getHeading(), ppg1.getHeading()).build();
        scoopPPG = follower.pathBuilder().addPath(new BezierLine(ppg1, ppg2)).setConstantHeadingInterpolation(ppg1.getHeading()).build();
        backToScorePPG = follower.pathBuilder().addPath(new BezierLine(ppg2, scoringPose)).setLinearHeadingInterpolation(ppg2.getHeading(), scoringPose.getHeading()).build();
        leavePPG = follower.pathBuilder().addPath(new BezierLine(scoringPose, new Pose(48, 49, Math.toRadians(135)))).setConstantHeadingInterpolation(scoringPose.getHeading()).build();

        // PGP
        Pose pgp1 = new Pose(48, 59, Math.toRadians(180));
        Pose pgp2 = new Pose(24, 59, Math.toRadians(180));
        alignPGP = follower.pathBuilder().addPath(new BezierLine(startPose, scoringPose)).setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading()).build();
        toPickup1PGP = follower.pathBuilder().addPath(new BezierLine(scoringPose, pgp1)).setLinearHeadingInterpolation(scoringPose.getHeading(), pgp1.getHeading()).build();
        scoopPGP = follower.pathBuilder().addPath(new BezierLine(pgp1, pgp2)).setConstantHeadingInterpolation(pgp1.getHeading()).build();
        backToScorePGP = follower.pathBuilder().addPath(new BezierLine(pgp2, scoringPose)).setLinearHeadingInterpolation(pgp2.getHeading(), scoringPose.getHeading()).build();
        leavePGP = follower.pathBuilder().addPath(new BezierLine(scoringPose, new Pose(48, 49, Math.toRadians(135)))).setConstantHeadingInterpolation(scoringPose.getHeading()).build();

        // GPP
        Pose gpp1 = new Pose(48, 35, Math.toRadians(180));
        Pose gpp2 = new Pose(24, 35, Math.toRadians(180));
        alignGPP = follower.pathBuilder().addPath(new BezierLine(startPose, scoringPose)).setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading()).build();
        toPickup1GPP = follower.pathBuilder().addPath(new BezierLine(scoringPose, gpp1)).setLinearHeadingInterpolation(scoringPose.getHeading(), gpp1.getHeading()).build();
        scoopGPP = follower.pathBuilder().addPath(new BezierLine(gpp1, gpp2)).setConstantHeadingInterpolation(gpp1.getHeading()).build();
        backToScoreGPP = follower.pathBuilder().addPath(new BezierLine(gpp2, scoringPose)).setLinearHeadingInterpolation(gpp2.getHeading(), scoringPose.getHeading()).build();
        leaveGPP = follower.pathBuilder().addPath(new BezierLine(scoringPose, new Pose(48, 49, Math.toRadians(135)))).setConstantHeadingInterpolation(scoringPose.getHeading()).build();
    }

    @Override public void stop() {
        if (limelight != null) limelight.stop();
        shooter.setPower(0);
        intake.setPower(0);
        rightSerial.setPosition(PUSHER_HOME);
    }
}