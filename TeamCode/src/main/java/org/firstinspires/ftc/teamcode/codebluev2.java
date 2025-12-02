package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BlueMecanumTeleOp - Slots + Full Rumble")
public class codebluev2 extends OpMode {

    DcMotor frontLeft, backLeft, frontRight, backRight;
    DcMotor shooter, intake;
    Servo spinner, rightSerial;
    AnalogInput turretenc;
    DistanceSensor checker;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    Follower follower;
    final Pose targetScoringPose = new Pose(57, 88, Math.toRadians(305));
    boolean isFollowingPath = false;
    PathChain scoringPath;

    final int[] intakePWMs = {500, 1166, 1833};
    final int[] shootPWMs  = {1500, 2165, 833};

    boolean[] slotsFull = new boolean[3];

    enum IntakeState { IDLE, RUNNING }
    enum ShootState  { IDLE, FIRING }

    IntakeState intakeState = IntakeState.IDLE;
    ShootState  shootState  = ShootState.IDLE;
    final ElapsedTime timer = new ElapsedTime();

    static final double OBJECT_DETECTED_CM = 5.0;

    List<Integer> emptySlots, fullSlots;
    int intakeTryIndex = 0;
    int shootIndex = 0;

    // NEW: Tracks if we already started the "all full" rumble
    private boolean wasAllFullLastLoop = false;

    @Override
    public void init() {
        // (same as before — no changes here)
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight  = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("shooter");
        intake  = hardwareMap.dcMotor.get("intake");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinner     = hardwareMap.get(Servo.class, "spinner");
        rightSerial = hardwareMap.servo.get("rightSerial");

        if (spinner instanceof ServoImplEx) {
            ((ServoImplEx) spinner).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        turretenc = hardwareMap.get(AnalogInput.class, "turretenc");
        checker   = hardwareMap.get(DistanceSensor.class, "checker");

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);

        telemetry.addData("Status", "Initialized – RT = Intake | LT = Shoot");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(48, 49, Math.toRadians(305)));
        follower.startTeleopDrive();
        intake.setPower(0.35);
        setSpinnerPWM(1500);
        rightSerial.setPosition(0.5);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        follower.update();

        // Pathing (unchanged)
        if (currentGamepad1.y && !previousGamepad1.y && !isFollowingPath) {
            buildPathToScoring();
            follower.followPath(scoringPath, true);
            isFollowingPath = true;
        }
        if (isFollowingPath && (currentGamepad1.b && !previousGamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            isFollowingPath = false;
        }

        if (!isFollowingPath) {
            double ly = -gamepad1.left_stick_y;
            double lx = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(ly, lx, rx, true);
        }

        if (gamepad1.right_trigger > 0.5 && intakeState == IntakeState.IDLE) startIntake();
        if (gamepad1.left_trigger  > 0.5 && shootState  == ShootState.IDLE)  startShooting();

        updateIntake();
        updateShooting();

        if (intakeState == IntakeState.IDLE && shootState == ShootState.IDLE) {
            intake.setPower(0.35);
            setSpinnerPWM(1500);
        }

        // NEW: CONTINUOUS RUMBLE WHEN ALL THREE SLOTS ARE FULL
        boolean allFull = slotsFull[0] && slotsFull[1] && slotsFull[2];

        if (allFull && !wasAllFullLastLoop) {
            // Just became full → start strong rumble
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);  // Full power continuous rumble
        } else if (!allFull && wasAllFullLastLoop) {
            // Just stopped being full → stop rumble
            gamepad1.stopRumble();
        }
        // If still all full → rumble keeps going automatically
        wasAllFullLastLoop = allFull;

        // Telemetry
        Pose p = follower.getPose();
        double h = Math.toDegrees((p.getHeading() + 2 * Math.PI) % (2 * Math.PI));
        double dist = checker.getDistance(DistanceUnit.CM);
        if (dist > 200 || Double.isNaN(dist)) dist = 999;

        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), h);
        telemetry.addData("Proximity cm", "%.1f", dist);
        telemetry.addData("Slots", "1:%b 2:%b 3:%b%s", slotsFull[0], slotsFull[1], slotsFull[2], allFull ? " ← FULL!" : "");
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Shoot", shootState);
        telemetry.update();
    }

    // INTAKE / SHOOTING / HELPERS — unchanged (same smart behavior as before)
    private void startIntake() {
        emptySlots = new ArrayList<>();
        for (int i = 0; i < 3; i++) if (!slotsFull[i]) emptySlots.add(i);
        if (emptySlots.isEmpty()) { gamepad1.rumble(500); return; }

        intakeTryIndex = 0;
        setSpinnerPWM(intakePWMs[emptySlots.get(0)]);
        intake.setPower(1.0);
        timer.reset();
        intakeState = IntakeState.RUNNING;
        gamepad1.rumble(200);
    }

    private void updateIntake() {
        if (intakeState != IntakeState.RUNNING) return;

        double dist = checker.getDistance(DistanceUnit.CM);
        boolean hasObject = (dist < OBJECT_DETECTED_CM && dist > 0);

        if (timer.seconds() >= 0.5) {
            if (hasObject) {
                slotsFull[emptySlots.get(intakeTryIndex)] = true;
                intakeDone();
            } else if (intakeTryIndex + 1 < emptySlots.size()) {
                intakeTryIndex++;
                setSpinnerPWM(intakePWMs[emptySlots.get(intakeTryIndex)]);
                timer.reset();
            } else {
                intakeDone();
            }
        }
    }

    private void intakeDone() {
        setSpinnerPWM(1500);
        intake.setPower(0.35);
        intakeState = IntakeState.IDLE;
    }

    private void startShooting() {
        fullSlots = new ArrayList<>();
        for (int i = 0; i < 3; i++) if (slotsFull[i]) fullSlots.add(i);
        if (fullSlots.isEmpty()) { gamepad1.rumble(500); return; }

        shootIndex = 0;
        shooter.setPower(0.75);
        setSpinnerPWM(shootPWMs[fullSlots.get(0)]);
        rightSerial.setPosition(1.0);
        timer.reset();
        shootState = ShootState.FIRING;
        gamepad1.rumble(400);
    }

    private void updateShooting() {
        if (shootState != ShootState.FIRING) return;

        if (timer.seconds() >= 0.25) {
            rightSerial.setPosition(0.5);
            slotsFull[fullSlots.get(shootIndex)] = false;

            shootIndex++;
            if (shootIndex < fullSlots.size()) {
                setSpinnerPWM(shootPWMs[fullSlots.get(shootIndex)]);
                rightSerial.setPosition(1.0);
                timer.reset();
            } else {
                shooter.setPower(0);
                setSpinnerPWM(1500);
                shootState = ShootState.IDLE;
            }
        }
    }

    private void setSpinnerPWM(int microseconds) {
        double pos = Range.clip((microseconds - 500.0) / 2000.0, 0.0, 1.0);
        spinner.setPosition(pos);
    }

    private void buildPathToScoring() {
        Pose current = follower.getPose();
        scoringPath = follower.pathBuilder()
                .addPath(new BezierLine(current, targetScoringPose))
                .setLinearHeadingInterpolation(current.getHeading(), targetScoringPose.getHeading())
                .build();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        setSpinnerPWM(1500);
        gamepad1.stopRumble();   // Safety
    }
}