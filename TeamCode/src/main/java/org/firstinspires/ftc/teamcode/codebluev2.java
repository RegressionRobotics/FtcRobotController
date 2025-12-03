package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "BlueMecanum - NO HOME, NO SKIPS, SPINNER MOVES")
public class codebluev2 extends OpMode {

    DcMotor frontLeft, backLeft, frontRight, backRight;
    DcMotor shooter, intake;
    Servo rightSerial;
    ServoImplEx spinner;

    Follower follower;
    final Pose targetScoringPose = new Pose(57, 88, Math.toRadians(305));
    boolean isFollowingPath = false;
    PathChain scoringPath;

    // 0.0 = 500μs | 1.0 = 2500μs
    final double INTAKE_1 = 0.000;   // 500
    final double INTAKE_2 = 0.333;   // 1166
    final double INTAKE_3 = 0.667;   // 1833

    final double SHOOT_1 = 0.166;    // 833
    final double SHOOT_2 = 0.500;    // 1500
    final double SHOOT_3 = 0.833;    // 1833

    final double PUSHER_FIRE = 0.15;
    final double PUSHER_HOME = 0.85;
    final double SHOOTER_IDLE = 0.15;
    final double SHOOTER_FIRE = 0.80;

    ElapsedTime timer = new ElapsedTime();
    int intakeStep = 0;   // 0 = idle, 1–3 = slots
    int shootStep  = 0;   // 0 = idle, 1–9 = shooting sequence

    @Override public void init() {
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight  = hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        intake      = hardwareMap.dcMotor.get("intake");
        rightSerial = hardwareMap.servo.get("rightSerial");
        spinner     = (ServoImplEx) hardwareMap.servo.get("spinner");

        // THIS MAKES YOUR CONTINUOUS SERVO ACTUALLY MOVE
        spinner.setPwmRange(new PwmControl.PwmRange(500, 2500));

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);
    }

    @Override public void start() {
        follower.setStartingPose(new Pose(48, 49, Math.toRadians(305)));
        follower.startTeleopDrive();
        intake.setPower(0.35);
        shooter.setPower(SHOOTER_IDLE);
        spinner.setPosition(SHOOT_1);   // start at first shoot position
        rightSerial.setPosition(PUSHER_HOME);
        timer.reset();
    }

    @Override public void loop() {
        follower.update();

        // AUTO-PATH (Y)
        if (gamepad1.y && !isFollowingPath) {
            scoringPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), targetScoringPose))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetScoringPose.getHeading())
                    .build();
            follower.followPath(scoringPath, true);
            isFollowingPath = true;
        }
        if (isFollowingPath && (gamepad1.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            isFollowingPath = false;
        }
        if (!isFollowingPath) {
            double ly = -gamepad1.left_stick_y;
            double lx = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(ly, lx, rx, true);
        }

        // INTAKE — NO HOME, NO SKIPS
        if (gamepad1.right_trigger > 0.5) {
            if (intakeStep == 0) { spinner.setPosition(INTAKE_1); intake.setPower(1.0); timer.reset(); intakeStep = 1; }
            else if (timer.seconds() >= 1.0) {
                intakeStep++;
                if (intakeStep == 2) spinner.setPosition(INTAKE_2);
                if (intakeStep == 3) spinner.setPosition(INTAKE_3);
                timer.reset();
            }
        } else {
            intakeStep = 0;
            intake.setPower(0.35);
            // spinner stays wherever it is — NO FORCED HOME
        }

        // SHOOTING — 5 sec first shot, 1 sec between next, NO SKIPS, NO HOME
        if (gamepad1.left_trigger > 0.5) {
            if (shootStep == 0) {
                shooter.setPower(SHOOTER_FIRE);
                spinner.setPosition(SHOOT_1);
                timer.reset();
                shootStep = 1;
            }

            // Step 1 → wait 5 sec
            if (shootStep == 1 && timer.seconds() >= 3.0) { rightSerial.setPosition(PUSHER_FIRE); timer.reset(); shootStep = 2; }
            if (shootStep == 2 && timer.seconds() >= 0.5) { rightSerial.setPosition(PUSHER_HOME);  timer.reset(); shootStep = 3; }

            // Step 3 → wait 1 sec → fire shot 2
            if (shootStep == 3 && timer.seconds() >= 1.0) { spinner.setPosition(SHOOT_2); rightSerial.setPosition(PUSHER_FIRE); timer.reset(); shootStep = 4; }
            if (shootStep == 4 && timer.seconds() >= 0.5) { rightSerial.setPosition(PUSHER_HOME);  timer.reset(); shootStep = 5; }

            // Step 5 → wait 1 sec → fire shot 3
            if (shootStep == 5 && timer.seconds() >= 1.0) { spinner.setPosition(SHOOT_3); rightSerial.setPosition(PUSHER_FIRE); timer.reset(); shootStep = 6; }
            if (shootStep == 6 && timer.seconds() >= 0.5) { rightSerial.setPosition(PUSHER_HOME);  timer.reset(); shootStep = 7; } // done
        } else {
            shootStep = 0;
            shooter.setPower(SHOOTER_IDLE);
            rightSerial.setPosition(PUSHER_HOME);
            // spinner stays at last position — NO HOME EVER
        }

        telemetry.addData("Spinner (0-1)", String.format("%.3f", spinner.getPosition()));
        telemetry.addData("PWM", (int)(spinner.getPosition() * 2000 + 500));
        telemetry.addData("Intake Step", intakeStep);
        telemetry.addData("Shoot Step", shootStep);
        telemetry.update();
    }

    @Override public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        rightSerial.setPosition(PUSHER_HOME);
        // spinner left wherever it was
    }
}