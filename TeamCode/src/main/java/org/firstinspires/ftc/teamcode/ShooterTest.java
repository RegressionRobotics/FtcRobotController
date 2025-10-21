package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends LinearOpMode {

    // Encoder ticks per revolution (adjust for your shooter motor)
    private static final double TICKS_PER_REV = 28.0;

    // PID constants (tune these!)
    private static final double kP = 0.0008;   // proportional gain
    private static final double kI = 0.000001; // integral gain
    private static final double kD = 0.0003;   // derivative gain

    @Override
    public void runOpMode() {
        // --- Hardware setup ---
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        // --- Shooter control variables ---
        boolean shooterOn = false;
        double targetRPM = 2000; // starting target speed
        int prevShooterPos = shooter.getCurrentPosition();
        double prevTime = getRuntime();

        // PID state
        double integral = 0;
        double lastError = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // --- Shooter toggle (X) ---
            if (currentGamepad1.x && !previousGamepad1.x) {
                shooterOn = !shooterOn;
            }

            // --- Adjust target RPM (D-pad) ---
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                targetRPM += 100;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                targetRPM -= 100;
                if (targetRPM < 0) targetRPM = 0;
            }
            
            // --- Shooter RPM measurement ---
            int currentPos = shooter.getCurrentPosition();
            double currentTime = getRuntime();
            double deltaTicks = currentPos - prevShooterPos;
            double deltaTime = currentTime - prevTime;

            double ticksPerSecond = 0;
            double currentRPM = 0;
            if (deltaTime > 1e-6) {
                ticksPerSecond = deltaTicks / deltaTime;
                currentRPM = (ticksPerSecond / TICKS_PER_REV) * 60.0;
            }

            // --- PID controller ---
            double error = targetRPM - currentRPM;
            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            double output = (kP * error) + (kI * integral) + (kD * derivative);

            // clamp output between 0 and 1
            double power = shooterOn ? Math.max(0.0, Math.min(output / 100.0, 1.0)) : 0.0;
            shooter.setPower(power);

            lastError = error;
            prevShooterPos = currentPos;
            prevTime = currentTime;

            // --- Telemetry ---
            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Error", "%.0f", error);
            telemetry.addData("Motor Power", "%.3f", power);
            telemetry.addData("PID", "P=%.5f I=%.6f D=%.5f", kP, kI, kD);
            telemetry.update();
        }
    }
}