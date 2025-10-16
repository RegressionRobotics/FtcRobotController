package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Shooter (Timed Stop)", group = "Tests")
public class ServoAndShooting extends LinearOpMode {

    // Declare your hardware variables
    DcMotor shooter;
    CRServo leftTransfer;
    CRServo rightTransfer;

    @Override
    public void runOpMode() {

        // --- HARDWARE MAPPING ---
        shooter = hardwareMap.dcMotor.get("shooter");
        leftTransfer = hardwareMap.crservo.get("leftTransfer");
        rightTransfer = hardwareMap.crservo.get("rightTransfer");

        // --- INITIALIZATION ---
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Optional: If one servo spins the wrong way, uncomment the next line
        // rightTransfer.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Ready for Timed Auto");
        telemetry.addData(">", "Press START to run the sequence.");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS SEQUENCE ---
        if (opModeIsActive()) {

            // Step 1: Power up the shooter motor
            telemetry.addData("Status", "Step 1: Spinning up shooter...");
            telemetry.update();
            shooter.setPower(1.0);

            // Step 2: Wait 2 seconds for the motor to get to speed
            sleep(2000);

            // Step 3: Turn on the transfer servos to feed
            telemetry.addData("Status", "Step 2: Feeding into shooter...");
            telemetry.update();
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);

            // Step 4: Wait 5 seconds while everything is running
            sleep(5000);

            // Step 5: Stop all motors and servos
            telemetry.addData("Status", "Step 3: Stopping all hardware.");
            telemetry.update();
            shooter.setPower(0);
            leftTransfer.setPower(0);
            rightTransfer.setPower(0);

            // The OpMode will now automatically end.
        }
    }
}