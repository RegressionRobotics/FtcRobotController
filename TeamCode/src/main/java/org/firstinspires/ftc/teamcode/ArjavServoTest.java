package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Shooter Sequence Test (With Arjav)", group = "Test")
public class ArjavServoTest extends LinearOpMode {

    private DcMotor shooter, intake;
    private CRServo leftTransfer, rightTransfer;
    private Servo arjav;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Mapping ---
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");
        arjav = hardwareMap.get(Servo.class, "arjav");

        telemetry.addLine("✅ Shooter + Arjav Test Ready");
        telemetry.addLine("Press PLAY to begin sequence.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Step 0: Activate Arjav servo ===
        telemetry.addData("Status", "Step 0: Setting Arjav servo to position 1");
        telemetry.update();
        arjav.setPosition(1.0);
        sleep(500); // small delay for servo to move

        // === Step 1: Power up the shooter motor ===
        telemetry.addData("Status", "Step 1: Spinning up shooter...");
        telemetry.update();
        shooter.setPower(0.85);

        // Wait 4 seconds for shooter to reach speed
        sleep(1000);

        // === Step 2: Feed first ball with transfer servos ===
        telemetry.addData("Status", "Step 2: Feeding first ball...");
        telemetry.update();
        leftTransfer.setPower(1.0);
        rightTransfer.setPower(1.0);
        sleep(100);

        // === Step 3: Run intake briefly to load second ball ===
        telemetry.addData("Status", "Step 3: Loading second ball...");
        telemetry.update();
        intake.setPower(0.85);
        sleep(250);


        // Keep shooter + transfers running for a while
        telemetry.addData("Status", "Step 4: Running shooter...");
        telemetry.update();

        sleep(8000);

        // === Step 5: Stop everything ===
        telemetry.addData("Status", "Step 5: Stopping all hardware.");
        telemetry.update();
        shooter.setPower(0);
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);



        telemetry.addLine("✅ Sequence Complete");
        telemetry.update();



    }
}
