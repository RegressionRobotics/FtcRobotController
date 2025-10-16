package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arjav Servo Test", group = "Test")
public class ArjavServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Mapping ---
        Servo arjav = hardwareMap.get(Servo.class, "arjav");

        telemetry.addLine("✅ Arjav Servo Initialized");
        telemetry.addLine("Press PLAY to start test.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Step 1: Moving to position 1.0");
        telemetry.update();
        arjav.setPosition(1.0);
        sleep(5000); // wait 5 seconds

        telemetry.addLine("Step 2: Moving back to position 0.5");
        telemetry.update();
        arjav.setPosition(0.5);
        sleep(1000); // wait a bit before stopping

        telemetry.addLine("✅ Test Complete");
        telemetry.update();

        // Keep OpMode alive for viewing telemetry
        while (opModeIsActive()) {
            telemetry.addData("Current Position", arjav.getPosition());
            telemetry.update();
            sleep(100);
        }
    }
}
