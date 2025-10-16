// The package for your team's code
package org.firstinspires.ftc.teamcode;

// Imports needed for the OpMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

// Annotation to name the OpMode and put it in the TeleOp group
@TeleOp(name = "Shooter Sequence Test", group = "Tests")
public class ServoAndShooting extends LinearOpMode {

    // Declare your hardware variables
    DcMotor shooter;
    CRServo leftTransfer;
    CRServo rightTransfer;

    @Override
    public void runOpMode() {

        // --- HARDWARE MAPPING ---
        // Links the variables in the code to the names in the robot configuration.
        // Make sure these names ("shooter", "leftTransfer", "rightTransfer")
        // match your configuration file exactly!
        shooter = hardwareMap.dcMotor.get("shooter");
        leftTransfer = hardwareMap.crservo.get("leftTransfer");
        rightTransfer = hardwareMap.crservo.get("rightTransfer");

        // --- INITIALIZATION ---
        // It's good practice to set the motor mode. RUN_WITHOUT_ENCODER is best
        // for simple power control without PID.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Optional: If your servos need to spin in opposite directions to feed forward,
        // you can reverse one of them here.
        // rightTransfer.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Press START to run shooter sequence.");
        telemetry.update();

        // Wait for the user to press the START button on the Driver Station
        waitForStart();

        // This code will run only ONCE after you press START.
        if (opModeIsActive()) {

            // 1. Power up the shooter motor at full speed
            telemetry.addData("Status", "Spinning up shooter...");
            telemetry.update();
            shooter.setPower(0.8);

            // 2. Wait for 2 seconds (2000 milliseconds) for the motor to get to speed
            sleep(2000);

            // 3. Turn on the transfer servos to feed into the shooter
            telemetry.addData("Status", "Shooting!");
            telemetry.update();
            leftTransfer.setPower(1.0);
            rightTransfer.setPower(1.0);
        }

        // --- KEEP RUNNING ---
        // This loop keeps the OpMode alive and the motor/servos running
        // until you press the STOP button.
        while (opModeIsActive()) {
            // We can add telemetry here to monitor power levels
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.addData("Left Transfer Power", leftTransfer.getPower());
            telemetry.addData("Right Transfer Power", rightTransfer.getPower());
            telemetry.addData(">", "Press STOP to end.");
            telemetry.update();
        }

        // --- AFTER STOP ---
        // This code runs after you press STOP to ensure everything turns off.
        shooter.setPower(0);
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
    }
}