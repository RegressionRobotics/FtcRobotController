package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumTeleOp")

public class MecanumTeleOp extends LinearOpMode {

    @Override

    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        //motor declarations
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Reverse everything on the left side
        //init ll3a

        //servos
        CRServo leftTransfer = hardwareMap.crservo.get("leftTransfer");
        CRServo rightTransfer = hardwareMap.crservo.get("rightTransfer");
        Servo arjav = hardwareMap.servo.get("arjav");
        //controller stuff
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //mecanum code stuff v
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double transfer = gamepad1.right_trigger;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            rightTransfer.setPower(transfer);
            leftTransfer.setPower(transfer);

            //intake things v
            if (currentGamepad1.a && !previousGamepad1.a) { //extend slides, flip down transfer, turn on intake
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                intake.setPower(1.0);
            }
            if (!currentGamepad1.a && previousGamepad1.a) { //turn off intake, flip up transfer, retract slides
                intake.setPower(0);
                gamepad1.stopRumble();
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) { //extend slides, flip down transfer, turn on intake
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                shooter.setPower(0.8);
            }
            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) { //turn off intake, flip up transfer, retract slides
                shooter.setPower(0);
                gamepad1.stopRumble();
            }
            if (currentGamepad1.left_bumper) {
            rightTransfer.setPower(1.0);
            leftTransfer.setPower(1.0);
            arjav.setPosition(1);
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
            rightTransfer.setPower(0);
            leftTransfer.setPower(0);
            arjav.setPosition(0.5);
            gamepad1.stopRumble();
            }
        }
    }
}