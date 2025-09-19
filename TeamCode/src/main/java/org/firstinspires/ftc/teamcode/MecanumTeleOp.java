package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumTeleOp")

public class MecanumTeleOp extends LinearOpMode {

    @Override

    public void runOpMode() {
        //limits for horz ext
        int horzext = 600;
        int horzret = 0;
        // Declare our motors
        // Make sure your ID's match your configuration
        //motor declarations
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor leftVertSlide = hardwareMap.dcMotor.get("leftVert");
        DcMotor rightVertSlide = hardwareMap.dcMotor.get("rightVert");
        DcMotor rightHorzSlide = hardwareMap.dcMotor.get("rightHorz");
        DcMotor leftHorzSlide = hardwareMap.dcMotor.get("leftHorz");
        //Reverse everything on the left side
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftHorzSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //init ll3a

        //servos
        Servo transfer = hardwareMap.servo.get("transfer");
        CRServo intake = hardwareMap.crservo.get("intake");
        //controller stuff
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        //encoder stuff for horzes
        leftHorzSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorzSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorzSlide.setTargetPosition(horzret);
        rightHorzSlide.setTargetPosition(horzret);
        leftHorzSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightHorzSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
            double rt = -gamepad1.left_trigger;
            double ex = gamepad1.right_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //mecanum code stuff v
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //vert slide code stuff v
            rightVertSlide.setPower(ex);
            leftVertSlide.setPower(ex);
            rightVertSlide.setPower(rt);
            leftVertSlide.setPower(rt);

            //intake things v
            if (currentGamepad1.a && !previousGamepad1.a) { //extend slides, flip down transfer, turn on intake
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                leftHorzSlide.setTargetPosition(horzext);
                leftHorzSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHorzSlide.setPower(1.0);
                rightHorzSlide.setTargetPosition(horzext);
                rightHorzSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHorzSlide.setPower(1.0);
                intake.setPower(1);
                transfer.setPosition(0);
            }
            if (!currentGamepad1.a && previousGamepad1.a) { //turn off intake, flip up transfer, retract slides
                intake.setPower(0);
                transfer.setPosition(1);
                leftHorzSlide.setTargetPosition(horzret);
                leftHorzSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHorzSlide.setPower(1.0);
                rightHorzSlide.setTargetPosition(horzret);
                rightHorzSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHorzSlide.setPower(1.0);
                gamepad1.stopRumble();
            }
            int left_position = leftHorzSlide.getCurrentPosition();
            telemetry.addData("Left Position", left_position);
            int right_position = rightHorzSlide.getCurrentPosition();
            telemetry.addData("Right Position", right_position);
            telemetry.update();

        }
    }
}