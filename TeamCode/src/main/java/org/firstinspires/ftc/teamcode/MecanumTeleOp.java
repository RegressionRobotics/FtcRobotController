package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumTeleOp")

public class MecanumTeleOp extends LinearOpMode {
    Limelight3A limelight; //declare name of limelight
    @Override

    public void runOpMode() {
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
        //init ll3a
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
        //servos
        Servo transfer = hardwareMap.servo.get("transfer");
        CRServo intake = hardwareMap.crservo.get("intake");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double ex = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //mecanum code stuff v
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //vert slide code stuff v
            rightVertSlide.setPower(ex);
            leftVertSlide.setPower(ex);
            rightVertSlide.setPower(rt);
            leftVertSlide.setPower(rt);


        }
    }
}