package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auton1")
public class Auton1  extends LinearOpMode {

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

        }


    }
}
