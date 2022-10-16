package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Base64;

@TeleOp
public class Main extends OpMode {

    int servoInitialPosition;
    int motorInitialPosition;

    int encoderTick = 4096;

    DcMotor frontLeftServoEncoder;
    DcMotor frontRightServoEncoder;
    DcMotor backLeftServoEncoder;
    DcMotor backRightServoEncoder;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    Servo frontLeftServo;
    Servo frontRightServo;
    Servo backLeftServo;
    Servo backRightServo;

    // Front Left Servo Limits
    double fl_lowerLimit = 0;
    double fl_middle = 0.4;
    double fl_upperLimit = 0.75;
    // Front Right Servo Limits
    double fr_lowerLimit = 0;
    double fr_middle = 0.35;
    double fr_upperLimit = 0.75;
    // Back Left Servo Limits
    double bl_lowerLimit = 0.3;
    double bl_middle = 0.65;
    double bl_upperLimit = 1;
    // Back Right Servo Limits
    double br_lowerLimit = 0;
    double br_middle = 0.35;
    double br_upperLimit = 0.75;

    // Controller inputs
    double x = 0;
    double y = 0;

    boolean testWheelDirections = true;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        frontLeftServo = hardwareMap.get(Servo.class, "front_left");
        frontRightServo = hardwareMap.get(Servo.class, "front_right");
        backLeftServo = hardwareMap.get(Servo.class, "back_left");
        backRightServo = hardwareMap.get(Servo.class, "back_right");

        // Servo Encoders
        frontLeftServoEncoder = hardwareMap.get(DcMotor.class, "front_left_encoder");
        frontRightServoEncoder = hardwareMap.get(DcMotor.class, "front_right_encoder");
        backLeftServoEncoder = hardwareMap.get(DcMotor.class, "back_left_encoder");
        backRightServoEncoder = hardwareMap.get(DcMotor.class, "back_right_encoder");


        int servoInitialPosition = backLeftServoEncoder.getCurrentPosition();
        int motorInitialPosition = backLeftMotor.getCurrentPosition();
    }

    public double anglePositionVal(double lowerLimit, double middle, double upperLimit, boolean isForwardPointingWheel) {
        double position = 0;
        double distance = 0;

        // Two front wheels
        if (isForwardPointingWheel) {
            if (x == 0) {
                position = 0;
            }
            else {
                double alpha = Math.atan(y / x);

                // top left or bottom right directions, so would add from middle (but since angles are negative, do not need to add)
                if (x * y < 0) {
                    distance = ((alpha * 2) / Math.PI) * (upperLimit - middle);
                    position = middle - distance;

                    if (position > upperLimit) {
                        position = upperLimit;
                    }
                }

                // top right or bottom left direction, so would subtract from middle (since is positive)
                else {
                    distance = ((alpha * 2) / Math.PI) * (middle - lowerLimit);
                    position = middle - distance;

                    if (position < lowerLimit) {
                        position = lowerLimit;
                    }
                }
            }
        }

        else {
            if (x == 0) {
                position = middle;
            }
            else {
                double alpha = Math.atan(y / x);
                double beta = (Math.PI / 2) - Math.abs(alpha);  //beta is always positive

                // top left or bottom right directions, so would subtract from middle
                if (x * y < 0) {
                    distance = ((beta * 2) / Math.PI) * (middle - lowerLimit);
                    position = middle - distance;

                    if (position < lowerLimit) {
                        position = lowerLimit;
                    }
                }

                // top right or bottom left direction, so would add to middle
                else {
                    distance = ((beta * 2) / Math.PI) * (upperLimit - middle);
                    position = middle + distance;

                    if (position > upperLimit) {
                        position = upperLimit;
                    }
                }
            }
        }

        return position;
    }

    public double calculateSpeedVals(double x, double y, boolean isForwardPointingWheel) {
        double speed = Math.sqrt(x * x + y * y) / Math.sqrt(2);
        if (isForwardPointingWheel) {
            if (x < 0) {
                speed *= -1;
            }
        }
        else {
            if (y < 0) {
                speed *= -1;
            }
        }

        return speed;
    }

    @Override
    public void loop() {
        if (testWheelDirections) {
            wheelDirectionTest();
        }

        x = gamepad1.right_stick_x;
        y = -gamepad1.right_stick_y;

        double speed = Math.sqrt(x*x + y*y) / Math.sqrt(2);

        frontLeftServo.setPosition(anglePositionVal(fl_lowerLimit, fl_middle, fl_upperLimit, true));
        frontRightServo.setPosition(anglePositionVal(fr_lowerLimit, fr_middle, fr_upperLimit, true));
        backLeftServo.setPosition(anglePositionVal(bl_lowerLimit, bl_middle, bl_upperLimit, false));
        backRightServo.setPosition(anglePositionVal(br_lowerLimit, br_middle, br_upperLimit, false));


//        frontLeftMotor.setPower(calculateSpeedVals(x, y, true));
//        frontRightMotor.setPower(calculateSpeedVals(x, y, true));
//        backLeftMotor.setPower(calculateSpeedVals(x, y, false));
//        backRightMotor.setPower(-calculateSpeedVals(x, y, false));

        telemetry.addLine("Please work..." + x);
        telemetry.addLine("Servo encoder tick difference..." + (backLeftServoEncoder.getCurrentPosition() - servoInitialPosition));
        telemetry.addLine("Wheel encoder tick difference..." + (backLeftMotor.getCurrentPosition() - motorInitialPosition));
    }



    public void wheelDirectionTest() {
        String motorTested = "";

        if (gamepad1.a) {
            frontLeftMotor.setPower(0.25);
            motorTested = "front left";
        }
        else if (gamepad1.b) {
            frontRightMotor.setPower(0.25);
            motorTested = "front right";
        }
        else if (gamepad1.x) {
            backLeftMotor.setPower(0.25);
            motorTested = "back left";
        }
        else if (gamepad1.y) {
            backRightMotor.setPower(0.25);
            motorTested = "back right";
        }
        telemetry.addLine(motorTested);
    }
}